"""ROS-independent regressions for the offline mapping workflow."""
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from agt_mapping_bringup.preflight import (
    PreflightError, check_output, frontend_remappings, inspect_bag, validate_number,
)
from agt_mapping_bringup.session_state import (
    create_session, mark_session, playback_action,
)
from agt_mapping_bringup.session_runtime import wait_until
from agt_mapping_bringup.session_lock import acquire_domain_lease


class BagFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'recording with spaces'
        self.bag.mkdir()
        (self.bag / 'scan_0.db3').write_bytes(b'recorded data')
        self.info = {
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': 12_500_000_000},
            'message_count': 110,
            'relative_file_paths': ['scan_0.db3'],
            'topics_with_message_count': [
                {'topic_metadata': {'name': '/livox/lidar',
                                    'type': 'livox_ros_driver2/msg/CustomMsg'},
                 'message_count': 10},
                {'topic_metadata': {'name': '/livox/imu',
                                    'type': 'sensor_msgs/msg/Imu'},
                 'message_count': 100},
            ],
        }
        self.save_metadata()

    def save_metadata(self):
        (self.bag / 'metadata.yaml').write_text(
            yaml.safe_dump({'rosbag2_bagfile_information': self.info}),
            encoding='utf-8')

    def test_raw_topics_are_detected_without_ros(self):
        bag = inspect_bag(self.bag)
        self.assertEqual(bag.lidar_topic, '/livox/lidar')
        self.assertEqual(bag.imu_topic, '/livox/imu')
        self.assertEqual(bag.duration_seconds, 12.5)
        self.assertEqual(bag.message_count, 110)

    def test_normalized_topics_are_detected(self):
        self.info['topics_with_message_count'][0]['topic_metadata']['name'] = '/agt/sensors/lidar/custom'
        self.info['topics_with_message_count'][1]['topic_metadata']['name'] = '/agt/sensors/imu/data'
        self.save_metadata()
        self.assertEqual(inspect_bag(self.bag).imu_topic, '/agt/sensors/imu/data')

    def test_ambiguous_sensor_requires_explicit_topic(self):
        self.info['topics_with_message_count'].append({
            'topic_metadata': {'name': '/second/imu', 'type': 'sensor_msgs/msg/Imu'},
            'message_count': 20})
        self.save_metadata()
        with self.assertRaisesRegex(PreflightError, 'imu-topic'):
            inspect_bag(self.bag)
        self.assertEqual(inspect_bag(self.bag, imu_topic='/second/imu').imu_topic, '/second/imu')

    def test_wrong_requested_topic_type_is_rejected(self):
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag, lidar_topic='/livox/imu')

    def test_empty_sensor_stream_is_rejected(self):
        self.info['topics_with_message_count'][0]['message_count'] = 0
        self.save_metadata()
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_missing_split_storage_file_is_rejected(self):
        self.info['relative_file_paths'].append('missing_1.db3')
        self.save_metadata()
        with self.assertRaisesRegex(PreflightError, 'missing_1.db3'):
            inspect_bag(self.bag)

    def test_storage_path_cannot_escape_bag(self):
        (self.root / 'outside.db3').write_bytes(b'data')
        self.info['relative_file_paths'] = ['../outside.db3']
        self.save_metadata()
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_absolute_storage_path_is_rejected(self):
        self.info['relative_file_paths'] = [str(self.bag / 'scan_0.db3')]
        self.save_metadata()
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_wrong_metadata_document_is_rejected(self):
        (self.bag / 'metadata.yaml').write_text('backend: PGO\n')
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_invalid_yaml_has_actionable_error(self):
        (self.bag / 'metadata.yaml').write_text('x: [unterminated')
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_metadata_type_error_is_not_an_attribute_error(self):
        self.info['topics_with_message_count'] = [None]
        self.save_metadata()
        with self.assertRaises(PreflightError):
            inspect_bag(self.bag)

    def test_output_preflight_has_no_side_effects(self):
        output = self.root / 'new' / 'output'
        self.assertEqual(check_output(output, self.bag), output)
        self.assertFalse(output.exists())

    def test_output_cannot_modify_source_bag(self):
        with self.assertRaises(PreflightError):
            check_output(self.bag / 'output', self.bag)

    def test_nonempty_output_is_never_reused(self):
        output = self.root / 'existing'
        output.mkdir()
        sentinel = output / 'old-map.pcd'
        sentinel.write_bytes(b'original')
        with self.assertRaises(PreflightError):
            check_output(output, self.bag)
        self.assertEqual(sentinel.read_bytes(), b'original')

    def test_empty_existing_output_is_allowed(self):
        output = self.root / 'empty'
        output.mkdir()
        self.assertEqual(check_output(output, self.bag), output)

    def test_fastlio_yaml_topic_is_explicitly_remapped(self):
        config = self.root / 'lio.yaml'
        content = ('imu_topic: /agt/sensors/imu/data\n'
                   'lidar_topic: /mapping/sensor/livox\n'
                   'cube_len: 300\ndet_range: 60\nmove_thresh: 1.5\n')
        config.write_text(content)
        self.assertEqual(frontend_remappings(config, '/livox/imu'), [
            ('/agt/sensors/imu/data', '/livox/imu'),
            ('/mapping/sensor/livox', '/mapping/sensor/livox')])
        self.assertEqual(config.read_text(), content)

    def test_fastlio_yaml_rejects_local_cube_that_moves_every_scan(self):
        config = self.root / 'lio.yaml'
        config.write_text('imu_topic: /imu\nlidar_topic: /lidar\n'
                          'cube_len: 200\ndet_range: 300\nmove_thresh: 1.5\n')
        with self.assertRaisesRegex(PreflightError, 'local map is unstable'):
            frontend_remappings(config, '/livox/imu')

    def test_output_path_unrepresentable_in_backend_status_is_rejected(self):
        with self.assertRaises(PreflightError):
            check_output(self.root / 'quoted"name', self.bag)

    def test_same_domain_cannot_run_two_mapping_sessions(self):
        with acquire_domain_lease(89, directory=self.root):
            with self.assertRaisesRegex(PreflightError, 'domain 89'):
                acquire_domain_lease(89, directory=self.root)
        with acquire_domain_lease(89, directory=self.root):
            pass

    def test_different_domains_have_independent_leases(self):
        with acquire_domain_lease(89, directory=self.root):
            with acquire_domain_lease(90, directory=self.root):
                pass

    def test_domain_lock_does_not_follow_symlinks(self):
        import os
        original = self.root / 'original'
        original.write_text('do not modify')
        (self.root / f'agt-mapping-{os.getuid()}-domain-89.lock').symlink_to(original)
        with self.assertRaises(OSError):
            acquire_domain_lease(89, directory=self.root)
        self.assertEqual(original.read_text(), 'do not modify')

    def test_session_is_reserved_and_recorded(self):
        output = self.root / 'session'
        bag = inspect_bag(self.bag)
        create_session(output, bag, {'playback_rate': 1.0})
        mark_session(output, 'replaying', 'input ready')
        record = json.loads((output / 'session.json').read_text())
        self.assertEqual(record['status'], 'replaying')
        self.assertEqual(record['bag_path'], str(self.bag))
        self.assertEqual(len(record['history']), 2)
        with self.assertRaises((PreflightError, FileExistsError)):
            create_session(output, bag, {})

    def test_shutdown_does_not_overwrite_verified_result(self):
        output = self.root / 'session'
        create_session(output, inspect_bag(self.bag), {})
        mark_session(output, 'completed', 'verified')
        mark_session(output, 'cancelled', 'shutdown')
        self.assertEqual(json.loads((output / 'session.json').read_text())['status'], 'completed')


class LifecyclePolicyTests(unittest.TestCase):
    def test_successful_playback_exports(self):
        self.assertEqual(playback_action(0, False, True), 'export')

    def test_failed_playback_never_exports(self):
        for code in (1, 2, 130, -2, -15):
            with self.subTest(code=code):
                self.assertEqual(playback_action(code, False, True), 'fail')

    def test_shutdown_never_exports_even_if_player_returns_zero(self):
        self.assertEqual(playback_action(0, True, True), 'stop')
        self.assertEqual(playback_action(-2, True, True), 'stop')

    def test_manual_mode_does_not_auto_export(self):
        self.assertEqual(playback_action(0, False, False), 'manual')

    def test_numeric_arguments_reject_invalid_values(self):
        for value in ('nan', 'inf', '-inf', 0, -1, 'oops'):
            with self.subTest(value=value), self.assertRaises(PreflightError):
                validate_number(value, 'rate')
        self.assertEqual(validate_number(0, 'drain', allow_zero=True), 0)
        self.assertEqual(validate_number('2.5', 'rate'), 2.5)

    def test_wall_clock_wait_times_out_even_without_simulated_clock(self):
        now = [0.0]
        def spin(seconds):
            now[0] += seconds
        with self.assertRaisesRegex(TimeoutError, 'service'):
            wait_until(lambda: False, spin, 1.0, 'service', now=lambda: now[0])
        self.assertLessEqual(now[0], 1.01)

    def test_wall_clock_wait_returns_on_readiness(self):
        now = [0.0]
        wait_until(lambda: now[0] >= 0.2, lambda dt: now.__setitem__(0, now[0] + dt),
                   1.0, 'ready', now=lambda: now[0])
        self.assertLess(now[0], 1.0)


if __name__ == '__main__':
    unittest.main()
