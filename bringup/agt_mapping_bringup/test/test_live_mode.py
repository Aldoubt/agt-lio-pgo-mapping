"""ROS-independent regressions for the live MID360 mapping mode."""
import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from agt_mapping_bringup import cli
from agt_mapping_bringup.live_source import inspect_live_config
from agt_mapping_bringup.preflight import PreflightError
from agt_mapping_bringup.session_state import create_session

REPOSITORY = Path(__file__).resolve().parents[3]


def livox_config(host='192.168.1.5', lidar='192.168.1.117', lidars=1):
    return {
        'lidar_summary_info': {'lidar_type': 8},
        'MID360': {
            'lidar_net_info': {'cmd_data_port': 56100, 'push_msg_port': 56200, 'point_data_port': 56300,
                               'imu_data_port': 56400, 'log_data_port': 56500},
            'host_net_info': {'cmd_data_ip': host, 'cmd_data_port': 56101, 'push_msg_ip': host,
                              'push_msg_port': 56201, 'point_data_ip': host, 'point_data_port': 56301,
                              'imu_data_ip': host, 'imu_data_port': 56401, 'log_data_ip': '',
                              'log_data_port': 56501},
        },
        'lidar_configs': [{'ip': lidar, 'pcl_data_type': 1, 'pattern_mode': 0}] * lidars,
    }


class LiveSourceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / 'MID360_config.json'
        self.config.write_text(json.dumps(livox_config()))

    def test_valid_single_mid360_config(self):
        source = inspect_live_config(self.config)
        self.assertEqual((source.host_ip, source.lidar_ip), ('192.168.1.5', '192.168.1.117'))
        self.assertEqual((source.lidar_topic, source.imu_topic), ('/livox/lidar', '/livox/imu'))
        self.assertEqual(source.storage_identifier, 'live')

    def test_rejects_multi_lidar_and_bad_network(self):
        self.config.write_text(json.dumps(livox_config(lidars=2)))
        with self.assertRaisesRegex(PreflightError, 'Exactly one'):
            inspect_live_config(self.config)
        self.config.write_text(json.dumps(livox_config(host='192.168.1.5', lidar='192.168.1.5')))
        with self.assertRaisesRegex(PreflightError, 'must differ'):
            inspect_live_config(self.config)
        self.config.write_text(json.dumps(livox_config(host='not-an-ip')))
        with self.assertRaisesRegex(PreflightError, 'invalid'):
            inspect_live_config(self.config)

    def test_rejects_unsupported_topics_and_frequency(self):
        with self.assertRaisesRegex(PreflightError, 'multi_topic'):
            inspect_live_config(self.config, lidar_topic='/other/lidar')
        with self.assertRaisesRegex(PreflightError, 'publish-freq'):
            inspect_live_config(self.config, publish_freq=7.0)

    def test_session_journal_accepts_live_source(self):
        source = inspect_live_config(self.config)
        output = self.root / 'run'
        record = create_session(output, source, {'mode': 'live'})
        self.assertEqual(record['status'], 'preparing')
        self.assertEqual(record['bag_path'], str(self.config))
        self.assertTrue((output / 'session.json').is_file())


class LiveCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / 'MID360_config.json'
        self.config.write_text(json.dumps(livox_config()))
        self.output = self.root / 'live output'
        self.environment = patch.dict(os.environ, {'AGT_MAPPING_REPOSITORY': str(REPOSITORY)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def invoke(self, options):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = cli.main(options)
        return code, stdout.getvalue(), stderr.getvalue()

    def test_live_dry_run_plans_driver_and_raw_bag(self):
        code, output, errors = self.invoke(['--live', str(self.output), '--livox-config', str(self.config),
                                            '--dry-run', '--no-rviz', '--duration', '30'])
        self.assertEqual((code, errors), (0, ''))
        plan = json.loads(output)
        self.assertEqual(plan['mode'], 'dry-run')
        self.assertEqual(plan['raw_bag'], str(self.output / 'raw_bag'))
        self.assertIn('mapping_live_mid360.launch.py', plan['command'])
        self.assertIn('duration_seconds:=30.0', plan['command'])
        self.assertIn('start_rviz:=false', plan['command'])
        self.assertIn('livox_config:=' + str(self.config), plan['command'])
        self.assertEqual(plan['environment']['ROS_LOCALHOST_ONLY'], '1')
        self.assertFalse(self.output.exists())

    def test_yhs_live_refuses_bunker_default_and_plans_sensor_only(self):
        code, _, error = self.invoke(['--live', str(self.output), '--robot', 'yhs_v1',
                                      '--dry-run', '--no-rviz'])
        self.assertEqual(code, 2)
        self.assertIn('requires --livox-config', error)
        code, output, error = self.invoke(['--live', str(self.output), '--robot', 'yhs_v1',
                                           '--livox-config', str(self.config), '--dry-run', '--no-rviz'])
        self.assertEqual((code, error), (0, ''))
        plan = json.loads(output)
        self.assertEqual(plan['robot'], 'yhs_v1')
        self.assertTrue(plan['sensor_only_no_base'])
        self.assertIn('mapping_live_yhs_mid360.launch.py', plan['command'])
        self.assertFalse(self.output.exists())

    def test_live_rejects_bag_and_replay_only_flags(self):
        bag = self.root / 'bag'
        bag.mkdir()
        code, _, error = self.invoke(['--live', str(bag), str(self.output), '--dry-run'])
        self.assertEqual(code, 2)
        self.assertIn('--live does not take a bag', error)
        code, _, error = self.invoke(['--live', str(self.output), '--livox-config', str(self.config),
                                      '--start-paused', '--dry-run'])
        self.assertEqual(code, 2)
        self.assertIn('bag replay only', error)

    def test_bag_mode_still_requires_a_bag(self):
        code, _, error = self.invoke(['--dry-run'])
        self.assertEqual(code, 2)
        self.assertIn('--live', error)

    def test_live_launch_uses_argv_forwarding(self):
        ros, overlay = self.root / 'ros.bash', self.root / 'overlay.bash'
        ros.write_text('# fixture\n')
        overlay.write_text('# fixture\n')
        with patch.object(cli.os, 'execvpe') as execute:
            code, _, _ = self.invoke(['--live', str(self.output), '--livox-config', str(self.config),
                                      '--ros-setup', str(ros), '--setup', str(overlay), '--no-rviz'])
        self.assertEqual(code, 0)
        executable, argv, environment = execute.call_args.args
        self.assertEqual(executable, '/bin/bash')
        self.assertNotIn('-c', argv)
        self.assertIn('mapping_live_mid360.launch.py', argv)
        self.assertEqual(environment['ROS_DOMAIN_ID'], '89')


class LiveLaunchHelpersTests(unittest.TestCase):
    def test_yhs_sensor_parameters_have_no_base_driver_or_bunker_tf(self):
        from agt_mapping_bringup.yhs_live_sensor import sensor_parameters
        source = type('Sensor', (), {'path': Path('/tmp/yhs-mid360.json'), 'publish_freq': 10.0})()
        params = sensor_parameters(source, 'livox_frame')
        self.assertEqual(params['user_config_path'], '/tmp/yhs-mid360.json')
        self.assertEqual(params['xfer_format'], 1)
        self.assertEqual(params['multi_topic'], 0)
        self.assertNotIn('can', ' '.join(params))
        self.assertNotIn('robot_description', ' '.join(params))
        live_source = (Path(__file__).resolve().parents[1] /
                       'agt_mapping_bringup' / 'live_launch.py').read_text()
        self.assertIn('if sensor_only:', live_source)
        self.assertIn('make_sensor_node', live_source)
        launch_source = (Path(__file__).resolve().parents[1] /
                         'launch' / 'mapping_live_yhs_mid360.launch.py').read_text()
        self.assertIn('sensor_only=True', launch_source)
        self.assertNotIn('bunker_v1', launch_source)

    def test_yhs_launch_requires_live_runtime_preflight(self):
        wrapper = (Path(__file__).resolve().parents[3] /
                   'scripts' / 'mapping_launch_env.sh').read_text()
        self.assertIn('mapping_live_yhs_mid360.launch.py', wrapper)
        self.assertIn('helpers+=(mapping_live_supervisor)', wrapper)
        self.assertIn('ros2 pkg prefix livox_ros_driver2', wrapper)

    def test_live_mapping_uses_robot_hardware_owner(self):
        try:
            from agt_mapping_bringup.live_launch import raw_record_command
        except ImportError:
            self.skipTest('launch/ament_index not importable without ROS')
        source = (Path(__file__).resolve().parents[1] /
                  'agt_mapping_bringup' / 'live_launch.py').read_text()
        self.assertIn("get_package_share_directory('agt_robot_bringup')", source)
        self.assertIn("'mid360_driver_mode': 'mapping_custom'", source)
        self.assertNotIn("Node(package='livox_ros_driver2'", source)
        command = raw_record_command('/tmp/run', '/livox/lidar', '/livox/imu')
        self.assertEqual(command[:3], ['ros2', 'bag', 'record'])
        self.assertIn('/tmp/run/raw_bag', command)
        self.assertIn('/livox/imu', command)


if __name__ == '__main__':
    unittest.main()
