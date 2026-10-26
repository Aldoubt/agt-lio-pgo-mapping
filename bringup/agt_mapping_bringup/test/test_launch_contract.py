"""Launch policy contract tests using stand-ins, NOT ROS integration tests."""
import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

from backend_fixture import backend_fixture
from agt_mapping_bringup.backend_registry import resolve_backend


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Process(Action):
    pass


class Node(Action):
    pass


class Registration(Action):
    pass


class ProcessExit(Action):
    pass


class ShutdownHandler(Action):
    pass


class Failure(Action):
    pass


class Emit(Action):
    pass


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.values[self.name]


def module(name, **attributes):
    result = types.ModuleType(name)
    result.__dict__.update(attributes)
    return result


class LaunchContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'bag'
        self.bag.mkdir()
        (self.bag / 'data.db3').write_bytes(b'fixture')
        (self.bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3', 'message_count': 2,
            'relative_file_paths': ['data.db3'],
            'topics_with_message_count': [
                {'topic_metadata': {'name': '/livox/lidar', 'type': 'livox_ros_driver2/msg/CustomMsg'},
                 'message_count': 1},
                {'topic_metadata': {'name': '/livox/imu', 'type': 'sensor_msgs/msg/Imu'},
                 'message_count': 1},
            ]}}))
        self.output = self.root / 'output'
        self.context = types.SimpleNamespace(is_shutdown=False, values={
            'bag_path': str(self.bag), 'output_dir': str(self.output),
            'lidar_topic': 'auto', 'imu_topic': 'auto', 'playback_rate': '1.0',
            'startup_timeout': '45', 'export_timeout': '180', 'drain_seconds': '3',
            'start_rviz': 'false', 'start_paused': 'false', 'auto_export': 'true', 'keep_open': 'false',
            'reference_mode': 'frontend', 'mapping_backend': '', 'backend_profile_source_sha256': '', 'backend_profile': '', 'fastlio_config': '',
        })
        share = Path(__file__).resolve().parents[1]
        replacements = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages',
                get_package_share_directory=lambda _: str(share)),
            'launch': module('launch'),
            'launch.actions': module('launch.actions', EmitEvent=Emit, ExecuteProcess=Process,
                LogInfo=Action, OpaqueFunction=Failure, RegisterEventHandler=Registration, IncludeLaunchDescription=Action),
            'launch.event_handlers': module('launch.event_handlers',
                OnProcessExit=ProcessExit, OnShutdown=ShutdownHandler),
            'launch.events': module('launch.events', Shutdown=Action),
            'launch.events.process': module('launch.events.process', SignalProcess=Action),
            'launch.launch_description_sources': module('launch.launch_description_sources', PythonLaunchDescriptionSource=Action),
            'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Configuration),
            'launch_ros': module('launch_ros'),
            'launch_ros.actions': module('launch_ros.actions', Node=Node),
            'launch_ros.parameter_descriptions': module('launch_ros.parameter_descriptions', ParameterValue=Action),
        }
        self.modules = patch.dict(sys.modules, replacements)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        name = 'agt_mapping_bringup.session_launch'
        sys.modules.pop(name, None)
        self.launch = importlib.import_module(name)
        profile_path = backend_fixture(self.root)
        registry = patch.object(self.launch, 'resolve_backend',
                                side_effect=lambda backend_id, **kw: resolve_backend(
                                    backend_id, profile_dir=profile_path.parent, **kw))
        registry.start()
        self.addCleanup(registry.stop)
        lease = patch.object(self.launch, 'acquire_domain_lease',
                             return_value=types.SimpleNamespace(close=lambda: None))
        lease.start()
        self.addCleanup(lease.stop)
        self.addCleanup(lambda: sys.modules.pop(name, None))

    def compose(self, **overrides):
        self.context.values.update(overrides)
        actions = self.launch.launch_session(self.context)
        self.handlers = [a.args[0] for a in actions if isinstance(a, Registration)]
        return actions

    def callback(self, target):
        for handler in self.handlers:
            if not isinstance(handler, ProcessExit):
                continue
            action = handler.kwargs['target_action']
            if (target == 'player' and isinstance(action, Process)) or action.kwargs.get('executable') == target:
                return handler.kwargs['on_exit']
        self.fail(f'No handler for {target}')

    def event(self, code):
        return types.SimpleNamespace(returncode=code)

    def test_player_is_gated_until_readiness_success(self):
        actions = self.compose()
        self.assertFalse(any(isinstance(a, Process) for a in actions))
        following = self.callback('mapping_wait_ready')(self.event(0), self.context)
        self.assertTrue(any(isinstance(a, Process) for a in following))

    def test_readiness_failure_aborts_without_playback(self):
        self.compose()
        following = self.callback('mapping_wait_ready')(self.event(1), self.context)
        self.assertTrue(any(isinstance(a, Failure) for a in following))
        self.assertFalse(any(isinstance(a, Process) for a in following))

    def test_failed_player_never_launches_exporter(self):
        self.compose()
        following = self.callback('player')(self.event(2), self.context)
        self.assertTrue(any(isinstance(a, Failure) for a in following))
        self.assertFalse(any(isinstance(a, Node) for a in following))
        self.assertEqual(json.loads((self.output / 'session.json').read_text())['status'], 'failed')

    def test_shutdown_even_with_zero_player_exit_never_exports(self):
        self.compose()
        self.context.is_shutdown = True
        self.assertEqual(self.callback('player')(self.event(0), self.context), [])

    def test_successful_player_starts_finalizer_not_shutdown(self):
        self.compose()
        following = self.callback('player')(self.event(0), self.context)
        self.assertEqual(following[0].kwargs['executable'], 'mapping_export_verified')
        self.assertFalse(any(isinstance(a, Emit) for a in following))

    def test_finalizer_failure_propagates_launch_failure(self):
        self.compose()
        following = self.callback('mapping_export_verified')(self.event(1), self.context)
        self.assertTrue(any(isinstance(a, Failure) for a in following))

    def test_successful_finalizer_closes_launch_by_default(self):
        self.compose()
        following = self.callback('mapping_export_verified')(self.event(0), self.context)
        self.assertTrue(any(isinstance(a, Emit) for a in following))

    def test_keep_open_is_explicit(self):
        self.compose(keep_open='true')
        following = self.callback('mapping_export_verified')(self.event(0), self.context)
        self.assertFalse(any(isinstance(a, Emit) for a in following))

    def test_manual_mode_never_starts_automatic_export(self):
        self.compose(auto_export='false')
        following = self.callback('player')(self.event(0), self.context)
        self.assertFalse(any(isinstance(a, Node) for a in following))
        self.assertEqual(json.loads((self.output / 'session.json').read_text())['status'], 'waiting_manual_export')

    def test_critical_node_zero_exit_is_still_unexpected(self):
        self.compose()
        following = self.callback('fastlivo_mapping')(self.event(0), self.context)
        self.assertTrue(any(isinstance(a, Failure) for a in following))

    def test_fastlivo_uses_profile_and_selected_bag_topics(self):
        actions = self.compose()
        lio = next(a for a in actions if isinstance(a, Node) and a.kwargs.get('executable') == 'fastlivo_mapping')
        self.assertEqual(lio.kwargs['package'], 'fast_livo')
        self.assertEqual(len(lio.kwargs['parameters']), 1)
        native_path = Path(lio.kwargs['parameters'][0])
        parameters = yaml.safe_load(native_path.read_bytes())['/**']['ros__parameters']
        self.assertEqual(parameters['common']['lid_topic'], '/livox/lidar')
        self.assertEqual(parameters['common']['imu_topic'], '/livox/imu')
        self.assertTrue(parameters['use_sim_time'])
        exporter = next(a for a in actions if isinstance(a, Node) and a.kwargs.get('executable') == 'frontend_map_exporter')
        from agt_mapping_bringup.backend_snapshot import load_backend_snapshot
        settings = exporter.kwargs['parameters'][0]
        frozen = load_backend_snapshot(settings['profile_path'], settings['profile_sha256'])
        self.assertEqual(frozen.config_path, native_path)

    def test_fastlivo_source_mode_does_not_launch_fastlio2_or_pgo(self):
        actions = self.compose(reference_mode='frontend')
        executables = [a.kwargs.get('executable') for a in actions if isinstance(a, Node)]
        self.assertIn('fastlivo_mapping', executables)
        self.assertIn('frontend_map_exporter', executables)
        self.assertNotIn('lio_node', executables)
        self.assertNotIn('fastlio_reference_exporter', executables)
        self.assertNotIn('pgo_node', executables)
        self.assertNotIn('pgo_backend_node', executables)
        self.assertNotIn('mapping_artifact_exporter', executables)
        session = json.loads((self.output / 'session.json').read_text())
        self.assertEqual(session['options']['reference_mode'], 'frontend')
        self.assertEqual(session['options']['mapping_backend'], 'fast_livo2_lio')

    def test_live_and_yhs_share_frozen_native_adapter_and_export_contract(self):
        from test_live_mode import livox_config
        from agt_mapping_bringup.backend_snapshot import load_backend_snapshot
        name = 'agt_mapping_bringup.live_launch'
        sys.modules.pop(name, None)
        implementation = importlib.import_module(name)
        self.addCleanup(lambda: sys.modules.pop(name, None))
        config = self.root / 'livox.json'
        config.write_text(json.dumps(livox_config()))
        for sensor_only in (False, True):
            with self.subTest(sensor_only=sensor_only):
                output = self.root / ('yhs' if sensor_only else 'live')
                self.context.values.update({
                    'livox_config': str(config), 'output_dir': str(output),
                    'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
                    'publish_freq': '10', 'frame_id': 'livox_frame',
                    'duration_seconds': '0', 'sensor_stall_seconds': '5',
                    'backend_profile': str(self.root / 'fast_livo2_lio.yaml'),
                })
                packages = []
                def package_share(package):
                    packages.append(package)
                    return str(self.root)
                with patch.object(implementation, 'acquire_domain_lease',
                                  return_value=types.SimpleNamespace(close=lambda: None)), \
                     patch.object(implementation, 'get_package_share_directory', side_effect=package_share):
                    actions = implementation.launch_live_session(self.context, sensor_only=sensor_only)
                nodes = {a.kwargs.get('executable'): a for a in actions if isinstance(a, Node)}
                self.assertNotIn('pgo_node', nodes)
                self.assertNotIn('lio_node', nodes)
                settings = nodes['frontend_map_exporter'].kwargs['parameters'][0]
                frozen = load_backend_snapshot(settings['profile_path'], settings['profile_sha256'])
                self.assertEqual(nodes['fastlivo_mapping'].kwargs['parameters'], [str(frozen.config_path)])
                native = yaml.safe_load(frozen.config_bytes)['/**']['ros__parameters']
                self.assertFalse(native['use_sim_time'])
                adapter = nodes['mapping_frontend_adapter_node'].kwargs['parameters'][0]
                self.assertEqual(adapter['T_body_lidar.x'], frozen.profile['sensor']['T_body_lidar_translation_m'][0])
                self.assertEqual(adapter['body_frame'], 'livox_imu')
                self.assertEqual(native['common']['lid_topic'], adapter['input_lidar_topic'])
                if sensor_only:
                    self.assertNotIn('agt_robot_bringup', packages)
                    self.assertIn('livox_ros_driver2_node', nodes)
                    self.assertNotIn('robot_state_publisher', nodes)

    def test_profile_change_after_cli_preflight_fails_before_startup(self):
        with self.assertRaisesRegex(ValueError, 'changed after CLI preflight'):
            self.compose(backend_profile_source_sha256='0' * 64)
        self.assertFalse(self.output.exists())

    def test_retired_launch_argument_fails_before_output_reservation(self):
        with self.assertRaisesRegex(ValueError, 'retired'):
            self.compose(fastlio_config='/mounted/old-fastlio.yaml')
        self.assertFalse(self.output.exists())

    def test_paused_and_rate_are_forwarded_to_player_argv(self):
        self.compose(start_paused='true', playback_rate='2.5')
        following = self.callback('mapping_wait_ready')(self.event(0), self.context)
        player = next(a for a in following if isinstance(a, Process))
        command = player.kwargs['cmd']
        self.assertIn('--start-paused', command)
        self.assertEqual(command[command.index('--rate') + 1], '2.5')
        self.assertEqual(command[command.index('--topics') + 1:command.index('--start-paused')],
                         ['/livox/lidar', '/livox/imu'])


if __name__ == '__main__':
    unittest.main()
