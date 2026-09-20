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
        })
        share = Path(__file__).resolve().parents[1]
        replacements = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages',
                get_package_share_directory=lambda _: str(share)),
            'launch': module('launch'),
            'launch.actions': module('launch.actions', EmitEvent=Emit, ExecuteProcess=Process,
                LogInfo=Action, OpaqueFunction=Failure, RegisterEventHandler=Registration),
            'launch.event_handlers': module('launch.event_handlers',
                OnProcessExit=ProcessExit, OnShutdown=ShutdownHandler),
            'launch.events': module('launch.events', Shutdown=Action),
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
        following = self.callback('pgo_node')(self.event(0), self.context)
        self.assertTrue(any(isinstance(a, Failure) for a in following))

    def test_lio_uses_explicit_topic_remapping_not_unused_parameter(self):
        actions = self.compose()
        lio = next(a for a in actions if isinstance(a, Node) and a.kwargs.get('executable') == 'lio_node')
        self.assertIn(('/agt/sensors/imu/data', '/livox/imu'), lio.kwargs['remappings'])
        self.assertNotIn('imu_topic', lio.kwargs['parameters'][0])

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
