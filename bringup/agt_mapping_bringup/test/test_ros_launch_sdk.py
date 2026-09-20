"""Real launch SDK checks (no ROS processes). Skipped when ROS is unavailable.

These supplement, not replace, the stand-in lifecycle tests and real bag runs.
"""
import importlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml


def ros_sdk_available():
    try:
        return all(importlib.util.find_spec(name) is not None
                   for name in ('launch', 'launch_ros', 'ament_index_python'))
    except (ImportError, ValueError):
        return False


@unittest.skipUnless(ros_sdk_available(), 'Requires a sourced ROS 2 launch SDK')
class RealLaunchSdkTests(unittest.TestCase):
    def test_python_launch_description_imports_real_sdk(self):
        from launch import LaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch import LaunchContext
        path = Path(__file__).resolve().parents[1] / 'launch/mapping_v0.launch.py'
        source = PythonLaunchDescriptionSource(str(path))
        description = source.get_launch_description(LaunchContext())
        self.assertIsInstance(description, LaunchDescription)

    def test_failure_action_raises_using_supported_opaque_function(self):
        from launch import LaunchContext
        from launch.actions import OpaqueFunction
        implementation = importlib.import_module('agt_mapping_bringup.session_launch')
        action = OpaqueFunction(function=implementation._raise_launch_error, args=['intentional failure'])
        with self.assertRaisesRegex(RuntimeError, 'intentional failure'):
            action.execute(LaunchContext())

    def test_real_node_and_event_actions_compose_without_starting_processes(self):
        from launch import LaunchContext
        from launch.actions import RegisterEventHandler
        from launch_ros.actions import Node
        from agt_mapping_bringup.session_lock import acquire_domain_lease
        implementation = importlib.import_module('agt_mapping_bringup.session_launch')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag = root / 'bag'
            bag.mkdir()
            (bag / 'data.db3').write_bytes(b'composition-only fixture, not played')
            (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
                'storage_identifier': 'sqlite3', 'message_count': 2,
                'relative_file_paths': ['data.db3'],
                'topics_with_message_count': [
                    {'topic_metadata': {'name': '/livox/lidar', 'type': 'livox_ros_driver2/msg/CustomMsg'},
                     'message_count': 1},
                    {'topic_metadata': {'name': '/livox/imu', 'type': 'sensor_msgs/msg/Imu'},
                     'message_count': 1},
                ]}}))
            context = LaunchContext()
            context.launch_configurations.update({
                'bag_path': str(bag), 'output_dir': str(root / 'output'),
                'lidar_topic': 'auto', 'imu_topic': 'auto', 'playback_rate': '1.0',
                'startup_timeout': '45', 'export_timeout': '180', 'drain_seconds': '3',
                'start_rviz': 'false', 'start_paused': 'false',
                'auto_export': 'true', 'keep_open': 'false',
            })
            share = Path(__file__).resolve().parents[1]
            with acquire_domain_lease(99, directory=root) as lease:
                with patch.object(implementation, 'get_package_share_directory', return_value=str(share)), \
                        patch.object(implementation, 'acquire_domain_lease', return_value=lease):
                    actions = implementation.launch_session(context)
            self.assertEqual(sum(isinstance(action, Node) for action in actions), 7)
            self.assertEqual(sum(isinstance(action, RegisterEventHandler) for action in actions), 10)


if __name__ == '__main__':
    unittest.main()
