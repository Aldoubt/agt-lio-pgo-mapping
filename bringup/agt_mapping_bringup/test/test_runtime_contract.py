"""Fake graph/service tests; deliberately not presented as ROS integration."""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
from agt_mapping_artifacts.frontend_package import write_frontend_map_package
from agt_mapping_bringup.preflight import BagInfo
from agt_mapping_bringup import session_runtime as runtime
from agt_mapping_bringup.session_state import create_session


class FakeClient:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.called = 0

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.called += 1
        return types.SimpleNamespace(done=lambda: True,
            result=lambda: types.SimpleNamespace(success=self.accepted, message='fixture response'))


class FakeNode:
    def __init__(self):
        self.params = {'export_timeout': 1.0, 'drain_seconds': 0.0,
                       'startup_timeout': 0.5, 'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu'}
        self.client = FakeClient()
        self.subscriptions = {}
        self.messages = []
        self.services = [('/mapping/frontend/export_artifact', ['std_srvs/srv/Trigger'])]
        self.graph = {
            '/livox/lidar': ['fastlivo_mapping'], '/livox/imu': ['fastlivo_mapping'],
            '/aft_mapped_to_init': ['mapping_frontend_adapter'],
            '/cloud_registered_lidar': ['mapping_frontend_adapter'],
            '/mapping/frontend/cloud': ['frontend_map_exporter'],
            '/mapping/frontend/odometry': ['frontend_map_exporter'],
        }

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=self.params.get(name, default))

    def create_client(self, *args):
        return self.client

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def get_logger(self):
        return types.SimpleNamespace(info=self.messages.append, error=self.messages.append)

    def get_service_names_and_types(self):
        return self.services

    def get_subscriptions_info_by_topic(self, topic):
        return [types.SimpleNamespace(node_name=name) for name in self.graph.get(topic, [])]


class RuntimeContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output = self.root / 'run'
        bag = self.root / 'bag'
        bag.mkdir()
        create_session(self.output, BagInfo(bag, '/livox/lidar', '/livox/imu', 1.0, 2, 'sqlite3'), {})
        self.node = FakeNode()
        self.now = [0.0]
        self.on_spin = lambda: None
        self.rclpy = types.SimpleNamespace(ok=lambda: True, spin_once=self.spin)
        message_modules = {
            'nav_msgs': types.ModuleType('nav_msgs'),
            'nav_msgs.msg': types.SimpleNamespace(Odometry=object),
            'diagnostic_msgs': types.ModuleType('diagnostic_msgs'),
            'diagnostic_msgs.msg': types.SimpleNamespace(DiagnosticArray=object),
            'std_msgs': types.ModuleType('std_msgs'),
            'std_msgs.msg': types.SimpleNamespace(String=object),
            'std_srvs': types.ModuleType('std_srvs'),
            'std_srvs.srv': types.SimpleNamespace(Trigger=types.SimpleNamespace(Request=lambda: object())),
        }
        self.modules = patch.dict(sys.modules, message_modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        clock = patch.object(runtime.time, 'monotonic', lambda: self.now[0])
        clock.start()
        self.addCleanup(clock.stop)
        original_wait = runtime.wait_until
        waiter = patch.object(runtime, 'wait_until', lambda check, spin, timeout, description:
            original_wait(check, spin, timeout, description, now=lambda: self.now[0]))
        waiter.start()
        self.addCleanup(waiter.stop)

    def spin(self, node, timeout_sec):
        self.now[0] += timeout_sec
        self.on_spin()

    def record(self):
        return json.loads((self.output / 'session.json').read_text())

    def write_artifact(self):
        package = self.output / 'map_package'
        if package.exists():
            return
        records = [{
            'stamp_sec': index + 1, 'stamp_nanosec': 0,
            'position': np.asarray([index * 0.6, 0.0, 0.0]),
            'quaternion_xyzw': np.asarray([0.0, 0.0, 0.0, 1.0]),
            'points_xyzi': np.asarray([[0.0, 0.0, 0.0, float(index)]], dtype='f4'),
        } for index in range(7)]
        backend = {
            'id': 'fast_livo2_lio', 'project': 'FAST-LIVO2', 'mode': 'lio_only',
            'source_commit': 'fixture', 'config_sha256': 'a' * 64,
            'loop_closure': False, 'gps_factor': False, 'external_global_correction': False,
        }
        write_frontend_map_package(package, records, {
            'mapping_backend': backend,
            'source': {'rosbag': 'fixture-bag', 'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu'},
            'frames': {'map': 'camera_init', 'body': 'livox_imu'},
            'reference': {'same_session': True, 'absolute_ground_truth': False,
                          'source': 'mapping_frontend_odometry', 'pgo_applied': False, 'optimized': False},
        })

    def test_acknowledgement_without_artifact_is_not_success(self):
        with self.assertRaises(TimeoutError):
            runtime._export(self.rclpy, self.node, self.output)
        self.assertEqual(self.node.client.called, 1)
        self.assertNotEqual(self.record()['status'], 'completed')
        self.assertLessEqual(self.now[0], 1.01)

    def test_rejected_request_is_not_success(self):
        self.node.client.accepted = False
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            runtime._export(self.rclpy, self.node, self.output)
        self.assertNotEqual(self.record()['status'], 'completed')

    def test_async_backend_failure_is_reported(self):
        failed = types.SimpleNamespace(name='mapping/frontend/exporter', message='no paired frames',
                                       values=[types.SimpleNamespace(key='artifact_state', value='failed')])
        self.on_spin = lambda: self.node.subscriptions['/mapping/frontend/status'](
            types.SimpleNamespace(status=[failed]))
        with self.assertRaisesRegex(RuntimeError, 'no paired frames'):
            runtime._export(self.rclpy, self.node, self.output)
        self.assertNotEqual(self.record()['status'], 'completed')

    def test_complete_verified_artifact_marks_success(self):
        self.on_spin = self.write_artifact
        runtime._export(self.rclpy, self.node, self.output)
        self.assertEqual(self.record()['status'], 'completed')
        self.assertTrue(self.record()['artifact_verified'])
        self.assertEqual(self.node.client.called, 1)

    def test_corrupt_artifact_never_marks_success(self):
        self.write_artifact()
        (self.output / 'map_package/map.pcd').write_text('corrupt')
        with self.assertRaises(TimeoutError):
            runtime._export(self.rclpy, self.node, self.output)
        self.assertNotEqual(self.record()['status'], 'completed')

    def test_complete_graph_opens_readiness_gate(self):
        runtime._wait_ready(self.rclpy, self.node, self.output)
        self.assertEqual(self.record()['status'], 'ready')
        self.assertEqual(self.node.client.called, 0)

    def test_missing_imu_consumer_times_out_with_topic_diagnostic(self):
        self.node.graph['/livox/imu'] = []
        with self.assertRaisesRegex(TimeoutError, '/livox/imu'):
            runtime._wait_ready(self.rclpy, self.node, self.output)
        self.assertNotEqual(self.record()['status'], 'ready')

    def test_legacy_pgo_mode_is_explicitly_rejected(self):
        self.node.params['reference_mode'] = 'pgo'
        with self.assertRaisesRegex(ValueError, 'PGO and FAST-LIO2 paths are disabled'):
            runtime._wait_ready(self.rclpy, self.node, self.output)

    def test_continuous_frontend_backlog_has_bounded_drain(self):
        self.node.params['drain_seconds'] = 0.4
        self.on_spin = lambda: self.node.subscriptions['/mapping/frontend/odometry'](object())
        with self.assertRaises(TimeoutError):
            runtime._export(self.rclpy, self.node, self.output)
        self.assertEqual(self.node.client.called, 0)
        self.assertLessEqual(self.now[0], 1.01)


if __name__ == '__main__':
    unittest.main()
