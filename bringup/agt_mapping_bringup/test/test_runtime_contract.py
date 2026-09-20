"""Fake graph/service tests; deliberately not presented as ROS integration."""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from agt_mapping_artifacts import ArtifactWriter
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
        self.services = [('/pgo/save_maps', ['interface/srv/SaveMaps'])]
        self.graph = {
            '/livox/lidar': ['mid360_adapter_node'], '/livox/imu': ['lio_node'],
            '/mapping/sensor/livox': ['lio_node'], '/mapping/frontend/cloud': ['pgo_node'],
            '/mapping/frontend/odometry': ['pgo_node', 'pgo_backend_node'],
            '/mapping/backend/status': ['mapping_artifact_exporter'],
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
        source = self.root / 'pgo'
        if source.exists():
            return
        (source / 'patches').mkdir(parents=True)
        pcd = 'VERSION 0.7\nFIELDS x y z\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n'
        (source / 'map.pcd').write_text(pcd)
        (source / 'patches/0.pcd').write_text(pcd)
        (source / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
        (source / 'poses_timed.txt').write_text('0.pcd 1.0 0 0 0 1 0 0 0\n')
        ArtifactWriter(self.output).write_optimized_pgo(source)

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
        self.on_spin = lambda: self.node.subscriptions['/mapping/backend/status'](
            types.SimpleNamespace(data='pgo_export_failed'))
        with self.assertRaisesRegex(RuntimeError, 'pgo_export_failed'):
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

    def test_wrong_pgo_service_type_never_opens_gate(self):
        self.node.services = [('/pgo/save_maps', ['std_srvs/srv/Trigger'])]
        with self.assertRaisesRegex(TimeoutError, '/pgo/save_maps'):
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
