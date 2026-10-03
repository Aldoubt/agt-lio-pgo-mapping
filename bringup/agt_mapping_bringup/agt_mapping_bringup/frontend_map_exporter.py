"""Collect normalized body-cloud/odometry pairs and write one backend-neutral map package."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from agt_mapping_artifacts.frontend_package import write_frontend_map_package


def main(args=None) -> int:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_srvs.srv import Trigger

    class FrontendMapExporter(Node):
        def __init__(self):
            super().__init__('frontend_map_exporter')
            self.output_dir = Path(str(self.declare_parameter('output_dir', '').value)).expanduser().resolve()
            self.bag_path = str(self.declare_parameter('bag_path', '').value)
            self.profile_path = Path(str(self.declare_parameter('profile_path', '')
                                          .value)).expanduser().resolve()
            self.keyframe_distance = float(self.declare_parameter('keyframe_distance_m', 0.5).value)
            self.keyframe_rotation_deg = float(self.declare_parameter('keyframe_rotation_deg', 10.0).value)
            self.body_frame = str(self.declare_parameter('body_frame', 'body').value)
            if not self.bag_path or not self.profile_path.is_file():
                raise ValueError('bag_path and profile_path are required')
            profile = yaml.safe_load(self.profile_path.read_text(encoding='utf-8'))
            self.mapping_backend = profile['backend']
            self.sensor = profile['sensor']
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.package_root = self.output_dir / 'map_package'
            self.records = []
            self.last_position = None
            self.last_quaternion = None
            self.last_stamp = -math.inf
            self.map_frame = ''
            self.capture_error = ''
            self.exported = False
            self.diagnostics = self.create_publisher(DiagnosticArray, '/mapping/frontend/status', 10)
            self.export_srv = self.create_service(Trigger, '/mapping/frontend/export_artifact', self.export)
            cloud_sub = Subscriber(self, PointCloud2, '/mapping/frontend/cloud', qos_profile=qos_profile_sensor_data)
            odom_sub = Subscriber(self, Odometry, '/mapping/frontend/odometry', qos_profile=10)
            self.sync = ApproximateTimeSynchronizer([cloud_sub, odom_sub], queue_size=50, slop=0.05)
            self.sync.registerCallback(self.on_pair)
            self.publish_state('ready', 'waiting for normalized cloud and odometry pairs')
            self.get_logger().info('backend-neutral map package exporter ready')

        def publish_state(self, state: str, detail: str):
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = 'mapping/frontend/exporter'
            status.hardware_id = str(self.mapping_backend.get('id', 'unknown'))
            status.level = DiagnosticStatus.ERROR if state == 'failed' else DiagnosticStatus.OK
            status.message = detail
            for key, value in (('backend_id', str(self.mapping_backend.get('id', 'unknown'))),
                               ('artifact_state', state), ('keyframe_count', str(len(self.records)))):
                item = KeyValue()
                item.key, item.value = key, value
                status.values.append(item)
            array.status.append(status)
            self.diagnostics.publish(array)

        def on_pair(self, cloud, odom):
            if self.capture_error or self.exported:
                return
            try:
                stamp = int(cloud.header.stamp.sec) + int(cloud.header.stamp.nanosec) * 1e-9
                if stamp <= self.last_stamp:
                    return
                if odom.child_frame_id != self.body_frame:
                    raise ValueError(f'normalized odometry child frame {odom.child_frame_id!r} != {self.body_frame!r}')
                if not self.map_frame:
                    self.map_frame = str(odom.header.frame_id)
                elif self.map_frame != odom.header.frame_id:
                    raise ValueError('frontend map frame changed during replay')
                pose = odom.pose.pose
                position = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype='f8')
                quaternion = np.asarray([pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w], dtype='f8')
                if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
                    raise ValueError('frontend pose contains nonfinite values')
                norm = float(np.linalg.norm(quaternion))
                if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
                    raise ValueError(f'frontend quaternion norm is invalid: {norm}')
                quaternion /= norm
                take = self.last_position is None
                if not take:
                    distance = float(np.linalg.norm(position - self.last_position))
                    dot = min(1.0, abs(float(np.dot(quaternion, self.last_quaternion))))
                    angle = math.degrees(2.0 * math.acos(dot))
                    take = distance >= self.keyframe_distance or angle >= self.keyframe_rotation_deg
                self.last_stamp = stamp
                if not take:
                    return
                names = {field.name for field in cloud.fields}
                if not {'x', 'y', 'z'}.issubset(names):
                    raise ValueError('normalized body cloud lacks x/y/z')
                selected_fields = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in names else [])
                raw = point_cloud2.read_points(cloud, field_names=selected_fields, skip_nans=True)
                if len(raw) == 0:
                    raise ValueError('normalized keyframe cloud is empty')
                xyz = np.column_stack([raw[name] for name in ('x', 'y', 'z')]).astype('<f4')
                intensity = np.asarray(raw['intensity'], dtype='<f4') if 'intensity' in selected_fields else np.zeros(len(xyz), '<f4')
                finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
                points = np.column_stack((xyz[finite], intensity[finite])).astype('<f4')
                if not len(points):
                    raise ValueError('normalized keyframe has no finite points')
                self.records.append({
                    'stamp_sec': int(cloud.header.stamp.sec),
                    'stamp_nanosec': int(cloud.header.stamp.nanosec),
                    'position': position,
                    'quaternion_xyzw': quaternion,
                    'points_xyzi': points,
                })
                self.last_position = position.copy()
                self.last_quaternion = quaternion.copy()
                if len(self.records) % 100 == 0:
                    self.get_logger().info(f'captured {len(self.records)} mapping keyframes')
            except Exception as exc:
                self.capture_error = str(exc)
                self.publish_state('failed', self.capture_error)
                self.get_logger().error(f'frontend map capture failed: {exc}')

        def export(self, _request, response):
            if self.capture_error:
                response.success = False
                response.message = self.capture_error
                return response
            if self.exported:
                response.success = False
                response.message = f'map package already exists: {self.package_root}'
                return response
            try:
                if not self.records:
                    raise ValueError('no synchronized frontend keyframes were received')
                backend = dict(self.mapping_backend)
                provenance = {
                    'mapping_backend': backend,
                    'source': {
                        'rosbag': self.bag_path,
                        'lidar_topic': self.sensor['lidar_topic'],
                        'imu_topic': self.sensor['imu_topic'],
                    },
                    'frames': {
                        'map': self.map_frame, 'body': self.body_frame,
                        'lidar': self.sensor['lidar_frame'], 'imu': self.sensor['imu_frame'],
                        'T_frontend_body': 'Odometry.pose.pose; native state rigidly normalized to body frame',
                        'T_body_lidar_translation_m': self.sensor['T_body_lidar_translation_m'],
                        'T_body_lidar_quaternion_xyzw': self.sensor['T_body_lidar_quaternion_xyzw'],
                    },
                    'calibration': {
                        'status': 'profile extrinsic',
                        'T_body_lidar_translation_m': self.sensor['T_body_lidar_translation_m'],
                        'T_body_lidar_quaternion_xyzw': self.sensor['T_body_lidar_quaternion_xyzw'],
                    },
                    'reference': {
                        'same_session': True,
                        'absolute_ground_truth': False,
                        'source': 'mapping_frontend_odometry',
                        'pgo_applied': False,
                        'optimized': False,
                    },
                }
                result = write_frontend_map_package(self.package_root, self.records, provenance)
                self.exported = True
                response.success = True
                response.message = (f"PASS: {result['backend_id']} map package with "
                                    f"{result['keyframes']} keyframes / {result['map_points']} points: "
                                    f'{self.package_root}')
                self.publish_state('complete', response.message)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.publish_state('failed', response.message)
            return response

    rclpy.init(args=args)
    node = None
    try:
        node = FrontendMapExporter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def export_client_main(args=None) -> int:
    """Call the export service and propagate its success to the launch process."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    rclpy.init(args=args)
    node = Node('frontend_map_export_client')
    client = node.create_client(Trigger, '/mapping/frontend/export_artifact')
    try:
        if not client.wait_for_service(timeout_sec=30.0):
            print('mapping frontend export service did not become available')
            return 2
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=180.0)
        if not future.done() or future.result() is None:
            print('mapping frontend export service timed out')
            return 3
        response = future.result()
        print(response.message)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
