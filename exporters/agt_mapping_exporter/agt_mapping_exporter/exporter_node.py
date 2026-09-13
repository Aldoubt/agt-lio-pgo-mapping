from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from std_msgs.msg import String

from agt_mapping_artifacts import ArtifactWriter


def _pose_record(pose: PoseStamped) -> dict:
    return {
        'stamp': {'sec': pose.header.stamp.sec, 'nanosec': pose.header.stamp.nanosec},
        'position': {'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z},
        'orientation': {'w': pose.pose.orientation.w, 'x': pose.pose.orientation.x,
                        'y': pose.pose.orientation.y, 'z': pose.pose.orientation.z},
    }


class MappingArtifactExporter(Node):
    def __init__(self):
        super().__init__('mapping_artifact_exporter')
        self.declare_parameter('output_dir', 'output')
        self.declare_parameter('keyframes_topic', '/mapping/backend/keyframes')
        self.declare_parameter('map_pose_topic', '/mapping/backend/map_pose')
        self.declare_parameter('status_topic', '/mapping/backend/status')
        self.keyframes: list[dict] = []
        self.map_pose: dict | None = None
        self.backend_status = 'waiting_for_backend'
        self.create_subscription(NavPath, self.get_parameter('keyframes_topic').value, self._keyframes, 10)
        self.create_subscription(PoseStamped, self.get_parameter('map_pose_topic').value, self._map_pose, 20)
        self.create_subscription(String, self.get_parameter('status_topic').value, self._status, 10)

    def _keyframes(self, message: NavPath):
        self.keyframes = [_pose_record(pose) for pose in message.poses]

    def _map_pose(self, message: PoseStamped):
        self.map_pose = _pose_record(message)

    def _status(self, message: String):
        self.backend_status = message.data
        if message.data == 'artifact_export_requested':
            root = ArtifactWriter(Path(self.get_parameter('output_dir').value)).write(
                self.keyframes, self.map_pose, self.backend_status)
            self.get_logger().info(f'Wrote mapping artifact: {root}')


def main(args=None):
    rclpy.init(args=args)
    node = MappingArtifactExporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
