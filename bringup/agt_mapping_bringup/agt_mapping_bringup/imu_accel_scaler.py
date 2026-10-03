"""Copy an IMU message and convert only acceleration units, preserving its stamp."""
from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuAccelerationScaler(Node):
    def __init__(self):
        super().__init__('mapping_imu_accel_scaler')
        input_topic = str(self.declare_parameter('input_topic', '/agt/sensors/imu/data').value)
        output_topic = str(self.declare_parameter('output_topic', '/mapping/sensor/imu_si').value)
        self.scale = float(self.declare_parameter('acceleration_scale', 9.80665).value)
        output_frame = str(self.declare_parameter('output_frame_id', 'livox_imu').value)
        if self.scale <= 0.0:
            raise ValueError('acceleration_scale must be positive')
        self.output_frame = output_frame
        self.publisher = self.create_publisher(Imu, output_topic, 100)
        self.subscription = self.create_subscription(Imu, input_topic, self.callback, qos_profile_sensor_data)
        self.count = 0

    def callback(self, source: Imu):
        output = copy.deepcopy(source)
        output.header.frame_id = self.output_frame
        output.linear_acceleration.x *= self.scale
        output.linear_acceleration.y *= self.scale
        output.linear_acceleration.z *= self.scale
        if any(output.linear_acceleration_covariance):
            output.linear_acceleration_covariance = [
                value * self.scale * self.scale for value in output.linear_acceleration_covariance]
        self.publisher.publish(output)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuAccelerationScaler()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
