"""Live single-MID360 mapping: driver -> FAST-LIO2 -> PGO -> verified map package (+ raw bag)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from agt_mapping_bringup.live_launch import launch_live_session


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('livox_config', description='livox_ros_driver2 MID360 user config JSON'),
        DeclareLaunchArgument('output_dir', description='New or empty run output directory'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('duration_seconds', default_value='0.0',
                              description='0 = until /mapping/session/finish or STOP_MAPPING file'),
        DeclareLaunchArgument('sensor_stall_seconds', default_value='5.0'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('keep_open', default_value='false'),
        DeclareLaunchArgument('startup_timeout', default_value='45.0'),
        DeclareLaunchArgument('export_timeout', default_value='180.0'),
        DeclareLaunchArgument('drain_seconds', default_value='3.0'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_live_session)])
