"""Backward-compatible entry point for the verified offline mapping workflow."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from agt_mapping_bringup.session_launch import launch_session


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('bag_path', description='Read-only rosbag2 directory'),
        DeclareLaunchArgument('output_dir', description='New or empty run output directory'),
        DeclareLaunchArgument('lidar_topic', default_value='auto'),
        DeclareLaunchArgument('imu_topic', default_value='auto'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('auto_export', default_value='true'),
        DeclareLaunchArgument('keep_open', default_value='false'),
        DeclareLaunchArgument('startup_timeout', default_value='45.0'),
        DeclareLaunchArgument('export_timeout', default_value='180.0'),
        DeclareLaunchArgument('drain_seconds', default_value='3.0'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_session)])
