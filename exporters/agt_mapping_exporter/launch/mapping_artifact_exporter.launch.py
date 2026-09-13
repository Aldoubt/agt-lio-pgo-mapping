from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        Node(package='agt_mapping_exporter', executable='mapping_artifact_exporter', output='screen',
             parameters=[{'output_dir': LaunchConfiguration('output_dir')}]),
    ])
