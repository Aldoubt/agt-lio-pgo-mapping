from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_odom_topic', default_value='/fastlio2/lio_odom'),
        DeclareLaunchArgument('input_cloud_topic', default_value='/fastlio2/body_cloud'),
        DeclareLaunchArgument('input_path_topic', default_value='/fastlio2/lio_path'),
        DeclareLaunchArgument('output_namespace', default_value='/mapping/frontend'),
        Node(
            package='agt_fastlio_backend', executable='fastlio_backend_node',
            name='fastlio_backend_node', output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_path_topic': LaunchConfiguration('input_path_topic'),
                'output_namespace': LaunchConfiguration('output_namespace'),
            }],
        ),
    ])
