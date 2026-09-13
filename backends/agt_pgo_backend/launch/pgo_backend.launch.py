from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_cloud_topic', default_value='/mapping/frontend/cloud'),
        DeclareLaunchArgument('input_odometry_topic', default_value='/mapping/frontend/odometry'),
        DeclareLaunchArgument('keyframe_distance_m', default_value='0.5'),
        DeclareLaunchArgument('start_external_pgo', default_value='false'),
        DeclareLaunchArgument('pgo_config', default_value=''),
        Node(
            package='agt_pgo_backend', executable='pgo_backend_node', name='pgo_backend_node', output='screen',
            parameters=[{
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_odometry_topic': LaunchConfiguration('input_odometry_topic'),
                'keyframe_distance_m': LaunchConfiguration('keyframe_distance_m'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('start_external_pgo')),
            package='pgo', executable='pgo_node', name='pgo_node', output='screen',
            parameters=[{'config_path': LaunchConfiguration('pgo_config')}],
        ),
    ])
