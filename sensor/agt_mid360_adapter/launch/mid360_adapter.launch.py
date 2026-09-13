from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('output_topic', default_value='/mapping/sensor/cloud'),
        DeclareLaunchArgument(
            'frame_id', default_value='',
            description='Output frame override; empty preserves the input CustomMsg frame_id.'),
        Node(
            package='agt_mid360_adapter',
            executable='mid360_adapter_node',
            name='mid360_adapter_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
