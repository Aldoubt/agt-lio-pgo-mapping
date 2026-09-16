from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('refinement_file', default_value='refinement.yaml'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='agt_map_editor',
            executable='map_refinement_editor',
            name='map_refinement_editor',
            output='screen',
            parameters=[{
                'refinement_file': LaunchConfiguration('refinement_file'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])