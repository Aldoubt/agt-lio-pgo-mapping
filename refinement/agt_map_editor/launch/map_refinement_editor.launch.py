from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('refinement_file', default_value='refinement.yaml'),
        DeclareLaunchArgument('map_pcd', default_value=''),
        DeclareLaunchArgument('map_package', default_value=''),
        DeclareLaunchArgument('output_package', default_value=''),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='agt_map_editor',
            executable='map_refinement_editor',
            name='map_refinement_editor',
            output='screen',
            parameters=[{
                'refinement_file': LaunchConfiguration('refinement_file'),
                'map_pcd': LaunchConfiguration('map_pcd'),
                'map_package': LaunchConfiguration('map_package'),
                'output_package': LaunchConfiguration('output_package'),
                'resolution': LaunchConfiguration('resolution'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='map_refinement_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('agt_map_editor'), 'rviz', 'map_refinement.rviz'])],
        ),
    ])