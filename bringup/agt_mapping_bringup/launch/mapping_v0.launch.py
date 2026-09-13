from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_mapping_bringup'))
    bag_path, output_dir = LaunchConfiguration('bag_path'), LaunchConfiguration('output_dir')
    lidar, imu = LaunchConfiguration('lidar_topic'), LaunchConfiguration('imu_topic')
    bag_playback = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--topics', lidar, imu],
    )
    auto_export = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/mapping/backend/export_artifact',
                 'std_srvs/srv/Trigger', '{}'],
        )],
        condition=IfCondition(LaunchConfiguration('auto_export')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('bag_path'), DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('auto_export', default_value='true'),
        Node(package='agt_mid360_adapter', executable='mid360_adapter_node', parameters=[{'input_topic': lidar}]),
        Node(package='fastlio2', namespace='fastlio2', executable='lio_node', parameters=[{'config_path': str(share / 'config' / 'fastlio2_mid360.yaml'), 'use_sim_time': True, 'lidar_topic': '/mapping/sensor/livox', 'imu_topic': imu}]),
        Node(package='agt_fastlio_backend', executable='fastlio_backend_node', parameters=[{'use_sim_time': True}]),
        Node(package='pgo', executable='pgo_node', parameters=[{'config_path': str(share / 'config' / 'pgo_frontend.yaml'), 'use_sim_time': True}]),
        Node(package='agt_pgo_backend', executable='pgo_backend_node', parameters=[{'use_sim_time': True, 'pgo_output_dir': PathJoinSubstitution([output_dir, 'pgo_raw'])}]),
        Node(package='agt_mapping_exporter', executable='mapping_artifact_exporter', parameters=[{'use_sim_time': True, 'output_dir': output_dir}]),
        Node(
            package='rviz2', executable='rviz2', name='agt_mapping_rviz', output='screen',
            arguments=['-d', str(share / 'rviz' / 'mapping_v0.rviz')],
            parameters=[{'use_sim_time': True}],
            # RViz is a host ROS package.  When launch is started from the
            # VS Code Snap terminal, these variables can make the dynamic
            # linker load Snap's glibc 2.31 pthread into the host process.
            additional_env={
                'SNAP': '', 'SNAP_LIBRARY_PATH': '', 'GTK_PATH': '',
                'GTK_EXE_PREFIX': '', 'GIO_MODULE_DIR': '',
                'GTK_IM_MODULE_FILE': '',
            },
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
        TimerAction(period=5.0, actions=[bag_playback]),
        RegisterEventHandler(OnProcessExit(target_action=bag_playback, on_exit=[auto_export])),
    ])
