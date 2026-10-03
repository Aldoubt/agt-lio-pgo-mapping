"""Select one audited LIO frontend, replay a bag, and export one common map package."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes'}


def _compose(context):
    from agt_mapping_bringup.backend_registry import (
        PROFILE_DIR,
        BackendSelectionError,
        resolve_backend,
    )

    backend_id = LaunchConfiguration('mapping_backend').perform(context).strip()
    allow_experimental = _bool(LaunchConfiguration('allow_experimental_backend').perform(context))
    try:
        profile = resolve_backend(backend_id or None, allow_experimental_backend=allow_experimental)
    except BackendSelectionError as exc:
        raise RuntimeError(str(exc)) from exc
    backend = profile['backend']
    sensor = profile['sensor']
    mapping = profile['mapping']
    bag_path = Path(LaunchConfiguration('bag_path').perform(context)).expanduser().resolve()
    output_dir = Path(LaunchConfiguration('output_dir').perform(context)).expanduser().resolve()
    playback_rate = float(LaunchConfiguration('playback_rate').perform(context))
    if not bag_path.is_dir() or not (bag_path / 'metadata.yaml').is_file():
        raise RuntimeError(f'bag_path must be a rosbag2 directory with metadata.yaml: {bag_path}')
    if not 0.1 <= playback_rate <= 2.0:
        raise RuntimeError('playback_rate must be in [0.1, 2.0]')
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f'refusing to reuse nonempty mapping output directory: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)

    actions = []
    if backend['id'] in {'lio_sam_noloop', 'point_lio', 'fast_lio2_legacy'}:
        actions.append(Node(
            package='agt_mid360_adapter', executable='mid360_adapter_node',
            name='mapping_mid360_adapter', output='screen',
            parameters=[{
                'input_topic': sensor['lidar_topic'],
                'output_topic': '/mapping/sensor/cloud',
                'timed_output_topic': sensor.get('normalized_lidar_topic', '/mapping/sensor/deskew_cloud'),
                'sanitized_topic': '/mapping/sensor/livox',
                'publish_timed_cloud': True,
                'frame_id': sensor['lidar_frame'],
            }],
        ))

    if backend['id'] == 'lio_sam_noloop':
        actions.append(Node(
            package='agt_mapping_bringup', executable='mapping_imu_accel_scaler',
            name='mapping_imu_accel_scaler', output='screen',
            parameters=[{
                'input_topic': sensor['imu_topic'],
                'output_topic': sensor['backend_imu_topic'],
                'acceleration_scale': sensor['imu_acceleration_scale'],
                'output_frame_id': sensor['imu_frame'],
            }],
        ))
        config = backend['config_path']
        overrides = {
            'use_sim_time': True,
            'pointCloudTopic': sensor['normalized_lidar_topic'],
            'imuTopic': sensor['backend_imu_topic'],
            'loopClosureEnableFlag': False,
            'gpsTopic': '/disabled/gps',
            'useGpsElevation': False,
        }
        actions.extend([
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='lio_sam_sensor_extrinsic', output='screen',
                 arguments=['--x', str(sensor['T_body_lidar_translation_m'][0]),
                            '--y', str(sensor['T_body_lidar_translation_m'][1]),
                            '--z', str(sensor['T_body_lidar_translation_m'][2]),
                            '--qx', str(sensor['T_body_lidar_quaternion_xyzw'][0]),
                            '--qy', str(sensor['T_body_lidar_quaternion_xyzw'][1]),
                            '--qz', str(sensor['T_body_lidar_quaternion_xyzw'][2]),
                            '--qw', str(sensor['T_body_lidar_quaternion_xyzw'][3]),
                            '--frame-id', sensor['body_frame'], '--child-frame-id', sensor['lidar_frame']]),
            *[
                Node(package='lio_sam', executable=name, name=name, output='screen',
                     parameters=[config, overrides])
                for name in ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
                             'lio_sam_featureExtraction', 'lio_sam_mapOptimization')
            ],
        ])
    elif backend['id'] == 'point_lio':
        actions.append(Node(
            package='point_lio', executable='pointlio_mapping', name='pointlio_mapping', output='screen',
            parameters=[backend['config_path'], {'use_sim_time': True}],
        ))
    elif backend['id'] == 'fast_livo2_lio':
        actions.append(Node(
            package='fast_livo', executable='fastlivo_mapping', name='fastlivo_mapping', output='screen',
            parameters=[backend['config_path'], {'use_sim_time': True}],
        ))
    elif backend['id'] == 'fast_lio2_legacy':
        actions.append(Node(
            package='fastlio2', executable='lio_node', name='lio_node', output='screen',
            parameters=[backend['config_path'], {'use_sim_time': True}],
        ))
    else:
        raise RuntimeError(f'no launch chain is implemented for explicit backend {backend["id"]}')

    profile_path = PROFILE_DIR / f'{backend_id}.yaml'
    actions.append(Node(
        package='agt_mapping_frontend_adapter', executable='mapping_frontend_adapter_node',
        name='mapping_frontend_adapter', output='screen',
        parameters=[{
            'use_sim_time': True,
            'backend_id': backend['id'],
            'source_commit': backend['source_commit'],
            'input_odom_topic': backend['output_odom_topic'],
            'input_cloud_topic': backend['output_cloud_topic'],
            'input_path_topic': backend['output_path_topic'] if backend.get('path_output_enabled', True) else '',
            'input_lidar_topic': sensor['lidar_topic'],
            'input_imu_topic': sensor['imu_topic'],
            'cloud_frame_mode': backend['output_cloud_frame_mode'],
            'body_frame': sensor['body_frame'],
            'lidar_frame': sensor['lidar_frame'],
            'max_sync_slop_s': backend.get('max_cloud_odom_slop_s', 0.05),
            'T_body_lidar.x': sensor['T_body_lidar_translation_m'][0],
            'T_body_lidar.y': sensor['T_body_lidar_translation_m'][1],
            'T_body_lidar.z': sensor['T_body_lidar_translation_m'][2],
            'T_body_lidar.qx': sensor['T_body_lidar_quaternion_xyzw'][0],
            'T_body_lidar.qy': sensor['T_body_lidar_quaternion_xyzw'][1],
            'T_body_lidar.qz': sensor['T_body_lidar_quaternion_xyzw'][2],
            'T_body_lidar.qw': sensor['T_body_lidar_quaternion_xyzw'][3],
        }],
    ))
    actions.append(Node(
        package='agt_mapping_bringup', executable='frontend_map_exporter',
        name='frontend_map_exporter', output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': str(output_dir),
            'bag_path': str(bag_path),
            'profile_path': str(profile_path),
            'keyframe_distance_m': float(mapping['keyframe_distance_m']),
            'keyframe_rotation_deg': float(mapping['keyframe_rotation_deg']),
            'body_frame': sensor['body_frame'],
        }],
    ))

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', str(bag_path), '--rate', str(playback_rate), '--clock',
             '--disable-keyboard-controls', '--topics', sensor['lidar_topic'], sensor['imu_topic']],
        output='screen',
    )
    export_process = ExecuteProcess(
        cmd=['ros2', 'run', 'agt_mapping_bringup', 'frontend_map_export_client'],
        output='screen',
    )

    def bag_finished(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'rosbag playback failed with exit code {event.returncode}; no artifact export requested'),
                EmitEvent(event=Shutdown(reason='bag playback failed')),
            ]
        return [TimerAction(period=3.0, actions=[export_process])]

    return [
        *actions,
        RegisterEventHandler(OnProcessExit(target_action=bag_player, on_exit=bag_finished)),
        RegisterEventHandler(OnProcessExit(
            target_action=export_process,
            on_exit=[EmitEvent(event=Shutdown(reason='frontend map package export finished'))],
        )),
        TimerAction(period=3.0, actions=[bag_player]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Read-only rosbag2 directory'),
        DeclareLaunchArgument('output_dir', description='New or empty run output directory'),
        DeclareLaunchArgument('mapping_backend', default_value='',
                              description='Explicit backend ID; default is gated until acceptance completes'),
        DeclareLaunchArgument('allow_experimental_backend', default_value='false'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        OpaqueFunction(function=_compose),
    ])
