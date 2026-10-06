"""Launch composition and explicit lifecycle for one offline mapping session."""
import atexit
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .backend_registry import BackendSelectionError, PROFILE_DIR, resolve_backend
from .preflight import inspect_bag, parse_bool, validate_number
from .session_state import create_session, mark_session, playback_action
from .session_lock import acquire_domain_lease


def _raise_launch_error(_context, message):
    # Humble has no launch.actions.RaiseError. OpaqueFunction propagates this
    # exception through LaunchService, preserving a nonzero launch result.
    raise RuntimeError(message)


def launch_session(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    bag = inspect_bag(value('bag_path'), value('lidar_topic'), value('imu_topic'))
    output = Path(value('output_dir')).expanduser().resolve()
    options = {
        'reference_mode': value('reference_mode'),
        'mapping_backend': value('mapping_backend').strip(),
        'playback_rate': validate_number(value('playback_rate'), 'playback_rate'),
        'startup_timeout': validate_number(value('startup_timeout'), 'startup_timeout'),
        'export_timeout': validate_number(value('export_timeout'), 'export_timeout'),
        'drain_seconds': validate_number(value('drain_seconds'), 'drain_seconds', allow_zero=True),
        **{key: parse_bool(value(key)) for key in
           ('start_rviz', 'start_paused', 'auto_export', 'keep_open')},
    }
    if options['export_timeout'] <= options['drain_seconds']:
        raise ValueError('export_timeout must exceed drain_seconds')
    if options['reference_mode'] != 'frontend':
        raise ValueError('only the verified frontend source-package mode is supported; PGO/FAST-LIO2 are disabled')
    try:
        profile = resolve_backend(options['mapping_backend'] or None)
    except BackendSelectionError as exc:
        raise ValueError(str(exc)) from exc
    backend = profile['backend']
    options['mapping_backend'] = backend['id']
    sensor = profile['sensor']
    mapping = profile['mapping']
    if backend['id'] != 'fast_livo2_lio':
        raise ValueError(f"offline session launch does not yet support backend {backend['id']!r}")
    share = Path(get_package_share_directory('agt_mapping_bringup'))
    text = lambda value: ParameterValue(str(value), value_type=str)
    nodes = [
        Node(package='fast_livo', executable='fastlivo_mapping', name='fastlivo_mapping', output='log',
             parameters=[backend['config_path'], {
                 'use_sim_time': True,
                 'common.lid_topic': bag.lidar_topic,
                 'common.imu_topic': bag.imu_topic,
             }]),
        Node(package='agt_mapping_frontend_adapter', executable='mapping_frontend_adapter_node',
             name='mapping_frontend_adapter', output='screen', parameters=[{
                 'use_sim_time': True, 'backend_id': backend['id'],
                 'source_commit': backend['source_commit'],
                 'input_odom_topic': backend['output_odom_topic'],
                 'input_cloud_topic': backend['output_cloud_topic'],
                 'input_path_topic': backend['output_path_topic'],
                 'input_lidar_topic': bag.lidar_topic, 'input_imu_topic': bag.imu_topic,
                 'cloud_frame_mode': backend['output_cloud_frame_mode'],
                 'body_frame': sensor['body_frame'], 'lidar_frame': sensor['lidar_frame'],
                 'max_sync_slop_s': backend.get('max_cloud_odom_slop_s', 0.05),
                 'T_body_lidar.x': sensor['T_body_lidar_translation_m'][0],
                 'T_body_lidar.y': sensor['T_body_lidar_translation_m'][1],
                 'T_body_lidar.z': sensor['T_body_lidar_translation_m'][2],
                 'T_body_lidar.qx': sensor['T_body_lidar_quaternion_xyzw'][0],
                 'T_body_lidar.qy': sensor['T_body_lidar_quaternion_xyzw'][1],
                 'T_body_lidar.qz': sensor['T_body_lidar_quaternion_xyzw'][2],
                 'T_body_lidar.qw': sensor['T_body_lidar_quaternion_xyzw'][3],
             }]),
        Node(package='agt_mapping_bringup', executable='frontend_map_exporter',
             name='frontend_map_exporter', output='screen', parameters=[{
                 'use_sim_time': True, 'output_dir': text(output), 'bag_path': str(bag.path),
                 'profile_path': str(PROFILE_DIR / f"{backend['id']}.yaml"),
                 'input_lidar_topic': bag.lidar_topic, 'input_imu_topic': bag.imu_topic,
                 'keyframe_distance_m': float(mapping['keyframe_distance_m']),
                 'keyframe_rotation_deg': float(mapping['keyframe_rotation_deg']),
                 'body_frame': sensor['body_frame'],
             }]),
    ]
    labels = ['FAST-LIVO2 LIO-only', 'frontend normalization adapter', 'frontend map exporter']
    playback_cmd = ['ros2', 'bag', 'play', str(bag.path), '--clock', '--rate',
                    str(options['playback_rate']), '--topics', bag.lidar_topic, bag.imu_topic]
    if options['start_paused']:
        playback_cmd.append('--start-paused')
    player = ExecuteProcess(cmd=playback_cmd, output='screen')
    ready = Node(package='agt_mapping_bringup', executable='mapping_wait_ready', output='screen',
                 parameters=[{'output_dir': text(output), 'startup_timeout': options['startup_timeout'],
                              'lidar_topic': bag.lidar_topic, 'imu_topic': bag.imu_topic,
                              'reference_mode': options['reference_mode']}])
    finalizer = Node(package='agt_mapping_bringup', executable='mapping_export_verified', output='screen',
                     parameters=[{'output_dir': text(output), 'export_timeout': options['export_timeout'],
                                  'drain_seconds': options['drain_seconds'],
                                  'reference_mode': options['reference_mode']}])

    def fail(message):
        mark_session(output, 'failed', message)
        return [OpaqueFunction(function=_raise_launch_error, args=[message])]

    def readiness_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if event.returncode != 0:
            return fail('Mapping readiness failed; bag was not started (see session.json/logs)')
        mark_session(output, 'replaying', 'Playing only the selected recorded lidar and IMU topics')
        message = '[2/4] Replaying bag. Ctrl+C cancels WITHOUT exporting a partial map.'
        if options['start_paused']:
            message += (' Player starts paused. Resume in the SAME ROS domain: '
                        'ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"')
        return [LogInfo(msg=message), player]

    def playback_exit(event, ctx):
        action = playback_action(event.returncode, ctx.is_shutdown, options['auto_export'])
        if action == 'stop':
            return []
        if action == 'fail':
            return fail(f'Rosbag failed with exit code {event.returncode}; automatic export skipped')
        if action == 'manual':
            mark_session(output, 'waiting_manual_export', 'Playback finished; automatic export disabled')
            return [LogInfo(msg='Manual mode: request /mapping/frontend/export_artifact, then run '
                                'the frontend source-package validator on the output. Ctrl+C closes nodes; '
                                'this mode does not declare automatic completion.')]
        return [finalizer]

    def export_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if event.returncode != 0:
            return fail('Map export/verification failed; no verified completion (see session.json/logs)')
        if options['keep_open']:
            return [LogInfo(msg=f'Verified FAST-LIVO2 source map: {output / "map_package"}. --keep-open: Ctrl+C to close.')]
        return [LogInfo(msg=f'Verified FAST-LIVO2 source map: {output / "map_package"}; closing this launch.'),
                EmitEvent(event=Shutdown(reason='FAST-LIVO2 LIO-only source artifact verified'))]

    def critical_exit(label):
        def handler(event, ctx):
            if ctx.is_shutdown:
                return []
            return fail(f'{label} exited unexpectedly ({event.returncode}); stopping this session')
        return handler

    def shutdown(event, ctx):
        mark_session(output, 'cancelled', 'Launch stopped before automatic verified completion')
        return []

    # Register before starting processes, so even a fast startup failure is observed.
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=shutdown)),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=readiness_exit)),
        RegisterEventHandler(OnProcessExit(target_action=player, on_exit=playback_exit)),
        RegisterEventHandler(OnProcessExit(target_action=finalizer, on_exit=export_exit)),
    ]
    actions.extend(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=critical_exit(label)))
                   for node, label in zip(nodes, labels))
    actions.extend(nodes)
    if options['start_rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='agt_mapping_rviz', output='screen',
            arguments=['-d', str(share / 'rviz' / 'mapping_v0.rviz')],
            parameters=[{'use_sim_time': True}],
            additional_env={'SNAP': '', 'SNAP_LIBRARY_PATH': '', 'GTK_PATH': '',
                            'GTK_EXE_PREFIX': '', 'GIO_MODULE_DIR': '', 'GTK_IM_MODULE_FILE': ''},
        ))
    actions.append(ready)
    lease = acquire_domain_lease()
    try:
        create_session(output, bag, options)
    except Exception:
        lease.close()
        raise
    # Keep the lock until ros2 launch exits, including its child-process teardown.
    atexit.register(lease.close)
    return actions
