"""Launch composition for one live MID360 mapping session.

Differences from the offline replay (session_launch.py):
  * livox_ros_driver2 (xfer_format=1 CustomMsg, multi_topic=0) replaces the
    rosbag player; wall clock is used (no --clock / use_sim_time).
  * A raw rosbag2 recording of /livox/lidar + /livox/imu is ALWAYS written to
    <output>/raw_bag so the run can be replayed offline with the existing
    verified workflow (and re-mapped after algorithm changes).
  * A supervisor process decides when the capture ends (service / stop file /
    duration / IMU stall). Its exit stops the recorder, and the same verified
    finalizer (mapping_export_verified) produces map_package.
"""
import atexit
from pathlib import Path
import signal

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .live_source import inspect_live_config
from .preflight import frontend_remappings, parse_bool, validate_number
from .session_launch import _raise_launch_error
from .session_lock import acquire_domain_lease
from .session_state import create_session, mark_session


def raw_record_command(output, lidar_topic, imu_topic):
    return ['ros2', 'bag', 'record', '--output', str(Path(output) / 'raw_bag'),
            '--storage', 'sqlite3', lidar_topic, imu_topic]


def launch_live_session(context, *, sensor_only=False):
    """Legacy Bunker hardware owner by default; YHS path owns only one MID360.

    sensor_only never resolves Bunker/YHS chassis config, starts a base driver,
    publishes a robot TF, or selects a map. It still runs the same recorder,
    mapping estimator, PGO and verified artifact exporter.
    """
    def value(name):
        return LaunchConfiguration(name).perform(context)

    source = inspect_live_config(value('livox_config'), value('lidar_topic'), value('imu_topic'),
                                 validate_number(value('publish_freq'), 'publish_freq'))
    output = Path(value('output_dir')).expanduser().resolve()
    options = {
        'mode': 'live',
        'duration_seconds': validate_number(value('duration_seconds'), 'duration_seconds', allow_zero=True),
        'sensor_stall_seconds': validate_number(value('sensor_stall_seconds'), 'sensor_stall_seconds',
                                                allow_zero=True),
        'startup_timeout': validate_number(value('startup_timeout'), 'startup_timeout'),
        'export_timeout': validate_number(value('export_timeout'), 'export_timeout'),
        'drain_seconds': validate_number(value('drain_seconds'), 'drain_seconds', allow_zero=True),
        'publish_freq': source.publish_freq,
        'frame_id': value('frame_id'),
        'mapping_sensor_owner': 'yhs_sensor_only' if sensor_only else 'bunker_legacy_hardware',
        **{key: parse_bool(value(key)) for key in ('start_rviz', 'keep_open')},
    }
    if options['export_timeout'] <= options['drain_seconds']:
        raise ValueError('export_timeout must exceed drain_seconds')
    share = Path(get_package_share_directory('agt_mapping_bringup'))
    lio_config = share / 'config' / 'fastlio2_mid360.yaml'
    remappings = frontend_remappings(lio_config, source.imu_topic)
    text = lambda v: ParameterValue(str(v), value_type=str)
    if sensor_only:
        # The YHS mapping path is sensor-only: do not resolve Bunker, do not
        # launch a chassis driver, and do not publish either robot's TF.
        from .yhs_live_sensor import make_sensor_node
        driver = make_sensor_node(source, options['frame_id'])
    else:
        robot_share = Path(get_package_share_directory('agt_robot_bringup'))
        driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_share / 'launch' / 'robot_hardware.launch.py')),
            launch_arguments={
                'robot': 'bunker_v1',
                'enable_robot_description': 'false',
                'enable_mid360': 'true',
                'mid360_driver_mode': 'mapping_custom',
                'mapping_livox_config': str(source.path),
                'mapping_publish_freq': str(source.publish_freq),
                'mapping_frame_id': options['frame_id'],
                'enable_bunker_can': 'false',
                'enable_rtk': 'false',
                'enable_camera_gimbal': 'false',
            }.items())
    nodes = [
        Node(package='agt_mid360_adapter', executable='mid360_adapter_node',
             parameters=[{'input_topic': source.lidar_topic}]),
        Node(package='fastlio2', namespace='fastlio2', executable='lio_node',
             parameters=[{'config_path': text(lio_config), 'use_sim_time': False}],
             remappings=remappings),
        Node(package='agt_fastlio_backend', executable='fastlio_backend_node',
             parameters=[{'use_sim_time': False}]),
        Node(package='pgo', executable='pgo_node',
             parameters=[{'config_path': text(share / 'config' / 'pgo_frontend.yaml'),
                          'use_sim_time': False}]),
        Node(package='agt_pgo_backend', executable='pgo_backend_node',
             parameters=[{'use_sim_time': False, 'pgo_output_dir': text(output / 'pgo_raw')}]),
        Node(package='agt_mapping_exporter', executable='mapping_artifact_exporter',
             parameters=[{'use_sim_time': False, 'output_dir': text(output)}]),
    ]
    labels = ['sensor adapter', 'FAST-LIO2', 'frontend bridge', 'PGO', 'PGO bridge',
              'artifact exporter']
    recorder = ExecuteProcess(cmd=raw_record_command(output, source.lidar_topic, source.imu_topic),
                              output='screen')
    ready = Node(package='agt_mapping_bringup', executable='mapping_wait_ready', output='screen',
                 parameters=[{'output_dir': text(output), 'startup_timeout': options['startup_timeout'],
                              'lidar_topic': source.lidar_topic, 'imu_topic': source.imu_topic,
                              'require_publishers': True}])
    supervisor = Node(package='agt_mapping_bringup', executable='mapping_live_supervisor', output='screen',
                      parameters=[{'output_dir': text(output),
                                   'duration_seconds': options['duration_seconds'],
                                   'sensor_stall_seconds': options['sensor_stall_seconds'],
                                   'imu_topic': source.imu_topic}])
    finalizer = Node(package='agt_mapping_bringup', executable='mapping_export_verified', output='screen',
                     parameters=[{'output_dir': text(output), 'export_timeout': options['export_timeout'],
                                  'drain_seconds': options['drain_seconds']}])

    def fail(message):
        mark_session(output, 'failed', message)
        return [OpaqueFunction(function=_raise_launch_error, args=[message])]

    def readiness_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if event.returncode != 0:
            return fail('Live readiness failed: driver/sensor streams or consumers missing (see session.json)')
        return [LogInfo(msg='[2/4] Sensor streams live; recording raw bag and mapping'), recorder, supervisor]

    def supervisor_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if event.returncode != 0:
            return fail(f'Live supervisor ended with {event.returncode}; no export (sensor stall or cancel)')
        # Close the recorder cleanly (SIGINT flushes rosbag2 metadata), then export.
        return [EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                                              process_matcher=lambda action: action is recorder))]

    def recorder_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if not (output / 'raw_bag' / 'metadata.yaml').is_file():
            return fail('Raw bag recorder closed without metadata.yaml; refusing to export an unreplayable run')
        mark_session(output, 'raw_bag_closed', 'Raw Livox recording closed; requesting verified export',
                     raw_bag=str(output / 'raw_bag'))
        return [finalizer]

    def export_exit(event, ctx):
        if ctx.is_shutdown:
            return []
        if event.returncode != 0:
            return fail('Map export/verification failed; no verified completion (see session.json/logs)')
        if options['keep_open']:
            return [LogInfo(msg=f'Verified map: {output / "map_package"}. --keep-open: Ctrl+C to close.')]
        return [LogInfo(msg=f'Verified map: {output / "map_package"}; raw bag: {output / "raw_bag"}; closing.'),
                EmitEvent(event=Shutdown(reason='Live mapping artifact verified'))]

    def critical_exit(label):
        def handler(event, ctx):
            if ctx.is_shutdown:
                return []
            return fail(f'{label} exited unexpectedly ({event.returncode}); stopping this live session')
        return handler

    def shutdown(event, ctx):
        mark_session(output, 'cancelled', 'Launch stopped before automatic verified completion')
        return []

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=shutdown)),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=readiness_exit)),
        RegisterEventHandler(OnProcessExit(target_action=supervisor, on_exit=supervisor_exit)),
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=recorder_exit)),
        RegisterEventHandler(OnProcessExit(target_action=finalizer, on_exit=export_exit)),
    ]
    actions.extend(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=critical_exit(label)))
                   for node, label in zip(nodes, labels))
    actions.append(driver)
    actions.extend(nodes)
    if options['start_rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='agt_mapping_rviz', output='screen',
            arguments=['-d', str(share / 'rviz' / 'mapping_v0.rviz')],
            additional_env={'SNAP': '', 'SNAP_LIBRARY_PATH': '', 'GTK_PATH': '',
                            'GTK_EXE_PREFIX': '', 'GIO_MODULE_DIR': '', 'GTK_IM_MODULE_FILE': ''},
        ))
    actions.append(ready)
    lease = acquire_domain_lease()
    try:
        create_session(output, source, options)
    except Exception:
        lease.close()
        raise
    atexit.register(lease.close)
    return actions
