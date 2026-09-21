"""Operator-friendly offline mapping entry point; --help does not require ROS."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import sys

from .live_source import (
    DEFAULT_IMU_TOPIC, DEFAULT_LIDAR_TOPIC, host_interface_hint, inspect_live_config,
)
from .preflight import PreflightError, check_output, inspect_bag, validate_number

DEFAULT_LIVOX_CONFIG = 'share/livox_ros_driver2/config/MID360_config.json'


def parser():
    result = argparse.ArgumentParser(
        prog='run_mid360_mapping.sh',
        description='MID360 rosbag (or --live sensor) -> FAST-LIO2 -> PGO -> verified map package',
        epilog='Input bag is read-only. Existing nonempty outputs are never overwritten. '
               'Ctrl+C cancels without automatic partial-map export. '
               '--live records a raw bag under <output>/raw_bag and finishes on '
               '`ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"`.')
    result.add_argument('bag', nargs='?', help='rosbag2 directory containing metadata.yaml (omit with --live)')
    result.add_argument('output', nargs='?', help='New/empty run directory (default: timestamped output)')
    result.add_argument('--rate', default=1.0, type=float, help='Playback speed, default 1.0')
    result.add_argument('--lidar-topic', default='auto', help='CustomMsg topic, default: auto-detect unique stream')
    result.add_argument('--imu-topic', default='auto', help='Imu topic, default: auto-detect unique stream')
    rviz = result.add_mutually_exclusive_group()
    rviz.add_argument('--rviz', dest='rviz', action='store_true', help='Open RViz')
    rviz.add_argument('--no-rviz', '--headless', dest='rviz', action='store_false', help='Run without RViz')
    result.set_defaults(rviz=None)
    result.add_argument('--start-paused', action='store_true', help='Start the rosbag player paused')
    result.add_argument('--manual-export', action='store_true', help='Stay open after playback; do not request export')
    result.add_argument('--keep-open', action='store_true', help='Keep nodes/RViz open after a verified export')
    result.add_argument('--startup-timeout', type=float, default=45.0, help='Readiness deadline in wall seconds')
    result.add_argument('--export-timeout', type=float, default=180.0, help='Total drain/export/verify wall seconds')
    result.add_argument('--drain-seconds', type=float, default=3.0, help='Frontend quiet window after playback')
    result.add_argument('--domain-id', type=int, default=89, help='Dedicated local ROS domain, default 89 (0..101)')
    result.add_argument('--ros-setup', default='/opt/ros/humble/setup.bash', help='Base ROS setup.bash')
    result.add_argument('--setup', help='Workspace overlay setup.bash; prefer install_mapping_framework if present')
    result.add_argument('--dry-run', action='store_true', help='Check input and print plan; no ROS nodes or output writes')
    live = result.add_argument_group('live MID360 mode')
    live.add_argument('--live', action='store_true',
                      help='Map from the connected MID360 instead of a bag; always records a raw bag')
    live.add_argument('--livox-config', help='livox_ros_driver2 MID360 JSON (default: installed MID360_config.json)')
    live.add_argument('--publish-freq', type=float, default=10.0, help='Livox publish frequency Hz (5/10/20/50)')
    live.add_argument('--frame-id', default='livox_frame', help='Livox driver frame_id')
    live.add_argument('--duration', type=float, default=0.0,
                      help='Stop capture automatically after N seconds (0 = manual finish)')
    live.add_argument('--sensor-stall-seconds', type=float, default=5.0,
                      help='Abort (no export) when no IMU samples arrive for this long; 0 disables')
    return result


def _default_livox_config(workspace, setup):
    candidates = [setup.parent / DEFAULT_LIVOX_CONFIG,
                  workspace / 'install' / DEFAULT_LIVOX_CONFIG,
                  workspace / 'src' / 'external' / 'livox_ros_driver2' / 'config' / 'MID360_config.json']
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise PreflightError('No livox_ros_driver2 MID360_config.json found; pass --livox-config')


def _live_plan(args, workspace, setup, ros_setup, start_rviz):
    for name in ('duration', 'sensor_stall_seconds'):
        setattr(args, name, validate_number(getattr(args, name), name, allow_zero=True))
    config = Path(args.livox_config).expanduser().resolve() if args.livox_config else _default_livox_config(workspace, setup)
    lidar_topic = DEFAULT_LIDAR_TOPIC if args.lidar_topic == 'auto' else args.lidar_topic
    imu_topic = DEFAULT_IMU_TOPIC if args.imu_topic == 'auto' else args.imu_topic
    source = inspect_live_config(config, lidar_topic, imu_topic, args.publish_freq)
    default = workspace / 'experiments' / 'artifacts' / 'output' / (
        'live_mid360_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output = check_output(args.output or default, source.path)
    parameters = {
        'livox_config': str(source.path), 'output_dir': str(output),
        'lidar_topic': source.lidar_topic, 'imu_topic': source.imu_topic,
        'publish_freq': source.publish_freq, 'frame_id': args.frame_id,
        'duration_seconds': args.duration, 'sensor_stall_seconds': args.sensor_stall_seconds,
        'start_rviz': start_rviz, 'keep_open': args.keep_open,
        'startup_timeout': args.startup_timeout, 'export_timeout': args.export_timeout,
        'drain_seconds': args.drain_seconds,
    }
    command = ['ros2', 'launch', 'agt_mapping_bringup', 'mapping_live_mid360.launch.py']
    command += [f'{key}:={str(value).lower() if isinstance(value, bool) else value}'
                for key, value in parameters.items()]
    plan = {
        'mode': 'dry-run' if args.dry_run else 'live', 'livox_config': str(source.path),
        'host_ip': source.host_ip, 'lidar_ip': source.lidar_ip,
        'lidar_topic': source.lidar_topic, 'imu_topic': source.imu_topic,
        'raw_bag': str(output / 'raw_bag'), 'output': str(output),
        'overlay': str(setup), 'ros_setup': str(ros_setup), 'command': command,
        'network_hint': host_interface_hint(source.host_ip), 'runtime_checked': False,
    }
    return source, output, command, plan


def _require_environment(ros_setup, setup):
    if not ros_setup.is_file():
        raise PreflightError(f'ROS setup not found: {ros_setup}. Use a ROS 2 Humble environment '
                             'or specify --ros-setup. --dry-run works without ROS.')
    if not setup.is_file():
        raise PreflightError(f'Workspace setup not found: {setup}. Build the mapping workspace '
                             'or choose --setup explicitly.')


def _launch(repository, ros_setup, setup, command, environment):
    env = dict(os.environ, **environment)
    launcher = repository / 'scripts' / 'mapping_launch_env.sh'
    # A script file with argv forwarding, never eval or a re-parsed command string.
    os.execvpe('/bin/bash', ['bash', str(launcher), str(ros_setup), str(setup), *command], env)


def main(argv=None):
    args = parser().parse_args(argv)
    repository = Path(os.environ.get('AGT_MAPPING_REPOSITORY', Path(__file__).resolve().parents[3]))
    workspace = repository.parent.parent
    try:
        for name in ('rate', 'startup_timeout', 'export_timeout', 'drain_seconds'):
            setattr(args, name, validate_number(getattr(args, name), name, allow_zero=name == 'drain_seconds'))
        if args.export_timeout <= args.drain_seconds:
            raise PreflightError('--export-timeout must exceed --drain-seconds')
        if not 0 <= args.domain_id <= 101:
            raise PreflightError('--domain-id must be between 0 and 101')
        if args.live:
            if args.bag and not args.output:
                # `--live OUTPUT` is the natural spelling: the positional is the output.
                args.bag, args.output = None, args.bag
            if args.bag:
                raise PreflightError('--live does not take a bag; the sensor is the input')
            if args.start_paused or args.manual_export:
                raise PreflightError('--start-paused/--manual-export apply to bag replay only')
        elif not args.bag:
            raise PreflightError('a rosbag2 directory is required (or use --live)')
        if args.setup:
            setup = Path(args.setup).expanduser().resolve()
        else:
            mapping_setup = workspace / 'install_mapping_framework' / 'setup.bash'
            setup = mapping_setup if mapping_setup.is_file() else workspace / 'install' / 'setup.bash'
        ros_setup = Path(args.ros_setup).expanduser().resolve()
        start_rviz = args.rviz if args.rviz is not None else bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        environment = {'ROS_DOMAIN_ID': str(args.domain_id), 'ROS_LOCALHOST_ONLY': '1'}
        if args.live:
            source, output, command, plan = _live_plan(args, workspace, setup, ros_setup, start_rviz)
            plan['environment'] = environment
            if args.dry_run:
                print(json.dumps(plan, ensure_ascii=False, indent=2))
                return 0
            _require_environment(ros_setup, setup)
            print(f'Live MID360: host {source.host_ip} <- lidar {source.lidar_ip} ({source.path})')
            print(f'Topics: {source.lidar_topic}, {source.imu_topic}\nOutput: {output}\nRaw bag: {output / "raw_bag"}')
            print(f'Overlay: {setup}\nIsolated local ROS domain: {args.domain_id}')
            print('Finish the capture in the SAME domain:')
            print(f'ROS_DOMAIN_ID={args.domain_id} ROS_LOCALHOST_ONLY=1 ros2 service call '
                  '/mapping/session/finish std_srvs/srv/Trigger "{}"')
            print('Launch: ' + shlex.join(command), flush=True)
            _launch(repository, ros_setup, setup, command, environment)
            return 0
        bag = inspect_bag(args.bag, args.lidar_topic, args.imu_topic)
        default = workspace / 'experiments' / 'artifacts' / 'output' / (
            bag.path.name + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        output = check_output(args.output or default, bag.path)
        parameters = {
            'bag_path': str(bag.path), 'output_dir': str(output),
            'lidar_topic': bag.lidar_topic, 'imu_topic': bag.imu_topic,
            'playback_rate': args.rate, 'start_rviz': start_rviz,
            'start_paused': args.start_paused, 'auto_export': not args.manual_export,
            'keep_open': args.keep_open, 'startup_timeout': args.startup_timeout,
            'export_timeout': args.export_timeout, 'drain_seconds': args.drain_seconds,
        }
        command = ['ros2', 'launch', 'agt_mapping_bringup', 'mapping_v0.launch.py']
        command += [f'{key}:={str(value).lower() if isinstance(value, bool) else value}'
                    for key, value in parameters.items()]
        plan = {
            'mode': 'dry-run' if args.dry_run else 'mapping', 'bag': str(bag.path),
            'duration_seconds': bag.duration_seconds, 'message_count': bag.message_count,
            'lidar_topic': bag.lidar_topic, 'imu_topic': bag.imu_topic,
            'output': str(output), 'overlay': str(setup), 'ros_setup': str(ros_setup),
            'environment': environment, 'command': command,
            'runtime_checked': False,
        }
        if args.dry_run:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0
        _require_environment(ros_setup, setup)
        print(f'Input: {bag.path}\nTopics: {bag.lidar_topic}, {bag.imu_topic}\nOutput: {output}')
        print(f'Overlay: {setup}\nIsolated local ROS domain: {args.domain_id}')
        print('Run controls in the SAME domain (do not use the robot runtime domain):')
        prefix = f'ROS_DOMAIN_ID={args.domain_id} ROS_LOCALHOST_ONLY=1 ros2 service call'
        print(prefix + ' /rosbag2_player/pause rosbag2_interfaces/srv/Pause "{}"')
        print(prefix + ' /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"')
        print('Launch: ' + shlex.join(command), flush=True)
        _launch(repository, ros_setup, setup, command, environment)
    except (PreflightError, OSError) as exc:
        print(f'Mapping preflight failed: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
