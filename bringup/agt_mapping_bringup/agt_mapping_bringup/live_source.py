"""Live MID360 source preflight; deliberately independent of ROS.

The offline workflow validates a rosbag before any node starts. The live
workflow validates the Livox driver configuration (network JSON) the same way
so that an unreachable or misconfigured sensor fails fast with an actionable
message instead of a silently idle FAST-LIO2.
"""
from dataclasses import dataclass
import ipaddress
import json
from pathlib import Path
import re

from .preflight import PreflightError

DEFAULT_LIDAR_TOPIC = '/livox/lidar'
DEFAULT_IMU_TOPIC = '/livox/imu'
DEFAULT_FRAME_ID = 'livox_frame'
TOPIC_PATTERN = re.compile(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+')


@dataclass(frozen=True)
class LiveSource:
    """Shape-compatible with BagInfo for session_state.create_session."""
    path: Path            # livox_ros_driver2 user config JSON
    lidar_topic: str
    imu_topic: str
    duration_seconds: float
    message_count: int
    storage_identifier: str
    host_ip: str
    lidar_ip: str
    publish_freq: float


def _validate_topic(name, flag):
    if not isinstance(name, str) or not TOPIC_PATTERN.fullmatch(name):
        raise PreflightError(f'--{flag} must be an absolute ROS topic name, got {name!r}')
    return name


def inspect_live_config(path, lidar_topic=DEFAULT_LIDAR_TOPIC, imu_topic=DEFAULT_IMU_TOPIC,
                        publish_freq=10.0):
    """Validate a livox_ros_driver2 MID360 JSON configuration without ROS."""
    config = Path(path).expanduser().resolve()
    if not config.is_file():
        raise PreflightError(f'Livox config JSON does not exist: {config}')
    try:
        document = json.loads(config.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise PreflightError(f'Cannot read Livox config {config}: {exc}') from exc
    if not isinstance(document, dict):
        raise PreflightError(f'Livox config must be a JSON object: {config}')
    mid360 = document.get('MID360')
    if not isinstance(mid360, dict):
        raise PreflightError('Livox config has no "MID360" section (single MID360 is required)')
    host = mid360.get('host_net_info')
    if not isinstance(host, dict):
        raise PreflightError('Livox config MID360.host_net_info is missing')
    host_ips = {host.get(key) for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')}
    if len(host_ips) != 1:
        raise PreflightError(f'Livox host_net_info must use one host IP for all streams, got {sorted(map(str, host_ips))}')
    host_ip = host_ips.pop()
    try:
        ipaddress.ip_address(host_ip)
    except ValueError as exc:
        raise PreflightError(f'Livox host IP is invalid: {host_ip!r}') from exc
    lidars = document.get('lidar_configs')
    if not isinstance(lidars, list) or len(lidars) != 1:
        raise PreflightError('Exactly one entry is required in lidar_configs (single MID360 mode)')
    lidar_ip = lidars[0].get('ip') if isinstance(lidars[0], dict) else None
    try:
        ipaddress.ip_address(lidar_ip)
    except (ValueError, TypeError) as exc:
        raise PreflightError(f'lidar_configs[0].ip is invalid: {lidar_ip!r}') from exc
    if lidar_ip == host_ip:
        raise PreflightError('Lidar IP and host IP must differ')
    for topic, flag in ((lidar_topic, 'lidar-topic'), (imu_topic, 'imu-topic')):
        _validate_topic(topic, flag)
    if lidar_topic == imu_topic:
        raise PreflightError('lidar and IMU topics must differ')
    if publish_freq not in (5.0, 10.0, 20.0, 50.0):
        raise PreflightError('--publish-freq must be one of 5, 10, 20, 50 Hz (Livox driver constraint)')
    if lidar_topic != DEFAULT_LIDAR_TOPIC or imu_topic != DEFAULT_IMU_TOPIC:
        # livox_ros_driver2 (multi_topic=0) always publishes /livox/lidar and
        # /livox/imu; other names would need a relay. Keep the contract explicit.
        raise PreflightError('Live mode uses the driver topics /livox/lidar and /livox/imu '
                             '(multi_topic=0). Remapping is not supported here.')
    return LiveSource(config, lidar_topic, imu_topic, 0.0, 0, 'live', host_ip, lidar_ip,
                      float(publish_freq))


def host_interface_hint(host_ip):
    """Shell-free hint used in error messages and dry-run output."""
    return (f'The host must own {host_ip} on the interface connected to the MID360 '
            f'(e.g. sudo ip addr add {host_ip}/24 dev <iface>).')
