"""Read-only rosbag and output validation; deliberately independent of ROS."""
from dataclasses import dataclass
import math
from pathlib import Path
import re


class PreflightError(ValueError):
    """An actionable input/environment error, before starting any ROS node."""


@dataclass(frozen=True)
class BagInfo:
    path: Path
    lidar_topic: str
    imu_topic: str
    duration_seconds: float
    message_count: int
    storage_identifier: str


def validate_number(value, name, *, allow_zero=False):
    try:
        number = float(value)
    except (ValueError, TypeError) as exc:
        raise PreflightError(f'{name} must be a finite number: {value}') from exc
    if not math.isfinite(number) or number < 0 or (number == 0 and not allow_zero):
        comparison = 'non-negative' if allow_zero else 'positive'
        raise PreflightError(f'{name} must be finite and {comparison}: {value}')
    return number


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if str(value).lower() not in ('true', 'false'):
        raise PreflightError(f'Expected true or false, got: {value}')
    return str(value).lower() == 'true'


def _load_metadata(path):
    try:
        import yaml
    except ImportError as exc:
        raise PreflightError('PyYAML is required for bag preflight (python3-yaml).') from exc
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise PreflightError(f'Cannot read rosbag metadata {path}: {exc}') from exc
    if not isinstance(document, dict) or not isinstance(
            document.get('rosbag2_bagfile_information'), dict):
        raise PreflightError(f'Not rosbag2 metadata: {path}')
    return document['rosbag2_bagfile_information']


def _select_topic(entries, requested, message_type, flag):
    candidates = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get('topic_metadata'), dict):
            raise PreflightError('Invalid topics_with_message_count entry in bag metadata')
        metadata = entry['topic_metadata']
        if metadata.get('type') != message_type:
            continue
        count = entry.get('message_count', 0)
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            continue
        name = metadata.get('name', '')
        if not isinstance(name, str) or not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', name):
            raise PreflightError(f'Invalid ROS topic name in metadata: {name!r}')
        candidates.append(name)
    if requested and requested != 'auto':
        if requested not in candidates:
            raise PreflightError(f'--{flag} {requested}: no nonempty {message_type} stream in bag')
        return requested
    if not candidates:
        raise PreflightError(f'Bag has no nonempty {message_type} stream. '
                             'Choose a bag containing both raw Livox CustomMsg and IMU data.')
    if len(candidates) != 1:
        raise PreflightError(f'Choose --{flag} explicitly; expected one nonempty {message_type} '
                             f'stream, found {candidates}')
    return candidates[0]


def inspect_bag(path, lidar_topic='', imu_topic=''):
    bag = Path(path).expanduser().resolve()
    if not bag.is_dir():
        raise PreflightError(f'Rosbag directory does not exist: {bag}')
    info = _load_metadata(bag / 'metadata.yaml')
    files = info.get('relative_file_paths')
    if not isinstance(files, list) or not files:
        raise PreflightError('Bag metadata contains no storage files')
    for name in files:
        if not isinstance(name, str) or not name or Path(name).is_absolute():
            raise PreflightError(f'Invalid relative bag storage path: {name!r}')
        file = (bag / name).resolve()
        if '..' in Path(name).parts or not file.is_relative_to(bag):
            raise PreflightError(f'Bag storage path escapes the input directory: {name}')
        if not file.is_file() or file.stat().st_size == 0:
            raise PreflightError(f'Missing or empty bag storage file: {file}')
    entries = info.get('topics_with_message_count')
    if not isinstance(entries, list):
        raise PreflightError('Bag metadata contains no topic information')
    lidar = _select_topic(entries, lidar_topic, 'livox_ros_driver2/msg/CustomMsg', 'lidar-topic')
    imu = _select_topic(entries, imu_topic, 'sensor_msgs/msg/Imu', 'imu-topic')
    count = info.get('message_count')
    if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
        raise PreflightError('Bag message_count must be a positive integer')
    duration = info.get('duration', {})
    if not isinstance(duration, dict):
        raise PreflightError('Invalid bag duration metadata')
    seconds = validate_number(duration.get('nanoseconds', 0), 'bag duration', allow_zero=True) / 1e9
    storage = info.get('storage_identifier')
    if not isinstance(storage, str) or not storage:
        raise PreflightError('Bag metadata is missing storage_identifier')
    return BagInfo(bag, lidar, imu, seconds, count, storage)


def frontend_remappings(config_path, imu_topic):
    """The pinned FAST-LIO2 reads YAML, not imu_topic/lidar_topic ROS parameters."""
    import yaml
    try:
        config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise PreflightError(f'Cannot read FAST-LIO2 config: {exc}') from exc
    if not isinstance(config, dict):
        raise PreflightError('FAST-LIO2 config must be a YAML mapping')
    for key in ('imu_topic', 'lidar_topic'):
        if not isinstance(config.get(key), str) or not config[key].startswith('/'):
            raise PreflightError(f'FAST-LIO2 config requires an absolute {key}')
    return [(config['imu_topic'], imu_topic),
            (config['lidar_topic'], '/mapping/sensor/livox')]


def check_output(path, bag_path):
    """Check only. Do not create directories during --dry-run."""
    output = Path(path).expanduser().resolve()
    bag = Path(bag_path).expanduser().resolve()
    # The existing backend status JSON cannot faithfully encode these path characters.
    if any(character in str(output) for character in ('"', '\\', '\n', '\r')):
        raise PreflightError('Output path cannot contain quotes, backslashes, or newlines '
                             '(current PGO status protocol limitation)')
    if output == bag or output.is_relative_to(bag):
        raise PreflightError(f'Output must be outside the read-only input bag: {output}')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise PreflightError(f'Output is not empty; choose a new directory: {output}')
    ancestor = output.parent
    while not ancestor.exists():
        ancestor = ancestor.parent
    if not ancestor.is_dir():
        raise PreflightError(f'Output parent is not a directory: {ancestor}')
    return output
