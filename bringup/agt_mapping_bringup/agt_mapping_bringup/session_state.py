"""Small, ROS-independent session journal and lifecycle policy."""
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

from .preflight import check_output

TERMINAL_STATES = {'completed', 'failed', 'cancelled'}


def _timestamp():
    return datetime.now(timezone.utc).isoformat()


def create_session(output, bag, options):
    output = check_output(output, bag.path)
    output.mkdir(parents=True, exist_ok=True)
    now = _timestamp()
    record = {
        'schema_version': 1, 'session_id': uuid.uuid4().hex,
        'status': 'preparing', 'created_at': now, 'updated_at': now,
        'bag_path': str(bag.path), 'lidar_topic': bag.lidar_topic,
        'imu_topic': bag.imu_topic, 'options': options,
        'history': [{'status': 'preparing', 'time': now, 'message': 'Input preflight passed'}],
    }
    # Exclusive creation also prevents two launches from reserving the same output.
    with (output / 'session.json').open('x', encoding='utf-8') as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return record


def mark_session(output, status, message, **details):
    output = Path(output)
    destination = output / 'session.json'
    record = json.loads(destination.read_text(encoding='utf-8'))
    if record['status'] in TERMINAL_STATES:
        return record
    now = _timestamp()
    record.update(status=status, updated_at=now, message=message, **details)
    record['history'].append({'status': status, 'time': now, 'message': message})
    temporary = output / ('.session-' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return record


def playback_action(returncode, shutting_down, auto_export):
    """Never export after playback failure or launch cancellation."""
    if shutting_down:
        return 'stop'
    if returncode != 0:
        return 'fail'
    return 'export' if auto_export else 'manual'
