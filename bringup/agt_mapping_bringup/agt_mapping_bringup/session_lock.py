"""Prevent two local sessions from publishing into the same ROS domain."""
import fcntl
import os
from pathlib import Path
import stat
import tempfile

from .preflight import PreflightError


def acquire_domain_lease(domain_id=None, *, directory=None):
    value = os.environ.get('ROS_DOMAIN_ID', '0') if domain_id is None else domain_id
    try:
        domain = int(value)
    except (TypeError, ValueError) as exc:
        raise PreflightError(f'Invalid ROS_DOMAIN_ID: {value}') from exc
    if not 0 <= domain <= 232:
        raise PreflightError(f'Invalid ROS_DOMAIN_ID: {domain}')
    root = Path(directory) if directory is not None else Path(tempfile.gettempdir())
    path = root / f'agt-mapping-{os.getuid()}-domain-{domain}.lock'
    flags = os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
    fd = os.open(path, flags, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
            raise PreflightError(f'Unsafe mapping lock file: {path}')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise PreflightError(f'Another mapping session uses ROS domain {domain}. '
                                 'Choose a different --domain-id or finish that session first.') from exc
        # Never unlink a flock file: contenders must continue locking the same inode.
        return os.fdopen(fd, 'r+')
    except Exception:
        os.close(fd)
        raise
