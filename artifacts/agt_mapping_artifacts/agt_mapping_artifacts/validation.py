"""Read-only verification of complete, nonempty optimized PGO artifacts."""
import argparse
import hashlib
from pathlib import Path, PurePosixPath
import re
import sys
import time

import yaml


class ArtifactValidationError(ValueError):
    """The package is incomplete, inconsistent, unsafe, or not optimized PGO."""


REQUIRED = {'map.pcd', 'poses.txt', 'poses_timed.txt', 'calibration.yaml',
            'metadata.yaml', 'manifest.yaml'}


def _nonempty_pcd(path):
    points = None
    with path.open('rb') as stream:
        total = 0
        while total < 65536:
            line = stream.readline(4096)
            total += len(line)
            if not line or len(line) == 4096:
                break
            fields = line.strip().split()
            if fields[:1] == [b'POINTS']:
                try:
                    points = int(fields[1])
                except (IndexError, ValueError) as exc:
                    raise ArtifactValidationError('Invalid PCD POINTS header') from exc
            if fields[:1] == [b'DATA']:
                if len(fields) != 2 or fields[1] not in (b'ascii', b'binary', b'binary_compressed'):
                    raise ArtifactValidationError('Invalid PCD DATA encoding')
                if points is None or points <= 0 or not stream.read(1):
                    raise ArtifactValidationError('Map PCD has no points or point payload')
                return
    raise ArtifactValidationError('Map PCD has no valid bounded header/DATA section')


def _checksum_index(path):
    records = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        match = re.fullmatch(r'([0-9a-fA-F]{64}) [ *](.+)', line)
        if not match:
            raise ArtifactValidationError('Malformed checksum entry')
        digest, name = match.groups()
        relative = PurePosixPath(name)
        if (relative.is_absolute() or '..' in relative.parts or '\\' in name
                or relative.as_posix() != name or name == 'checksums.sha256'
                or name in records):
            raise ArtifactValidationError(f'Unsafe or duplicate checksum path: {name}')
        records[name] = digest.lower()
    if not records:
        raise ArtifactValidationError('Empty checksum list')
    return records


def _check_deadline(deadline):
    if deadline is not None and time.monotonic() >= deadline:
        raise TimeoutError('Artifact verification deadline exceeded')


def verify_artifact(path, *, deadline=None):
    """Return the package path only after full coverage and SHA-256 verification."""
    root = Path(path).expanduser().resolve()
    if (root / 'map_package').is_dir():
        root = root / 'map_package'
    try:
        _check_deadline(deadline)
        if not root.is_dir():
            raise ArtifactValidationError(f'Artifact directory does not exist: {root}')
        actual = set()
        for file in root.rglob('*'):
            _check_deadline(deadline)
            if file.is_symlink():
                raise ArtifactValidationError(f'Artifact contains a symbolic link: {file}')
            if file.is_file() and file.relative_to(root).as_posix() != 'checksums.sha256':
                actual.add(file.relative_to(root).as_posix())
        for name in REQUIRED | {'checksums.sha256'}:
            file = root / name
            if not file.is_file() or file.stat().st_size == 0:
                raise ArtifactValidationError(f'Missing or empty artifact file: {name}')
        if not any(name.startswith('patches/') and name.endswith('.pcd') for name in actual):
            raise ArtifactValidationError('Artifact contains no keyframe patches')
        metadata = yaml.safe_load((root / 'metadata.yaml').read_text(encoding='utf-8'))
        if not isinstance(metadata, dict):
            raise ArtifactValidationError('Invalid metadata document')
        status = metadata.get('backend_status')
        if (metadata.get('backend') != 'PGO' or not isinstance(status, dict)
                or status.get('optimized') is not True):
            raise ArtifactValidationError('Metadata does not attest optimized PGO output')
        manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
        if not isinstance(manifest, dict):
            raise ArtifactValidationError('Invalid manifest document')
        _nonempty_pcd(root / 'map.pcd')
        checksums = _checksum_index(root / 'checksums.sha256')
        if set(checksums) != actual:
            missing = sorted(actual - set(checksums))
            absent = sorted(set(checksums) - actual)
            raise ArtifactValidationError(f'Incomplete checksum coverage; unlisted={missing}, absent={absent}')
        for name, expected in checksums.items():
            digest = hashlib.sha256()
            with (root / name).open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    _check_deadline(deadline)
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                raise ArtifactValidationError(f'checksum mismatch: {name}')
        _check_deadline(deadline)
        return root
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ArtifactValidationError(f'Cannot verify artifact: {exc}') from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Run output directory or map_package directory')
    args = parser.parse_args(argv)
    try:
        root = verify_artifact(args.output)
    except ArtifactValidationError as exc:
        print(f'Artifact NOT verified: {exc}', file=sys.stderr)
        return 1
    print(f'Artifact verified: {root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
