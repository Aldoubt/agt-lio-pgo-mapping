"""Integrity and immutable publication helpers for offline derived outputs.

Uses the existing artifact manifest/checksums layout. Cleanup clouds are review
attachments, not a replacement mapping source, READY Site, or semantic detector.
"""
from __future__ import annotations

import argparse
import ctypes
import errno
import os
from pathlib import Path
import sys

import yaml

from .artifact_writer import sha256, write_checksums
from .frontend_package import require_output_outside
from .validation import verify_source_integrity


def publish_directory(stage: Path, output: Path) -> None:
    """Atomically publish on Linux without replacing even an empty destination."""
    stage, output = Path(stage), Path(output)
    if stage.parent.resolve() != output.parent.resolve():
        raise ValueError('Staging and destination must share a parent filesystem')
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise OSError('Atomic no-replace publication requires renameat2')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    for item in stage.rglob('*'):
        if item.is_symlink():
            raise ValueError('Staging may not contain symbolic links')
        if item.is_file():
            with item.open('rb') as stream:
                os.fsync(stream.fileno())
    descriptor = os.open(stage, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if rename(-100, os.fsencode(stage), -100, os.fsencode(output), 1) != 0:
        code = ctypes.get_errno()
        if code in (errno.EEXIST, errno.ENOTEMPTY):
            raise FileExistsError(f'Refusing to replace existing output: {output}')
        raise OSError(code, os.strerror(code), str(output))
    descriptor = os.open(output.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def snapshot_source(source: str | Path) -> dict:
    root = Path(source).expanduser().resolve(strict=True)
    return {'path': str(root), 'files': verify_source_integrity(source)}


def require_current_source(source: str | Path, snapshot: dict) -> None:
    if snapshot_source(source) != snapshot:
        raise ValueError('Source changed during derivation; no output published')


def prepare_cleanup(source: Path, stage: Path, output: Path, producer_version: str,
                    cloud_subdirectory: str = '.', projection_source: Path | None = None) -> None:
    if output.exists() or output.is_symlink():
        raise FileExistsError(f'Output already exists: {output}')
    output = require_output_outside(output, [source], label='Cleanup output')
    if output.exists() or output.is_symlink():
        raise FileExistsError(f'Output already exists: {output}')
    require_output_outside(stage, [source], label='Cleanup staging')
    if stage.resolve().parent != output.parent:
        raise ValueError('Staging must be a sibling of output')
    if not producer_version:
        raise ValueError('Producer version is required')
    relative = Path(cloud_subdirectory)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Cloud output must be inside the transaction root')
    projection = yaml.safe_load((stage / 'projection.yaml').read_text(encoding='utf-8'))
    if not isinstance(projection, dict) or projection.get('projection_mode') != 'traversability':
        raise ValueError('Cleanup requires effective traversability parameters')
    robot = projection.get('robot')
    if not isinstance(robot, dict) or not robot.get('footprint') or not robot.get('base_from_body'):
        raise ValueError('Effective projection parameters must contain the robot geometry')
    snapshot = {
        'source': snapshot_source(source),
        'projection_sha256': sha256(stage / 'projection.yaml'),
        'producer': {'package': 'agt_pcd2grid_exporter', 'version': producer_version},
        'output': str(output),
        'cloud_subdirectory': relative.as_posix(),
    }
    if projection_source is not None:
        projection_source = projection_source.resolve(strict=True)
        digest = sha256(projection_source)
        if sha256(stage / 'input_projection.yaml') != digest:
            raise ValueError('Selected projection config changed while copying')
        snapshot['projection_input'] = {'path': str(projection_source), 'sha256': digest}
    (stage / '.input_snapshot.yaml').write_text(yaml.safe_dump(snapshot, sort_keys=True), encoding='utf-8')


def finish_cleanup(stage: Path, output: Path) -> None:
    snapshot_path = stage / '.input_snapshot.yaml'
    snapshot = yaml.safe_load(snapshot_path.read_text(encoding='utf-8'))
    if str(output.resolve(strict=False)) != snapshot['output']:
        raise ValueError('Output differs from prepared destination')
    source = Path(snapshot['source']['path'])
    require_output_outside(output, [source], label='Cleanup output')
    require_current_source(source, snapshot['source'])
    if sha256(stage / 'projection.yaml') != snapshot['projection_sha256']:
        raise ValueError('Effective projection changed during export')
    projection_input = snapshot.get('projection_input')
    if projection_input and (sha256(Path(projection_input['path'])) != projection_input['sha256'] or
                             sha256(stage / 'input_projection.yaml') != projection_input['sha256']):
        raise ValueError('Selected projection config changed during export')
    cloud_root = Path(snapshot['cloud_subdirectory'])
    required = [cloud_root / name for name in ('static_map.pcd', 'removed_candidates.pcd',
                                               'cloud_cleanup.yaml')]
    required.extend(Path(name) for name in ('map.pgm', 'map.yaml', 'metadata.yaml'))
    for name in required:
        if not (stage / name).is_file() or (stage / name).stat().st_size == 0:
            raise ValueError(f'Incomplete cleanup output: {name}')
    metadata_path = stage / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['derivation'] = {
        'status': 'REVIEW_ONLY_NOT_READY',
        'producer': snapshot['producer'],
        'source': snapshot['source'],
        'projection_sha256': snapshot['projection_sha256'],
        'projection_input': projection_input,
        'cloud_subdirectory': snapshot['cloud_subdirectory'],
        'robot_geometry': 'robot section of the bound projection.yaml; no execution-profile verification',
        'cloud_fields': ['x', 'y', 'z'],
        'cloud_semantics': 'persistent observations plus all classified ground; not grid accepted-points',
        'removed_semantics': 'unconfirmed candidates; not semantically detected people',
        'patches': 'immutable parent evidence; these clouds are not a paired mapping source',
        'limitations': ['near-only static obstacles can be removed',
                        'stationary dynamic objects can be retained',
                        'trajectory footprint sweep can still mark driven cells free'],
    }
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
    snapshot_path.unlink()
    checksums = write_checksums(stage)
    (stage / 'manifest.yaml').write_text(
        yaml.safe_dump({'format_version': 1, 'checksums': checksums}, sort_keys=True), encoding='utf-8')
    write_checksums(stage)
    # Check again after output serialization, immediately before publication.
    require_current_source(source, snapshot['source'])
    if projection_input and sha256(Path(projection_input['path'])) != projection_input['sha256']:
        raise ValueError('Selected projection config changed before publication')
    publish_directory(stage, output)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='operation', required=True)
    prepare = subparsers.add_parser('prepare')
    prepare.add_argument('--source', type=Path, required=True)
    prepare.add_argument('--stage', type=Path, required=True)
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--producer-version', required=True)
    prepare.add_argument('--cloud-subdirectory', default='.')
    prepare.add_argument('--projection-source', type=Path)
    finish = subparsers.add_parser('finish')
    finish.add_argument('--stage', type=Path, required=True)
    finish.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.operation == 'prepare':
            prepare_cleanup(args.source, args.stage, args.output, args.producer_version,
                            args.cloud_subdirectory, args.projection_source)
        else:
            finish_cleanup(args.stage, args.output)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'Derived export refused: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
