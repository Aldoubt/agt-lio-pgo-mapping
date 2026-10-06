"""Backend-neutral writer and exact validator for LIO frontend map packages."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml


BACKEND_IDS = {
    'lio_sam_noloop', 'point_lio', 'fast_livo2_lio',
    'fast_lio2_legacy', 'lio_sam_loop_experimental',
}
TOLERANCE_M = 5e-6


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def path_is_same_or_within(path: str | Path, root: str | Path) -> bool:
    """Resolve existing symlink parents before checking an output boundary."""
    candidate = Path(path).expanduser().resolve(strict=False)
    boundary = Path(root).expanduser().resolve(strict=False)
    return candidate == boundary or boundary in candidate.parents


def require_output_outside(path: str | Path, protected_roots: Iterable[str | Path], *,
                           label: str = 'output') -> Path:
    """Reject new output paths inside immutable inputs/assets, including symlink paths."""
    candidate = Path(path).expanduser().resolve(strict=False)
    for protected in protected_roots:
        boundary = Path(protected).expanduser().resolve(strict=False)
        if path_is_same_or_within(candidate, boundary):
            raise ValueError(f'{label} must be outside protected asset: {boundary}')
    for ancestor in (candidate, *candidate.parents):
        manifest_path = ancestor / 'manifest.yaml'
        if not manifest_path.is_file() or manifest_path.is_symlink():
            continue
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            continue
        if isinstance(manifest, dict) and manifest.get('asset_type') in {
                'agt.keyframe_block_set/v1', 'agt.relocalization_evidence/v1'}:
            raise ValueError(f'{label} must be outside immutable asset bundle: {ancestor}')
    return candidate


def rotation_from_xyzw(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid zero/nonfinite quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype='f8')


def _write_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 4 or not len(points):
        raise ValueError(f'PCD requires nonempty Nx4 XYZI data: {path}')
    if not np.isfinite(points).all():
        raise ValueError(f'PCD contains nonfinite points: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
        'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n'
    ).encode('ascii')
    with path.open('xb') as stream:
        stream.write(header)
        np.ascontiguousarray(points).tofile(stream)


def _read_pcd(path: Path) -> np.ndarray:
    fields = None
    count = None
    with Path(path).open('rb') as stream:
        for _ in range(64):
            line = stream.readline(4096)
            if not line:
                raise ValueError(f'incomplete PCD header: {path}')
            tokens = line.decode('ascii').strip().split()
            if not tokens or tokens[0].startswith('#'):
                continue
            if tokens[0] == 'FIELDS':
                fields = tokens[1:]
            elif tokens[0] == 'POINTS':
                count = int(tokens[1])
            elif tokens[0] == 'DATA':
                if tokens[1:] != ['binary'] or fields != ['x', 'y', 'z', 'intensity'] or count is None:
                    raise ValueError(f'unsupported PCD layout: {path}')
                payload = stream.read()
                if len(payload) != count * 16 or count < 1:
                    raise ValueError(f'PCD payload does not match its header: {path}')
                points = np.frombuffer(payload, dtype='<f4').reshape(count, 4)
                if not np.isfinite(points).all():
                    raise ValueError(f'PCD contains nonfinite values: {path}')
                return points
    raise ValueError(f'PCD DATA record missing: {path}')


def _hash_files(root: Path, *, exclude: set[str]) -> dict[str, str]:
    return {
        item.relative_to(root).as_posix(): sha256(item)
        for item in sorted(root.rglob('*'))
        if item.is_file() and item.relative_to(root).as_posix() not in exclude
    }


def _pose_line(name: str, record: dict[str, Any], timed: bool) -> str:
    position = [float(v) for v in record['position']]
    qx, qy, qz, qw = [float(v) for v in record['quaternion_xyzw']]
    prefix = name + ' '
    if timed:
        prefix += f"{int(record['stamp_sec'])}.{int(record['stamp_nanosec']):09d} "
    return prefix + ' '.join(f'{v:.12g}' for v in (*position, qw, qx, qy, qz)) + '\n'


def write_frontend_map_package(
    package_root: str | Path, records: list[dict[str, Any]], provenance: dict[str, Any],
) -> dict[str, Any]:
    """Write patches, poses, and map from the same normalized cloud/pose pairs."""
    root = Path(package_root)
    backend = provenance.get('mapping_backend')
    if not isinstance(backend, dict) or backend.get('id') not in BACKEND_IDS:
        raise ValueError('mapping_backend.id is missing or unsupported')
    required = ('project', 'mode', 'source_commit', 'config_sha256', 'loop_closure', 'gps_factor',
                'external_global_correction')
    if any(key not in backend for key in required):
        raise ValueError('mapping_backend provenance is incomplete')
    for key in ('loop_closure', 'gps_factor', 'external_global_correction'):
        if not isinstance(backend[key], bool):
            raise ValueError(f'mapping_backend.{key} must be an explicit boolean')
    if not isinstance(backend['source_commit'], str) or not backend['source_commit']:
        raise ValueError('mapping_backend.source_commit must be recorded')
    if (not isinstance(backend['config_sha256'], str) or len(backend['config_sha256']) != 64
            or any(c not in '0123456789abcdef' for c in backend['config_sha256'].lower())):
        raise ValueError('mapping_backend.config_sha256 must be a SHA-256 digest')
    if backend['mode'] == 'no_loop' and (backend['loop_closure'] or backend['gps_factor']):
        raise ValueError('no-loop package provenance may not enable loop closure or GPS')
    if provenance.get('reference', {}).get('absolute_ground_truth') is not False:
        raise ValueError('frontend trajectory may not be labelled as absolute ground truth')
    if root.exists():
        raise FileExistsError(f'refusing to overwrite map package: {root}')
    if len(records) < 7:
        raise ValueError(f'map package needs at least seven paired keyframes, got {len(records)}')

    root.mkdir(parents=True)
    patches = root / 'patches'
    patches.mkdir()
    poses, timed_poses, world_clouds = [], [], []
    previous_stamp = -math.inf
    point_count = 0
    for index, record in enumerate(records):
        stamp = int(record['stamp_sec']) + int(record['stamp_nanosec']) * 1e-9
        if stamp <= previous_stamp:
            raise ValueError('frontend keyframe timestamps must be strictly increasing')
        previous_stamp = stamp
        points = np.asarray(record['points_xyzi'], dtype='<f4')
        if points.ndim != 2 or points.shape[1] != 4 or not len(points) or not np.isfinite(points).all():
            raise ValueError(f'keyframe {index} cloud is empty or invalid')
        name = f'{index}.pcd'
        _write_pcd(patches / name, points)
        rotation = rotation_from_xyzw(record['quaternion_xyzw'])
        position = np.asarray(record['position'], dtype='f8')
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError(f'keyframe {index} pose position is invalid')
        world = points.copy()
        world[:, :3] = (points[:, :3].astype('f8') @ rotation.T + position).astype('<f4')
        world_clouds.append(world)
        point_count += len(points)
        poses.append(_pose_line(name, record, timed=False))
        timed_poses.append(_pose_line(name, record, timed=True))

    _write_pcd(root / 'map.pcd', np.concatenate(world_clouds, axis=0))
    (root / 'poses.txt').write_text(''.join(poses), encoding='ascii')
    (root / 'poses_timed.txt').write_text(''.join(timed_poses), encoding='ascii')
    frames = provenance.get('frames', {})
    source = provenance.get('source', {})
    metadata = {
        'format_version': 1,
        'artifact_kind': 'frontend_mapping_map_package',
        'reference_type': 'FRONTEND_SAME_SESSION_REFERENCE',
        'mapping_backend': backend,
        'source': source,
        'frames': frames,
        'reference': provenance['reference'],
        'pose_semantics': 'T_frontend_body',
        'patches_frame': 'body',
        'keyframe_count': len(records),
        'map_point_count': point_count,
        'outputs': {
            'map': 'map.pcd', 'poses': 'poses.txt', 'poses_timed': 'poses_timed.txt',
            'patches_dir': 'patches', 'calibration': 'calibration.yaml',
        },
    }
    calibration = provenance.get('calibration', {'status': 'source backend extrinsic'} )
    (root / 'calibration.yaml').write_text(yaml.safe_dump(calibration, sort_keys=True), encoding='utf-8')
    (root / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
    payload = _hash_files(root, exclude={'manifest.yaml', 'checksums.sha256'})
    manifest = {
        'schema_version': 1,
        'package_kind': 'mapping_source',
        'mapping_backend': backend,
        'reference': provenance['reference'],
        'source': source,
        'files': payload,
    }
    (root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')
    checksums = _hash_files(root, exclude={'checksums.sha256'})
    (root / 'checksums.sha256').write_text(
        ''.join(f'{digest}  {name}\n' for name, digest in checksums.items()), encoding='ascii')
    return verify_frontend_map_package(root)


def verify_frontend_map_package(package_root: str | Path) -> dict[str, Any]:
    """Verify checksums and that every map segment is exactly patch_i + pose_i."""
    source_root = Path(package_root)
    if source_root.is_symlink():
        raise ValueError('map package root may not be a symbolic link')
    root = source_root.resolve(strict=True)
    required = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'patches', 'calibration.yaml',
                'metadata.yaml', 'manifest.yaml', 'checksums.sha256')
    if not root.is_dir() or any(not (root / name).exists() for name in required):
        raise ValueError('map package is missing a required file or directory')
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text(encoding='utf-8'))
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
    if not isinstance(metadata, dict) or not isinstance(manifest, dict):
        raise ValueError('metadata and manifest must be YAML mappings')
    backend = metadata.get('mapping_backend')
    if not isinstance(backend, dict) or backend.get('id') not in BACKEND_IDS:
        raise ValueError('metadata has no supported mapping_backend identity')
    if manifest.get('mapping_backend') != backend:
        raise ValueError('manifest and metadata backend identities differ')
    reference = metadata.get('reference', {})
    if (reference.get('same_session') is not True or
            reference.get('absolute_ground_truth') is not False or
            manifest.get('reference') != reference):
        raise ValueError('reference metadata must be same-session and non-ground-truth')

    indexed = {}
    for line in (root / 'checksums.sha256').read_text(encoding='ascii').splitlines():
        pair = line.split('  ', 1)
        if len(pair) != 2 or len(pair[0]) != 64:
            raise ValueError(f'malformed checksum row: {line!r}')
        digest, name = pair
        path = Path(name)
        if path.is_absolute() or '..' in path.parts or name in indexed:
            raise ValueError(f'unsafe or repeated checksum path: {name}')
        target = root / path
        if target.is_symlink() or not target.is_file() or sha256(target) != digest:
            raise ValueError(f'checksum mismatch or missing file: {name}')
        indexed[name] = digest
    actual = set(_hash_files(root, exclude={'checksums.sha256'}))
    if set(indexed) != actual:
        raise ValueError('checksum file does not cover the package exactly once')
    files = manifest.get('files', {})
    if set(files) != actual - {'manifest.yaml'}:
        raise ValueError('manifest file list does not cover package payload exactly once')
    if any(indexed.get(name) != digest for name, digest in files.items()):
        raise ValueError('manifest payload checksum mismatch')

    timed = (root / 'poses_timed.txt').read_text(encoding='ascii').splitlines()
    plain = (root / 'poses.txt').read_text(encoding='ascii').splitlines()
    if len(timed) < 7 or len(timed) != len(plain) or len(timed) != metadata.get('keyframe_count'):
        raise ValueError('map package has invalid pose/keyframe count')
    map_points = _read_pcd(root / 'map.pcd')
    cursor = 0
    prev_stamp = -math.inf
    names = set()
    for index, (timed_line, plain_line) in enumerate(zip(timed, plain)):
        tokens = timed_line.split()
        old = plain_line.split()
        if len(tokens) != 9 or len(old) != 8 or tokens[0] != old[0]:
            raise ValueError(f'pose format mismatch at keyframe {index}')
        patch_name = tokens[0]
        if Path(patch_name).name != patch_name or not patch_name.endswith('.pcd') or patch_name in names:
            raise ValueError(f'unsafe or duplicate patch reference: {patch_name}')
        names.add(patch_name)
        values = np.asarray([float(value) for value in tokens[1:]], dtype='f8')
        if not np.isfinite(values).all():
            raise ValueError(f'nonfinite pose at keyframe {index}')
        stamp = values[0]
        if stamp <= prev_stamp:
            raise ValueError('pose timestamps are not strictly increasing')
        prev_stamp = stamp
        if not np.allclose(np.asarray([float(v) for v in old[1:]], dtype='f8'),
                           values[1:], rtol=0.0, atol=1e-10):
            raise ValueError(f'poses.txt and poses_timed.txt differ at keyframe {index}')
        patch_path = root / 'patches' / patch_name
        if patch_path.is_symlink() or not patch_path.is_file():
            raise ValueError(f'missing/unsafe patch: {patch_name}')
        patch = _read_pcd(patch_path)
        qw, qx, qy, qz = values[4:8]
        quat_norm = float(np.linalg.norm([qx, qy, qz, qw]))
        if abs(quat_norm - 1.0) > 1e-3:
            raise ValueError(f'invalid quaternion at keyframe {index}')
        expected = patch[:, :3].astype('f8') @ rotation_from_xyzw([qx, qy, qz, qw]).T + values[1:4]
        next_cursor = cursor + len(patch)
        if next_cursor > len(map_points):
            raise ValueError('map has fewer points than its patches')
        if not np.allclose(map_points[cursor:next_cursor, :3], expected.astype('<f4'), rtol=0.0, atol=TOLERANCE_M):
            raise ValueError(f'map/patch/pose geometry differs by more than {TOLERANCE_M} m at keyframe {index}')
        if not np.array_equal(map_points[cursor:next_cursor, 3], patch[:, 3]):
            raise ValueError(f'map/patch intensity differs at keyframe {index}')
        cursor = next_cursor
    if cursor != len(map_points) or len(map_points) != metadata.get('map_point_count'):
        raise ValueError('map point count differs from patched pose sequence')
    if {path.name for path in (root / 'patches').iterdir()} != names:
        raise ValueError('patch directory and pose references differ')
    return {'status': 'PASS', 'backend_id': backend['id'], 'keyframes': len(timed),
            'map_points': cursor, 'map_frame': metadata.get('frames', {}).get('map')}
