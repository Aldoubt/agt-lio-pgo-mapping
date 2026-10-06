"""Deterministic, read-only-input keyframe block assets for validated frontend maps.

Block PCDs are expressed in the source map frame. Their bytes, the ordered
source patch identities, source pose revision, partition policy and block
index are committed independently in a new block-set manifest/checksum index.
This module contains no registration or SLAM implementation.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
from typing import Any, Iterable

import numpy as np
import yaml

from .frontend_package import (
    _read_pcd, _write_pcd, rotation_from_xyzw, sha256, require_output_outside,
    verify_frontend_map_package,
)


BLOCK_SCHEMA_VERSION = 1
BLOCK_ASSET_TYPE = 'agt.keyframe_block_set/v1'
UNKNOWN = 'UNKNOWN'


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def _finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(result) or (positive and result <= 0):
        qualifier = 'positive and finite' if positive else 'finite'
        raise ValueError(f'{name} must be {qualifier}')
    return result


def _read_pose_records(source_root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    seen_ids: set[int] = set()
    previous_timestamp = -math.inf
    for line_number, line in enumerate((source_root / 'poses_timed.txt').read_text(encoding='ascii').splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        tokens = line.split()
        if len(tokens) != 9:
            raise ValueError(f'poses_timed.txt:{line_number}: expected 9 columns')
        patch_name = tokens[0]
        if Path(patch_name).name != patch_name or not patch_name.endswith('.pcd'):
            raise ValueError(f'poses_timed.txt:{line_number}: unsafe patch name')
        stem = Path(patch_name).stem
        if not stem.isdigit():
            raise ValueError(f'poses_timed.txt:{line_number}: patch ID must be numeric')
        keyframe_id = int(stem)
        if keyframe_id in seen_ids:
            raise ValueError(f'poses_timed.txt:{line_number}: duplicate keyframe ID {keyframe_id}')
        seen_ids.add(keyframe_id)
        values = np.asarray([float(token) for token in tokens[1:]], dtype='f8')
        if not np.isfinite(values).all():
            raise ValueError(f'poses_timed.txt:{line_number}: nonfinite pose or timestamp')
        timestamp, x, y, z, qw, qx, qy, qz = map(float, values)
        if timestamp <= previous_timestamp:
            raise ValueError(f'poses_timed.txt:{line_number}: timestamps must be strictly increasing')
        previous_timestamp = timestamp
        quaternion_xyzw = np.asarray([qx, qy, qz, qw], dtype='f8')
        rotation = rotation_from_xyzw(quaternion_xyzw)
        records.append({
            'keyframe_id': keyframe_id, 'patch_id': patch_name,
            'timestamp': timestamp, 'position': np.asarray([x, y, z], dtype='f8'),
            'quaternion_xyzw': quaternion_xyzw, 'rotation': rotation,
        })
    if not records:
        raise ValueError('source package has no timed poses')
    return records


def partition_pose_records(records: list[dict[str, Any]], *, block_keyframe_count: int,
                           stride: int, max_timestamp_gap_s: float) -> list[list[dict[str, Any]]]:
    """Partition by temporal/keyframe continuity only; spatial revisits never merge."""
    if isinstance(block_keyframe_count, bool) or not isinstance(block_keyframe_count, int) or block_keyframe_count < 1:
        raise ValueError('block_keyframe_count must be a positive integer')
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError('stride must be a positive integer')
    gap = _finite_number(max_timestamp_gap_s, 'max_timestamp_gap_s', positive=True)
    if not records:
        raise ValueError('cannot partition an empty pose sequence')

    segments: list[list[dict[str, Any]]] = []
    active: list[dict[str, Any]] = []
    last: dict[str, Any] | None = None
    seen: set[int] = set()
    prior_stamp = -math.inf
    for record in records:
        keyframe_id = record.get('keyframe_id')
        timestamp = _finite_number(record.get('timestamp'), 'timestamp')
        if isinstance(keyframe_id, bool) or not isinstance(keyframe_id, int) or keyframe_id < 0:
            raise ValueError('keyframe_id must be a nonnegative integer')
        if keyframe_id in seen:
            raise ValueError(f'duplicate keyframe ID {keyframe_id}')
        if timestamp <= prior_stamp:
            raise ValueError('timestamps must be strictly increasing')
        seen.add(keyframe_id)
        discontinuity = (last is not None and
                         (keyframe_id != last['keyframe_id'] + 1 or
                          timestamp - last['timestamp'] > gap))
        if discontinuity:
            segments.append(active)
            active = []
        active.append(record)
        last = record
        prior_stamp = timestamp
    if active:
        segments.append(active)

    blocks: list[list[dict[str, Any]]] = []
    for segment in segments:
        for start in range(0, len(segment), stride):
            block = segment[start:start + block_keyframe_count]
            if block:
                blocks.append(block)
    return blocks


def _voxel_centroids(points: np.ndarray, leaf: float) -> np.ndarray:
    keys = np.floor(points[:, :3].astype('f8') / leaf).astype('<i8')
    _unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(_unique), 4), dtype='f8')
    counts = np.bincount(inverse, minlength=len(_unique)).astype('f8')
    np.add.at(sums, inverse, points.astype('f8'))
    return (sums / counts[:, None]).astype('<f4')


def _label_for(record: dict[str, Any], frame_labels: dict[int, dict[str, Any]]) -> dict[str, Any]:
    label = frame_labels.get(record['keyframe_id'], {})
    row_id = label.get('physical_row_id', UNKNOWN)
    confirmed = label.get('label_confidence') == 'confirmed' and row_id not in (None, '', UNKNOWN)
    return {
        'row_id': str(row_id) if confirmed else UNKNOWN,
        'along_row_s_m': label.get('along_row_s_m') if confirmed else None,
        'scene_type': label.get('scene_type', UNKNOWN) if confirmed else UNKNOWN,
        'review_state': 'CONFIRMED' if confirmed else 'UNKNOWN',
    }


def _validate_labels(frame_labels: dict[int, dict[str, Any]] | None,
                     annotation_digest: str | None) -> dict[int, dict[str, Any]]:
    labels = frame_labels or {}
    if not isinstance(labels, dict):
        raise ValueError('frame_labels must map keyframe IDs to reviewed labels')
    if annotation_digest is not None and (not isinstance(annotation_digest, str) or
                                          re.fullmatch(r'[0-9a-f]{64}', annotation_digest) is None):
        raise ValueError('row_annotation_sha256 must be a lowercase SHA-256 digest')
    if labels and (not isinstance(annotation_digest, str) or
                   re.fullmatch(r'[0-9a-f]{64}', annotation_digest) is None):
        raise ValueError('reviewed frame labels require the exact annotation SHA-256')
    normalized: dict[int, dict[str, Any]] = {}
    for key, value in labels.items():
        if isinstance(key, bool) or not isinstance(key, int) or key < 0 or not isinstance(value, dict):
            raise ValueError('frame label keys must be nonnegative keyframe IDs with mapping values')
        confidence = value.get('label_confidence', value.get('confidence', 'UNKNOWN'))
        row_id = value.get('physical_row_id', value.get('row_id', UNKNOWN))
        scene_type = value.get('scene_type', value.get('zone_type', UNKNOWN))
        along = value.get('along_row_s_m')
        confirmed = confidence == 'confirmed' and isinstance(row_id, str) and row_id not in ('', UNKNOWN)
        if confirmed:
            if scene_type not in ('ROW', 'ROW_ENTRY', 'ROW_MIDDLE', 'ROW_END', 'HEADLAND'):
                raise ValueError(f'keyframe {key}: unsupported confirmed scene_type')
            if along is not None and (isinstance(along, bool) or not math.isfinite(float(along)) or float(along) < 0):
                raise ValueError(f'keyframe {key}: along_row_s_m must be finite and nonnegative')
        normalized[key] = {
            'physical_row_id': row_id if confirmed else UNKNOWN,
            'along_row_s_m': float(along) if confirmed and along is not None else None,
            'scene_type': scene_type if confirmed else UNKNOWN,
            'label_confidence': 'confirmed' if confirmed else 'UNKNOWN',
        }
    return normalized


def _write_checksums(root: Path) -> None:
    entries = []
    for path in sorted(root.rglob('*')):
        if path.is_file() and path.relative_to(root).as_posix() != 'checksums.sha256':
            entries.append(f'{sha256(path)}  {path.relative_to(root).as_posix()}')
    (root / 'checksums.sha256').write_text('\n'.join(entries) + '\n', encoding='ascii')


def _bundle_path(root: Path, value: Any, *, label: str) -> Path:
    if not isinstance(value, str):
        raise ValueError(f'{label} must be a relative POSIX bundle path')
    rel = PurePosixPath(value)
    if (rel.is_absolute() or '..' in rel.parts or not rel.parts or rel.as_posix() != value or
            '\\' in value or any(character.isspace() for character in value)):
        raise ValueError(f'{label} must be a safe relative POSIX bundle path')
    path = root
    for part in rel.parts:
        path = path / part
        if path.is_symlink():
            raise ValueError(f'{label} must not traverse a symbolic link: {value}')
    return path


def build_keyframe_blocks(source_package: str | Path, output_dir: str | Path, *,
                          block_keyframe_count: int = 11, stride: int | None = None,
                          max_timestamp_gap_s: float = 2.0,
                          voxel_leaf_m: float | None = None,
                          frame_labels: dict[int, dict[str, Any]] | None = None,
                          row_annotation_sha256: str | None = None) -> dict[str, Any]:
    """Build a new immutable-by-revision block set; source package is read-only."""
    source_root = Path(source_package).expanduser().resolve(strict=True)
    destination = require_output_outside(output_dir, [source_root], label='block output')
    if destination.exists():
        raise FileExistsError(f'refusing to overwrite block asset revision: {destination}')
    stride = block_keyframe_count if stride is None else stride
    gap = _finite_number(max_timestamp_gap_s, 'max_timestamp_gap_s', positive=True)
    leaf = None if voxel_leaf_m is None else _finite_number(voxel_leaf_m, 'voxel_leaf_m', positive=True)

    verified = verify_frontend_map_package(source_root)
    if verified.get('status') != 'PASS':
        raise ValueError(f'frontend source package validation failed: {verified}')
    metadata = yaml.safe_load((source_root / 'metadata.yaml').read_text(encoding='utf-8')) or {}
    source_manifest_sha256 = sha256(source_root / 'manifest.yaml')
    source_checksums_sha256 = sha256(source_root / 'checksums.sha256')
    pose_revision = sha256(source_root / 'poses_timed.txt')
    source_backend = metadata.get('mapping_backend', {}).get('id', 'UNKNOWN')
    source_identity = f'frontend_mapping_map_package:{source_backend}:{source_manifest_sha256}'
    map_frame = metadata.get('frames', {}).get('map')
    if not isinstance(map_frame, str) or not map_frame:
        raise ValueError('source metadata must declare frames.map')
    records = _read_pose_records(source_root)
    blocks = partition_pose_records(records, block_keyframe_count=block_keyframe_count,
                                    stride=stride, max_timestamp_gap_s=gap)
    labels = _validate_labels(frame_labels, row_annotation_sha256)
    builder_config = {
        'algorithm': 'paired_body_patch_pose_transform_concat',
        'block_keyframe_count': block_keyframe_count, 'stride': stride,
        'max_timestamp_gap_s': gap, 'endpoint_policy': 'truncate_at_segment_boundary',
        'timestamp_policy': 'strictly_increasing_and_split_on_gap_or_missing_keyframe_id',
        'output_frame': map_frame, 'voxel_leaf_m': leaf,
    }
    config_sha256 = _canonical_sha256(builder_config)
    block_set_identity = {
        'asset_type': BLOCK_ASSET_TYPE,
        'parent_source_id': source_identity,
        'parent_source_digest': source_manifest_sha256,
        'pose_revision': pose_revision,
        'builder_config_digest': config_sha256,
        'row_annotation_sha256': row_annotation_sha256,
        'revision': 1,
    }
    block_set_id = 'blockset-' + _canonical_sha256(block_set_identity)[:24]

    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_root = Path(tempfile.mkdtemp(prefix=f'.{destination.name}.tmp-', dir=destination.parent))
    committed = False
    try:
        blocks_dir = temp_root / 'blocks'
        blocks_dir.mkdir()
        block_rows: list[dict[str, Any]] = []
        keyframe_to_blocks: dict[str, list[str]] = {}
        row_to_blocks: dict[str, list[str]] = {}
        spatial = []
        timestamp_lookup = []

        for ordinal, group in enumerate(blocks):
            block_id = f'block_{ordinal:04d}'
            center = group[(len(group) - 1) // 2]
            chunks = []
            for record in group:
                patch_path = source_root / 'patches' / record['patch_id']
                patch = _read_pcd(patch_path)
                if patch.ndim != 2 or patch.shape[1] != 4 or not len(patch) or not np.isfinite(patch).all():
                    raise ValueError(f'invalid patch geometry: {record["patch_id"]}')
                mapped = np.asarray(patch, dtype='<f4').copy()
                mapped[:, :3] = (patch[:, :3].astype('f8') @ record['rotation'].T +
                                 record['position']).astype('<f4')
                if not np.isfinite(mapped).all():
                    raise ValueError(f'bad transform for keyframe {record["keyframe_id"]}')
                chunks.append(mapped)
            fused = np.concatenate(chunks, axis=0)
            if leaf is not None:
                fused = _voxel_centroids(fused, leaf)
            fused = np.asarray(fused, dtype='<f4')
            if not len(fused) or not np.isfinite(fused).all():
                raise ValueError(f'block {block_id} has no finite points')
            output_rel = f'blocks/{block_id}.pcd'
            _write_pcd(temp_root / output_rel, fused)
            minimum, maximum = fused[:, :3].min(axis=0), fused[:, :3].max(axis=0)
            position_steps = [float(np.linalg.norm(b['position'] - a['position']))
                              for a, b in zip(group, group[1:])]
            path_length = float(sum(position_steps))
            extent = (maximum - minimum).astype('f8')
            block_labels = [_label_for(record, labels) for record in group]
            center_label = _label_for(center, labels)
            common_row_ids = {item['row_id'] for item in block_labels}
            block_row_id = (next(iter(common_row_ids)) if len(common_row_ids) == 1 and
                            UNKNOWN not in common_row_ids else UNKNOWN)
            scene_types = {item['scene_type'] for item in block_labels}
            scene_type = next(iter(scene_types)) if len(scene_types) == 1 else 'MIXED'
            patch_hashes = [sha256(source_root / 'patches' / record['patch_id']) for record in group]
            record = {
                'block_id': block_id, 'revision': 1,
                'parent_source_id': source_identity,
                'parent_source_digest': source_manifest_sha256,
                'pose_revision': pose_revision,
                'row_annotation_sha256': row_annotation_sha256,
                'center_keyframe_id': center['keyframe_id'],
                'ordered_patch_ids': [item['patch_id'] for item in group],
                'ordered_patch_hashes': patch_hashes,
                'first_timestamp': group[0]['timestamp'], 'last_timestamp': group[-1]['timestamp'],
                'output_frame': map_frame,
                'requested_block_keyframe_count': block_keyframe_count,
                'actual_keyframe_count': len(group), 'half_range': (len(group) - 1) // 2,
                'stride': stride, 'endpoint_policy': 'truncate_at_segment_boundary',
                'max_timestamp_gap_s': gap, 'voxel_leaf_m': leaf,
                'bbox': {'min_xyz_m': minimum.astype(float).tolist(),
                         'max_xyz_m': maximum.astype(float).tolist()},
                'actual_trajectory_length_m': path_length,
                'actual_spatial_extent_m': extent.astype(float).tolist(),
                'center_pose_xyz_m': center['position'].astype(float).tolist(),
                'row_id': block_row_id,
                'along_row_s_m': center_label['along_row_s_m'] if block_row_id != UNKNOWN else None,
                'scene_type': scene_type if block_row_id != UNKNOWN else UNKNOWN,
                'review_state': ('CONFIRMED' if block_row_id != UNKNOWN and
                                 all(item['review_state'] == 'CONFIRMED' for item in block_labels)
                                 else 'UNKNOWN'),
                'builder_config_digest': config_sha256,
                'output_path': output_rel,
                'output_point_count': int(len(fused)),
                'output_content_sha256': sha256(temp_root / output_rel),
            }
            block_rows.append(record)
            for item in group:
                keyframe_to_blocks.setdefault(str(item['keyframe_id']), []).append(block_id)
                timestamp_lookup.append({'timestamp': item['timestamp'], 'keyframe_id': item['keyframe_id'],
                                         'block_id': block_id})
            if block_row_id != UNKNOWN:
                row_to_blocks.setdefault(block_row_id, []).append(block_id)
            spatial.append({'block_id': block_id, 'center_keyframe_id': center['keyframe_id'],
                            'bbox': record['bbox'], 'center_pose_xyz_m': record['center_pose_xyz_m'],
                            'row_id': block_row_id})

        index = {
            'schema_version': 1, 'block_set_id': block_set_id, 'revision': 1,
            'parent_source_digest': source_manifest_sha256,
            'keyframe_to_blocks': keyframe_to_blocks,
            'timestamp_lookup': timestamp_lookup,
            'spatial_lookup': spatial,
            'row_id_to_blocks': row_to_blocks,
            'row_annotation_status': 'FROZEN_REVIEWED' if row_annotation_sha256 else UNKNOWN,
        }
        index_path = temp_root / 'block_index.json'
        index_path.write_text(json.dumps(index, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')
        index_sha256 = sha256(index_path)
        manifest = {
            'schema_version': BLOCK_SCHEMA_VERSION, 'asset_type': BLOCK_ASSET_TYPE,
            'block_set_id': block_set_id, 'revision': 1,
            'parent_source_id': source_identity,
            'parent_source_digest': source_manifest_sha256,
            'parent_source_manifest_sha256': source_manifest_sha256,
            'pose_revision': pose_revision,
            'row_annotation_sha256': row_annotation_sha256,
            'map_frame': map_frame,
            'builder_config': builder_config,
            'builder_config_digest': config_sha256,
            'index_path': 'block_index.json', 'index_sha256': index_sha256,
            'block_count': len(block_rows), 'blocks': block_rows,
            'hash_semantics': {
                'block_content': 'sha256 of exact output PCD bytes',
                'parent_source_digest': 'sha256 of exact parent manifest.yaml bytes',
                'pose_revision': 'sha256 of exact parent poses_timed.txt bytes',
                'manifest': 'excluded from self; included by external checksums.sha256',
            },
        }
        (temp_root / 'manifest.yaml').write_text(
            yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding='utf-8')
        _write_checksums(temp_root)
        final_source_check = verify_frontend_map_package(source_root)
        if (final_source_check.get('status') != 'PASS' or
                sha256(source_root / 'manifest.yaml') != source_manifest_sha256 or
                sha256(source_root / 'checksums.sha256') != source_checksums_sha256 or
                sha256(source_root / 'poses_timed.txt') != pose_revision):
            raise ValueError('source package changed while building block assets; refusing to publish')
        for block in block_rows:
            for patch_id, patch_digest in zip(block['ordered_patch_ids'], block['ordered_patch_hashes']):
                if sha256(source_root / 'patches' / patch_id) != patch_digest:
                    raise ValueError(f'source patch changed while building block assets: {patch_id}')
        # Same-filesystem directory rename is the atomic no-overwrite commit.
        # The complete revision appears at once or not at all.
        os.rename(temp_root, destination)
        committed = True
        return manifest
    finally:
        if not committed and temp_root.exists():
            shutil.rmtree(temp_root)


def verify_keyframe_blocks(block_dir: str | Path, *, source_package: str | Path | None = None) -> dict[str, Any]:
    requested_root = Path(block_dir).expanduser()
    if requested_root.is_symlink():
        raise ValueError(f'unsafe block directory: {requested_root}')
    root = requested_root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f'unsafe block directory: {root}')
    checksum_path = root / 'checksums.sha256'
    if not checksum_path.is_file() or checksum_path.is_symlink():
        raise ValueError('block set has no safe checksums.sha256')
    entries: dict[str, str] = {}
    for line in checksum_path.read_text(encoding='ascii').splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  (.+)', line)
        if not match:
            raise ValueError(f'malformed block checksum entry: {line[:100]!r}')
        digest, rel_text = match.groups()
        rel = PurePosixPath(rel_text)
        if rel.is_absolute() or '..' in rel.parts or rel.as_posix() != rel_text or rel_text in entries:
            raise ValueError(f'unsafe/duplicate block checksum path: {rel_text}')
        path = _bundle_path(root, rel_text, label='checksum path')
        if not path.is_file() or sha256(path) != digest:
            raise ValueError(f'block asset checksum mismatch: {rel_text}')
        entries[rel_text] = digest
    actual = {path.relative_to(root).as_posix() for path in root.rglob('*')
              if path.is_file() and path != checksum_path}
    if actual != set(entries):
        raise ValueError(f'block checksum coverage mismatch: unlisted={sorted(actual-set(entries))}, absent={sorted(set(entries)-actual)}')
    manifest_path, index_path = root / 'manifest.yaml', root / 'block_index.json'
    if not manifest_path.is_file() or not index_path.is_file():
        raise ValueError('block manifest or index missing')
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    index = json.loads(index_path.read_text(encoding='utf-8'))
    if (not isinstance(manifest, dict) or manifest.get('schema_version') != BLOCK_SCHEMA_VERSION or
            manifest.get('asset_type') != BLOCK_ASSET_TYPE):
        raise ValueError('unsupported keyframe block manifest')
    if manifest.get('index_sha256') != sha256(index_path):
        raise ValueError('block index digest mismatch')
    if (not isinstance(index, dict) or index.get('block_set_id') != manifest.get('block_set_id') or
            index.get('parent_source_digest') != manifest.get('parent_source_digest')):
        raise ValueError('block index identity does not match manifest')
    block_ids: set[str] = set()
    for block in manifest.get('blocks', []):
        if not isinstance(block, dict) or block.get('block_id') in block_ids:
            raise ValueError('invalid/duplicate block record')
        block_ids.add(block['block_id'])
        count = block.get('actual_keyframe_count')
        if (isinstance(count, bool) or not isinstance(count, int) or count < 1 or
                len(block.get('ordered_patch_ids', [])) != count or
                len(block.get('ordered_patch_hashes', [])) != count):
            raise ValueError(f'{block.get("block_id")}: keyframe/patch coverage mismatch')
        if any(re.fullmatch(r'[0-9a-f]{64}', digest or '') is None
               for digest in block.get('ordered_patch_hashes', [])):
            raise ValueError(f'{block.get("block_id")}: malformed source patch hash')
        output_value = block.get('output_path')
        if output_value not in entries:
            raise ValueError(f'{block.get("block_id")}: output path is not covered by checksums')
        path = _bundle_path(root, output_value, label=f'{block.get("block_id")} output path')
        if not path.is_file() or sha256(path) != block.get('output_content_sha256'):
            raise ValueError(f'{block.get("block_id")}: content hash mismatch')
    if source_package is not None:
        source_root = Path(source_package).expanduser().resolve(strict=True)
        verified = verify_frontend_map_package(source_root)
        parent_digest = sha256(source_root / 'manifest.yaml')
        pose_digest = sha256(source_root / 'poses_timed.txt')
        if verified.get('status') != 'PASS' or parent_digest != manifest.get('parent_source_digest') or pose_digest != manifest.get('pose_revision'):
            raise ValueError('block set is stale against current source package')
        for block in manifest.get('blocks', []):
            for patch_id, patch_digest in zip(block.get('ordered_patch_ids', []),
                                              block.get('ordered_patch_hashes', [])):
                patch_path = source_root / 'patches' / patch_id
                if patch_path.is_symlink() or not patch_path.is_file() or sha256(patch_path) != patch_digest:
                    raise ValueError(f'block set source patch identity mismatch: {patch_id}')
    return {'status': 'PASS', 'block_set_id': manifest['block_set_id'],
            'revision': manifest['revision'], 'block_count': len(manifest.get('blocks', [])),
            'parent_source_digest': manifest['parent_source_digest'],
            'manifest_sha256': sha256(manifest_path), 'index_sha256': sha256(index_path),
            'checksums_sha256': sha256(checksum_path)}


def blocks_near_position(block_dir: str | Path, xyz: Iterable[float], *,
                         radius_m: float = 0.0) -> list[dict[str, Any]]:
    root = Path(block_dir)
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
    index = json.loads((root / manifest['index_path']).read_text(encoding='utf-8'))
    point = np.asarray(list(xyz), dtype='f8')
    radius = _finite_number(radius_m, 'radius_m')
    if point.shape != (3,) or not np.isfinite(point).all() or radius < 0:
        raise ValueError('position must be finite XYZ and radius_m nonnegative')
    matches = []
    by_id = {b['block_id']: b for b in manifest['blocks']}
    for item in index['spatial_lookup']:
        lower = np.asarray(item['bbox']['min_xyz_m'], dtype='f8') - radius
        upper = np.asarray(item['bbox']['max_xyz_m'], dtype='f8') + radius
        nearest = np.minimum(np.maximum(point, lower), upper)
        distance = float(np.linalg.norm(point - nearest))
        if distance <= 1e-9:
            block = by_id[item['block_id']]
            matches.append({'block_id': item['block_id'], 'distance_to_bbox_m': distance,
                            'center_keyframe_id': block['center_keyframe_id'],
                            'row_id': block.get('row_id', UNKNOWN)})
    return matches
