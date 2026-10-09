"""Query Set, batch study, summary and project persistence services.

This module orchestrates the existing single-query GLOBAL/Top-K/GICP operation;
it contains no registration implementation. Query Sets and Studies are authored
assets with explicit source identities and immutable frozen revisions.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import shutil
import sys
import tempfile
import time
from datetime import datetime, timezone
from typing import Any
import uuid

import numpy as np
import yaml
from shapely.geometry import LineString

from agt_mapping_artifacts.frontend_package import sha256
from agt_mapping_artifacts.keyframe_blocks import verify_keyframe_blocks
from agt_mapping_artifacts.relocalization_evidence import verify_evidence_bundle

from .greenhouse import MapPackage
from .relocalization_mvp import (  # noqa: E402
    _load_topology, canonical_digest, current_algorithm_state, run_query,
)


QUERY_SET_TYPE = 'agt.relocalization_query_set/v1'
STUDY_TYPE = 'agt.relocalization_study/v1'
PROJECT_TYPE = 'agt.mapstudio_project/v1'
VALID_SCENES = {'ENTRY', 'MIDDLE', 'EXIT', 'HEADLAND', 'OTHER'}
VALID_REFERENCE_LEVELS = {'SAME_SESSION_REFERENCE', 'INDEPENDENT_GROUND_TRUTH', 'UNKNOWN'}
VALID_RESULTS = {'CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT', 'NO_DATA',
                 'REFERENCE_UNKNOWN', 'NOT_RUN'}
ALGORITHM_NAME = 'existing native GLOBAL + Top-K + GICP'


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_yaml_atomic(path: Path, value: dict[str, Any], *, overwrite: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not overwrite and path.exists():
        raise FileExistsError(f'refusing to overwrite immutable asset: {path}')
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(value, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        if not overwrite and path.exists():
            raise FileExistsError(f'refusing to overwrite immutable asset: {path}')
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{path}: expected a YAML mapping')
    return value


def _asset_digest(asset: dict[str, Any]) -> str:
    return canonical_digest({key: value for key, value in asset.items() if key != 'content_sha256'})


def _relative_ref(path: Path, anchor: Path) -> dict[str, str]:
    """Use a relative runtime hint while identity remains the digest contract."""
    return {'kind': 'filesystem', 'path': os.path.relpath(path.resolve(), anchor.resolve())}


def _resolve_ref(reference: dict[str, Any], anchor: Path) -> Path:
    if not isinstance(reference, dict) or reference.get('kind') != 'filesystem':
        raise ValueError('unsupported asset reference kind')
    value = reference.get('path')
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise ValueError('asset references must use a nonempty relative path hint')
    return (anchor / value).resolve(strict=True)


def _rebase_ref(reference: dict[str, Any] | None, old_anchor: Path, new_anchor: Path) -> dict[str, Any] | None:
    if reference is None:
        return None
    absolute = _resolve_ref(reference, old_anchor)
    return {**reference, **_relative_ref(absolute, new_anchor)}


def _source_ref(source: Path, anchor: Path) -> dict[str, Any]:
    source = source.resolve(strict=True)
    from agt_mapping_artifacts.frontend_package import verify_frontend_map_package
    checked = verify_frontend_map_package(source)
    if checked.get('status') != 'PASS':
        raise ValueError(f'source package is not valid: {checked}')
    manifest = source / 'manifest.yaml'
    metadata = _load_yaml(source / 'metadata.yaml') if (source / 'metadata.yaml').is_file() else {}
    return {
        **_relative_ref(source, anchor), 'identity': f"frontend_mapping_map_package:{metadata.get('mapping_backend', {}).get('id', 'UNKNOWN')}",
        'manifest_sha256': sha256(manifest),
        'checksums_sha256': sha256(source / 'checksums.sha256'),
        'pose_revision': sha256(source / 'poses_timed.txt'),
        'backend_id': metadata.get('mapping_backend', {}).get('id', 'UNKNOWN'),
        'reference_level': ('SAME_SESSION_REFERENCE'
                            if metadata.get('reference', {}).get('same_session') is True
                            else 'UNKNOWN'),
    }


def _block_ref(block_dir: Path, source: Path, anchor: Path) -> dict[str, Any]:
    block_dir = block_dir.resolve(strict=True)
    check = verify_keyframe_blocks(block_dir, source_package=source)
    if check.get('status') != 'PASS':
        raise ValueError(f'block set validation failed: {check}')
    manifest = _load_yaml(block_dir / 'manifest.yaml')
    return {**_relative_ref(block_dir, anchor),
            'block_set_id': manifest['block_set_id'], 'revision': manifest['revision'],
            'manifest_sha256': sha256(block_dir / 'manifest.yaml'),
            'index_sha256': sha256(block_dir / manifest['index_path']),
            'checksums_sha256': sha256(block_dir / 'checksums.sha256'),
            'parent_source_digest': manifest['parent_source_digest'],
            'row_annotation_sha256': manifest.get('row_annotation_sha256')}


def _annotation_ref(topology_path: Path | None, source: Path, anchor: Path,
                    *, require_frozen: bool = False) -> dict[str, Any] | None:
    if topology_path is None:
        if require_frozen:
            raise ValueError('this operation requires a manually reviewed frozen annotation')
        return None
    topology_path = topology_path.resolve(strict=True)
    topology, _ = _load_topology(topology_path, source)
    annotation = topology.get('annotation', {})
    if require_frozen and (annotation.get('status') != 'frozen' or
                           annotation.get('manual_review_confirmed') is not True):
        raise ValueError('physical row sampling requires annotation.status=frozen and manual_review_confirmed=true')
    return {**_relative_ref(topology_path, anchor),
            'revision': int(annotation.get('revision', 1)),
            'sha256': sha256(topology_path),
            'status': str(annotation.get('status', 'draft')),
            'manual_review_confirmed': annotation.get('manual_review_confirmed') is True}


def _nearest_legal_keyframe(dataset: MapPackage, xy: tuple[float, float], *, max_frames: int = 5) -> tuple[int, float]:
    if max_frames not in (1, 3, 5):
        raise ValueError('max query window must be 1, 3 or 5 frames')
    margin = max_frames // 2
    candidates = dataset.poses[margin:len(dataset.poses) - margin]
    if not candidates:
        raise ValueError('source has no keyframe with a complete query window')
    click = np.asarray(xy, dtype='f8')
    distances = [float(np.linalg.norm(pose.t[:2] - click)) for pose in candidates]
    best = int(np.argmin(distances))
    return candidates[best].index, distances[best]


def _make_query(dataset: MapPackage, *, query_id: str, scene: str,
                xy: tuple[float, float], row_id: str = 'UNKNOWN',
                along_row_s_m: float | None = None, sample_ratio: float | None = None,
                max_frames: int = 5) -> dict[str, Any]:
    if scene not in VALID_SCENES:
        raise ValueError(f'unsupported scene: {scene}')
    keyframe_index, distance = _nearest_legal_keyframe(dataset, xy, max_frames=max_frames)
    pose = dataset.poses[keyframe_index]
    patch_id = Path(pose.patch).stem
    if not patch_id.isdigit():
        raise ValueError(f'keyframe patch has no numeric identity: {pose.patch}')
    return {
        'query_id': query_id, 'enabled': True, 'scene': scene,
        'row_id': row_id, 'along_row_s_m': along_row_s_m,
        'sample_ratio': sample_ratio,
        'requested_location': {'frame': 'map', 'x_m': float(xy[0]), 'y_m': float(xy[1])},
        'resolved_keyframe_id': int(patch_id), 'resolved_pose_index': keyframe_index,
        'resolved_timestamp': float(pose.stamp), 'snap_distance_m': distance,
        'resolved_position_m': [float(value) for value in pose.t],
        'reference_level': ('SAME_SESSION_REFERENCE'
                            if dataset.reference.get('same_session') is True else 'UNKNOWN'),
        'source_patch_id': pose.patch,
    }


def _base_query_set(query_set_id: str, source: dict[str, Any], block_set: dict[str, Any],
                    annotation: dict[str, Any] | None, strategy: dict[str, Any],
                    queries: list[dict[str, Any]], *, anchor: Path) -> dict[str, Any]:
    asset = {
        'schema_version': 1, 'asset_type': QUERY_SET_TYPE,
        'query_set_id': query_set_id, 'revision': 1, 'status': 'DRAFT',
        'created_utc': _now(), 'source_ref': source, 'block_set_ref': block_set,
        'annotation_ref': annotation, 'sampling_strategy': strategy,
        'query_count': len(queries), 'queries': queries,
        'revision_semantics': 'content-addressed; frozen revisions are immutable',
        'anchor_hint': '.',
    }
    asset['content_sha256'] = _asset_digest(asset)
    return asset


def create_manual_query_set(*, query_set_id: str, map_package: Path, block_dir: Path,
                            locations: list[dict[str, Any]], output: Path,
                            topology_path: Path | None = None) -> dict[str, Any]:
    anchor = output.parent.resolve()
    source_path = map_package.resolve(strict=True)
    dataset = MapPackage(source_path)
    source = _source_ref(source_path, anchor)
    block = _block_ref(block_dir, source_path, anchor)
    annotation = _annotation_ref(topology_path, source_path, anchor)
    if block.get('row_annotation_sha256') is not None and (
            annotation is None or annotation.get('sha256') != block['row_annotation_sha256']):
        raise ValueError('block set requires the exact row annotation revision used when it was built')
    queries = []
    for item in locations:
        query_id = str(item.get('query_id', '')).strip()
        if not query_id:
            query_id = f'Q{len(queries) + 1:03d}'
        xy = item.get('xy')
        if not isinstance(xy, (list, tuple)) or len(xy) != 2:
            raise ValueError(f'{query_id}: xy must contain map-frame x and y')
        queries.append(_make_query(dataset, query_id=query_id,
                                   scene=str(item.get('scene', 'OTHER')),
                                   xy=(float(xy[0]), float(xy[1])),
                                   max_frames=5))
    if not queries:
        raise ValueError('manual Query Set needs at least one selected map point')
    _unique_queries(queries)
    asset = _base_query_set(query_set_id, source, block, annotation,
                            {'kind': 'MANUAL_MAP_POINT', 'snap_rule': 'nearest legal trajectory/keyframe observation',
                             'max_query_accumulation_frames': 5}, queries, anchor=anchor)
    _write_yaml_atomic(output, asset, overwrite=False)
    return asset


def create_structure_query_set(*, query_set_id: str, map_package: Path, block_dir: Path,
                               topology_path: Path, output: Path,
                               ratios: tuple[float, ...] = (0.0, 0.25, 0.5, 0.75, 1.0)) -> dict[str, Any]:
    anchor = output.parent.resolve()
    source_path = map_package.resolve(strict=True)
    dataset = MapPackage(source_path)
    source = _source_ref(source_path, anchor)
    block = _block_ref(block_dir, source_path, anchor)
    annotation = _annotation_ref(topology_path, source_path, anchor, require_frozen=True)
    if block.get('row_annotation_sha256') != annotation.get('sha256'):
        raise ValueError('block set and frozen annotation do not share the exact annotation digest')
    topology, _ = _load_topology(topology_path, source_path)
    queries = []
    for row in topology.get('rows', []):
        if row.get('confidence') != 'confirmed':
            continue
        line = LineString(row['centerline'])
        for ratio in ratios:
            if not math.isfinite(ratio) or not 0 <= ratio <= 1:
                raise ValueError('sampling ratios must be finite and in [0, 1]')
            point = line.interpolate(ratio, normalized=True)
            scene = 'ENTRY' if ratio == 0 else ('EXIT' if ratio == 1 else 'MIDDLE')
            suffix = {0.0: 'entry', 0.25: 'q25', 0.5: 'middle', 0.75: 'q75', 1.0: 'exit'}.get(ratio, f'r{ratio:.3f}')
            queries.append(_make_query(
                dataset, query_id=f"{row['id']}_{suffix}", scene=scene,
                xy=(point.x, point.y), row_id=row['id'],
                along_row_s_m=float(line.length * ratio), sample_ratio=float(ratio), max_frames=5))
    if not queries:
        raise ValueError('frozen annotation has no manually confirmed rows')
    _unique_queries(queries)
    asset = _base_query_set(query_set_id, source, block, annotation,
                            {'kind': 'REVIEWED_ROW_SAMPLING', 'ratios': list(ratios),
                             'scene_mapping': 'ENTRY/MIDDLE/EXIT',
                             'max_query_accumulation_frames': 5}, queries, anchor=anchor)
    _write_yaml_atomic(output, asset, overwrite=False)
    return asset


def _unique_queries(queries: list[dict[str, Any]]) -> None:
    ids = [query.get('query_id') for query in queries]
    if (any(not isinstance(value, str) or not value or
            any(character not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for character in value)
            for value in ids) or len(ids) != len(set(ids))):
        raise ValueError('query_id values must be unique nonempty strings')
    for query in queries:
        if query['scene'] not in VALID_SCENES:
            raise ValueError(f"{query['query_id']}: unsupported scene")
        keyframe = query.get('resolved_keyframe_id')
        if not isinstance(keyframe, int) or isinstance(keyframe, bool) or keyframe < 0:
            raise ValueError(f"{query['query_id']}: invalid resolved_keyframe_id")
        if query.get('reference_level') not in VALID_REFERENCE_LEVELS:
            raise ValueError(f"{query['query_id']}: invalid reference_level")
        location = query.get('requested_location')
        if not isinstance(location, dict) or location.get('frame') != 'map':
            raise ValueError(f"{query['query_id']}: requested_location must be in map frame")
        if not all(math.isfinite(float(location.get(axis, float('nan')))) for axis in ('x_m', 'y_m')):
            raise ValueError(f"{query['query_id']}: requested map point must be finite")
        if not math.isfinite(float(query.get('snap_distance_m', float('nan')))) or query['snap_distance_m'] < 0:
            raise ValueError(f"{query['query_id']}: snap distance must be finite and nonnegative")


def validate_query_set(path: Path, *, verify_dependencies: bool = True) -> dict[str, Any]:
    asset = _load_yaml(path)
    errors: list[str] = []
    if asset.get('schema_version') != 1 or asset.get('asset_type') != QUERY_SET_TYPE:
        errors.append('unsupported Query Set contract')
    if asset.get('status') not in ('DRAFT', 'FROZEN'):
        errors.append('status must be DRAFT or FROZEN')
    if not isinstance(asset.get('query_set_id'), str) or not asset['query_set_id']:
        errors.append('query_set_id is required')
    if not isinstance(asset.get('revision'), int) or asset.get('revision', 0) < 1:
        errors.append('revision must be a positive integer')
    try:
        _unique_queries(asset.get('queries', []))
    except (ValueError, TypeError, KeyError) as exc:
        errors.append(str(exc))
    if asset.get('query_count') != len(asset.get('queries', [])):
        errors.append('query_count does not match queries')
    expected_digest = _asset_digest(asset)
    if asset.get('content_sha256') != expected_digest:
        errors.append('content_sha256 mismatch')
    anchor = path.parent.resolve()
    if verify_dependencies and not errors:
        try:
            source_path = _resolve_ref(asset['source_ref'], anchor)
            block_path = _resolve_ref(asset['block_set_ref'], anchor)
            dataset = MapPackage(source_path)
            current_source = _source_ref(source_path, anchor)
            current_block = _block_ref(block_path, source_path, anchor)
            for label, saved, current in (('source', asset['source_ref'], current_source),
                                          ('block set', asset['block_set_ref'], current_block)):
                for field in ('manifest_sha256', 'checksums_sha256', 'pose_revision',
                              'index_sha256', 'parent_source_digest'):
                    if field in saved and saved.get(field) != current.get(field):
                        errors.append(f'{label} is STALE: {field} changed')
            block_manifest = _load_yaml(block_path / 'manifest.yaml')
            annotation_path = None
            if asset.get('annotation_ref'):
                annotation_path = _resolve_ref(asset['annotation_ref'], anchor)
                if sha256(annotation_path) != asset['annotation_ref'].get('sha256'):
                    errors.append('annotation is STALE: revision changed')
                ann, labels = _load_topology(annotation_path, source_path)
                if block_manifest.get('row_annotation_sha256') not in (None, sha256(annotation_path)):
                    errors.append('block set is bound to a different annotation revision')
                if asset.get('status') == 'FROZEN' and (
                        any(query.get('row_id', 'UNKNOWN') != 'UNKNOWN' for query in asset.get('queries', [])) and
                        (ann.get('annotation', {}).get('status') != 'frozen' or
                         ann.get('annotation', {}).get('manual_review_confirmed') is not True)):
                    errors.append('physical row identities in a frozen Query Set are no longer manually reviewed')
            elif block_manifest.get('row_annotation_sha256') is not None:
                errors.append('Query Set is missing the annotation required by its block set')
            pose_by_id = {int(Path(pose.patch).stem): pose for pose in dataset.poses
                          if Path(pose.patch).stem.isdigit()}
            annotation_rows = {row.get('id'): row for row in (ann.get('rows', []) if asset.get('annotation_ref') else [])}
            for query in asset.get('queries', []):
                identifier = query.get('query_id', 'UNKNOWN')
                pose = pose_by_id.get(query.get('resolved_keyframe_id'))
                if pose is None:
                    errors.append(f'{identifier}: resolved keyframe is absent from source')
                    continue
                if abs(float(query.get('resolved_timestamp', float('inf'))) - pose.stamp) > 1e-6:
                    errors.append(f'{identifier}: resolved timestamp disagrees with source observation')
                recorded_position = query.get('resolved_position_m')
                if (not isinstance(recorded_position, list) or len(recorded_position) != 3 or
                        not np.allclose(np.asarray(recorded_position, dtype='f8'), pose.t, atol=1e-6)):
                    errors.append(f'{identifier}: resolved position disagrees with source observation')
                location = query['requested_location']
                nearest, distance = _nearest_legal_keyframe(dataset, (float(location['x_m']), float(location['y_m'])))
                if nearest != pose.index or abs(distance - float(query['snap_distance_m'])) > 1e-5:
                    errors.append(f'{identifier}: keyframe snap is inconsistent with the source trajectory')
                row_id = query.get('row_id', 'UNKNOWN')
                if row_id != 'UNKNOWN':
                    row = annotation_rows.get(row_id)
                    if (annotation_path is None or ann.get('annotation', {}).get('status') != 'frozen' or
                            ann.get('annotation', {}).get('manual_review_confirmed') is not True or
                            row is None or row.get('confidence') != 'confirmed'):
                        errors.append(f'{identifier}: physical row identity requires a frozen manually reviewed row')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f'dependency validation failed: {exc}')
    state = ('STALE' if any('STALE' in error for error in errors) else
             ('INVALID' if errors else 'CURRENT'))
    return {'valid': not errors, 'state': state,
            'errors': errors, 'query_set_id': asset.get('query_set_id'),
            'revision': asset.get('revision'), 'status': asset.get('status'),
            'query_count': len(asset.get('queries', []))}


def freeze_query_set(draft_path: Path, frozen_path: Path) -> dict[str, Any]:
    check = validate_query_set(draft_path)
    if not check['valid']:
        raise ValueError(f'cannot freeze invalid Query Set: {check["errors"]}')
    draft = _load_yaml(draft_path)
    if draft.get('status') == 'FROZEN':
        raise ValueError('input Query Set is already frozen; create a new draft revision')
    if not any(query.get('enabled', True) for query in draft.get('queries', [])):
        raise ValueError('cannot freeze a Query Set with no enabled queries')
    frozen = dict(draft)
    frozen['status'] = 'FROZEN'
    frozen['revision'] = int(draft['revision']) + 1
    frozen['parent_content_sha256'] = draft['content_sha256']
    frozen['frozen_utc'] = _now()
    for field in ('source_ref', 'block_set_ref', 'annotation_ref'):
        frozen[field] = _rebase_ref(frozen.get(field), draft_path.parent.resolve(), frozen_path.parent.resolve())
    frozen.pop('content_sha256', None)
    frozen['content_sha256'] = _asset_digest(frozen)
    _write_yaml_atomic(frozen_path, frozen, overwrite=False)
    return frozen


def clone_query_set_as_draft(query_set_path: Path, output_path: Path) -> dict[str, Any]:
    source = _load_yaml(query_set_path)
    check = validate_query_set(query_set_path)
    if not check['valid'] or source.get('status') != 'FROZEN':
        raise ValueError(f'only a valid frozen Query Set can seed a new draft: {check}')
    draft = dict(source)
    draft['status'] = 'DRAFT'
    draft['revision'] = int(source['revision']) + 1
    draft['derived_from_sha256'] = source['content_sha256']
    draft.pop('frozen_utc', None)
    for field in ('source_ref', 'block_set_ref', 'annotation_ref'):
        draft[field] = _rebase_ref(draft.get(field), query_set_path.resolve().parent,
                                   output_path.parent.resolve())
    draft.pop('content_sha256', None)
    draft['content_sha256'] = _asset_digest(draft)
    _write_yaml_atomic(output_path, draft, overwrite=False)
    return draft


def add_manual_query(query_set_path: Path, *, xy: tuple[float, float], scene: str = 'OTHER',
                     query_id: str | None = None) -> dict[str, Any]:
    """Append a manual map selection to a draft only; frozen revisions are immutable."""
    asset = _load_yaml(query_set_path)
    if asset.get('status') != 'DRAFT':
        raise ValueError('frozen Query Set is immutable; clone it to a new draft revision')
    source = _resolve_ref(asset['source_ref'], query_set_path.resolve().parent)
    dataset = MapPackage(source)
    identifier = query_id or f'Q{len(asset.get("queries", [])) + 1:03d}'
    record = _make_query(dataset, query_id=identifier, scene=scene, xy=xy)
    _unique_queries([*asset.get('queries', []), record])
    asset.setdefault('queries', []).append(record)
    asset['query_count'] = len(asset['queries'])
    _refresh_digest(asset)
    _write_yaml_atomic(query_set_path, asset, overwrite=True)
    return record


def set_query_enabled(query_set_path: Path, query_id: str, enabled: bool) -> None:
    asset = _load_yaml(query_set_path)
    if asset.get('status') != 'DRAFT':
        raise ValueError('frozen Query Set is immutable; clone it to a new draft revision')
    query = next((item for item in asset.get('queries', []) if item.get('query_id') == query_id), None)
    if query is None:
        raise KeyError(f'query not found: {query_id}')
    query['enabled'] = bool(enabled)
    _refresh_digest(asset)
    _write_yaml_atomic(query_set_path, asset, overwrite=True)


def delete_draft_query(query_set_path: Path, query_id: str) -> None:
    asset = _load_yaml(query_set_path)
    if asset.get('status') != 'DRAFT':
        raise ValueError('frozen Query Set is immutable; clone it to a new draft revision')
    prior = len(asset.get('queries', []))
    asset['queries'] = [item for item in asset.get('queries', []) if item.get('query_id') != query_id]
    if len(asset['queries']) == prior:
        raise KeyError(f'query not found: {query_id}')
    asset['query_count'] = len(asset['queries'])
    _refresh_digest(asset)
    _write_yaml_atomic(query_set_path, asset, overwrite=True)


def create_study(*, study_id: str, query_set_path: Path, output_dir: Path,
                 frames: tuple[int, ...], candidate_top_k: int = 5,
                 time_budget_s: float = 18.0) -> dict[str, Any]:
    checked = validate_query_set(query_set_path)
    if not checked['valid'] or checked['status'] != 'FROZEN':
        raise ValueError(f'study requires a current frozen Query Set: {checked}')
    query_set = _load_yaml(query_set_path)
    if not frames or len(set(frames)) != len(frames) or any(frame not in (1, 3, 5) for frame in frames):
        raise ValueError('frames must be a unique subset of 1,3,5')
    if not 1 <= candidate_top_k <= 5:
        raise ValueError('candidate_top_k is fixed to the small supported range 1..5')
    if not math.isfinite(time_budget_s) or time_budget_s <= 0:
        raise ValueError('time_budget_s must be finite and positive')
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    query_ref = _relative_ref(query_set_path.resolve(), output_dir)
    algorithm = {'name': ALGORITHM_NAME,
                 'implementation': 'agt_run_relocalization_query existing native adapter',
                 'native_algorithm_changes': False,
                 'time_budget_s': float(time_budget_s),
                 'global_profile': 'existing_native_global_topk_gicp',
                 'candidate_top_k': candidate_top_k,
                 'query_accumulation_frames': sorted(frames)}
    algorithm['config_digest'] = canonical_digest({key: value for key, value in algorithm.items()
                                                    if key != 'config_digest'})
    study = {
        'schema_version': 1, 'asset_type': STUDY_TYPE,
        'study_id': study_id, 'revision': 1, 'status': 'QUEUED',
        'created_utc': _now(),
        'source_ref': _rebase_ref(query_set['source_ref'], query_set_path.resolve().parent, output_dir),
        'annotation_ref': _rebase_ref(query_set.get('annotation_ref'), query_set_path.resolve().parent, output_dir),
        'block_set_ref': _rebase_ref(query_set['block_set_ref'], query_set_path.resolve().parent, output_dir),
        'query_set_ref': {**query_ref, 'content_sha256': query_set['content_sha256'],
                          'query_set_id': query_set['query_set_id'],
                          'revision': query_set['revision']},
        'query_accumulation_frames': sorted(frames), 'candidate_top_k': candidate_top_k,
        'algorithm': algorithm,
        'reference_level': ('SAME_SESSION_REFERENCE'
                            if all(q['reference_level'] == 'SAME_SESSION_REFERENCE'
                                   for q in query_set['queries'] if q.get('enabled', True))
                            else 'UNKNOWN'),
        'job': {'cancel_requested': False, 'completed': 0,
                'total': sum(bool(q.get('enabled', True)) for q in query_set['queries']) * len(frames),
                'failed': 0, 'current_query': None, 'started_utc': None, 'finished_utc': None},
        'results': [],
    }
    study['content_sha256'] = _asset_digest(study)
    _write_yaml_atomic(output_dir / 'study.yaml', study, overwrite=False)
    return study


def _validate_study_schema(study: dict[str, Any]) -> list[str]:
    errors = []
    if study.get('schema_version') != 1 or study.get('asset_type') != STUDY_TYPE:
        errors.append('unsupported Relocalization Study contract')
    if study.get('status') not in {'QUEUED', 'RUNNING', 'SUCCEEDED', 'PARTIAL', 'FAILED', 'CANCELED', 'STALE'}:
        errors.append('unsupported study job state')
    frames = study.get('query_accumulation_frames')
    if not isinstance(frames, list) or not frames or any(frame not in (1, 3, 5) for frame in frames):
        errors.append('query_accumulation_frames must be a nonempty subset of 1,3,5')
    if study.get('candidate_top_k') not in (1, 2, 3, 4, 5):
        errors.append('candidate_top_k must be between 1 and 5')
    if study.get('reference_level') not in VALID_REFERENCE_LEVELS:
        errors.append('invalid reference_level')
    algorithm = study.get('algorithm')
    if not isinstance(algorithm, dict) or not isinstance(algorithm.get('config_digest'), str):
        errors.append('algorithm config digest is required')
    elif algorithm.get('config_digest') != canonical_digest({key: value for key, value in algorithm.items()
                                                             if key != 'config_digest'}):
        errors.append('algorithm config digest mismatch')
    if not isinstance(study.get('results'), list):
        errors.append('results must be a list')
    elif any(item.get('classification') not in VALID_RESULTS for item in study['results']):
        errors.append('result has unsupported classification')
    if study.get('content_sha256') != _asset_digest(study):
        errors.append('content_sha256 mismatch')
    return errors


def _study_summary(study: dict[str, Any]) -> dict[str, Any]:
    results = study.get('results', [])
    totals = {name: sum(item.get('classification') == name for item in results)
              for name in ('CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT', 'NO_DATA', 'REFERENCE_UNKNOWN', 'NOT_RUN')}
    attempted = totals['CORRECT'] + totals['FALSE_ACCEPT'] + totals['REJECTED'] + totals['TIMEOUT']
    decided = totals['CORRECT'] + totals['FALSE_ACCEPT']
    summary: dict[str, Any] = {
        'total_queries': int(study.get('job', {}).get('total', 0)),
        'completed': len(results), **totals,
        'false_accept_rate': (totals['FALSE_ACCEPT'] / decided) if decided else None,
        'false_accept_rate_denominator': decided,
        'reject_rate': (totals['REJECTED'] / attempted) if attempted else None,
        'reject_rate_denominator': attempted,
        'rate_semantics': 'empirical counts; not a probability model',
        'reference_level': study.get('reference_level', 'UNKNOWN'),
        'correct_at_k': {}, 'by_scene': {}, 'by_frame_count': {}, 'alias_type': {'N/A': len(results)},
    }
    for k in (1, 3, 5):
        eligible = [item for item in results if item.get('reference_level') in
                    ('SAME_SESSION_REFERENCE', 'INDEPENDENT_GROUND_TRUTH')]
        if not eligible:
            summary['correct_at_k'][str(k)] = {'value': None, 'label': 'N/A', 'denominator': 0}
            continue
        correct = 0
        known = 0
        for item in eligible:
            rank = item.get('selected_candidate_rank')
            classification = item.get('classification')
            if classification in ('CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT') and rank is not None:
                known += 1
                if classification == 'CORRECT' and int(rank) <= k:
                    correct += 1
        summary['correct_at_k'][str(k)] = ({'value': correct / known, 'count': correct, 'denominator': known}
                                           if known else {'value': None, 'label': 'N/A', 'denominator': 0})
    for key, label in (('scene', 'by_scene'), ('query_accumulation_frames', 'by_frame_count')):
        buckets: dict[str, dict[str, int]] = {}
        for item in results:
            bucket = str(item.get(key, 'UNKNOWN'))
            buckets.setdefault(bucket, {name: 0 for name in ('CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT', 'NO_DATA', 'REFERENCE_UNKNOWN', 'NOT_RUN')})
            classification = item.get('classification', 'NO_DATA')
            if classification in buckets[bucket]:
                buckets[bucket][classification] += 1
        summary[label] = buckets
    summary['alias_type'] = {name: sum(item.get('alias_type') == name for item in results)
                             for name in ('CROSS_ROW_ALIAS', 'SAME_ROW_LONGITUDINAL_ALIAS', 'YAW_ALIAS')}
    summary['alias_type']['UNKNOWN_OR_NOT_DERIVED'] = sum(not item.get('alias_type') for item in results)
    return summary


def _refresh_digest(study: dict[str, Any]) -> None:
    study.pop('content_sha256', None)
    study['content_sha256'] = _asset_digest(study)


def _write_summary(study_dir: Path, study: dict[str, Any]) -> None:
    summary = _study_summary(study)
    _write_yaml_atomic(study_dir / 'summary.yaml', summary, overwrite=True)
    with (study_dir / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['metric', 'value', 'denominator'])
        for key in ('total_queries', 'completed', 'CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT', 'NO_DATA'):
            writer.writerow([key, summary.get(key), ''])
        writer.writerow(['false_accept_rate', summary['false_accept_rate'], summary['false_accept_rate_denominator']])
        writer.writerow(['reject_rate', summary['reject_rate'], summary['reject_rate_denominator']])
        for k, record in summary['correct_at_k'].items():
            writer.writerow([f'correct@{k}', record.get('value'), record.get('denominator')])


def _study_query_set_binding_errors(study: dict[str, Any], study_dir: Path,
                                    query_set_path: Path,
                                    query_set: dict[str, Any]) -> list[str]:
    """Require a Study to consume the exact source/structure/block revisions of its frozen Query Set."""
    errors: list[str] = []
    query_anchor = query_set_path.resolve().parent
    bindings = (
        ('source_ref', ('identity', 'manifest_sha256', 'checksums_sha256', 'pose_revision', 'backend_id'),
         'source'),
        ('block_set_ref', ('block_set_id', 'revision', 'manifest_sha256', 'index_sha256',
                           'checksums_sha256', 'parent_source_digest', 'row_annotation_sha256'),
         'block set'),
    )
    for field, identity_fields, label in bindings:
        try:
            study_path = _resolve_ref(study[field], study_dir)
            query_path = _resolve_ref(query_set[field], query_anchor)
        except (KeyError, OSError, ValueError, TypeError):
            errors.append(f'Study {label} reference cannot be resolved from its Query Set')
            continue
        if study_path != query_path or any(study[field].get(key) != query_set[field].get(key)
                                           for key in identity_fields):
            errors.append(f'Study {label} does not match the exact frozen Query Set binding')
    study_annotation = study.get('annotation_ref')
    query_annotation = query_set.get('annotation_ref')
    if (study_annotation is None) != (query_annotation is None):
        errors.append('Study annotation does not match the exact frozen Query Set binding')
    elif study_annotation is not None and query_annotation is not None:
        try:
            study_path = _resolve_ref(study_annotation, study_dir)
            query_path = _resolve_ref(query_annotation, query_anchor)
        except (OSError, ValueError, TypeError):
            errors.append('Study annotation reference cannot be resolved from its Query Set')
        else:
            if (study_path != query_path or study_annotation.get('sha256') != query_annotation.get('sha256') or
                    study_annotation.get('revision') != query_annotation.get('revision')):
                errors.append('Study annotation does not match the exact frozen Query Set binding')
    if query_set.get('status') != 'FROZEN':
        errors.append('Study must reference a frozen Query Set revision')
    if study.get('query_set_ref', {}).get('content_sha256') != query_set.get('content_sha256'):
        errors.append('Study Query Set digest does not match the referenced revision')
    return errors


def run_study(study_path: Path, *, ros_install: Path, native_localizer_path: Path | None = None) -> dict[str, Any]:
    study_path = study_path.resolve(strict=True)
    study_dir = study_path.parent
    study = _load_yaml(study_path)
    errors = _validate_study_schema(study)
    if errors:
        raise ValueError(f'invalid study manifest: {errors}')
    query_set_path = _resolve_ref(study['query_set_ref'], study_dir)
    check = validate_query_set(query_set_path)
    if not check['valid'] or check['status'] != 'FROZEN':
        study['status'] = 'STALE'
        study.setdefault('issues', []).append(f'Query Set is not current and frozen: {check["errors"]}')
        _refresh_digest(study)
        _write_yaml_atomic(study_path, study, overwrite=True)
        raise ValueError(f'study Query Set is stale or invalid: {check}')
    query_set = _load_yaml(query_set_path)
    binding_errors = _study_query_set_binding_errors(study, study_dir, query_set_path, query_set)
    if binding_errors:
        study['status'] = 'STALE'
        study.setdefault('issues', []).extend(binding_errors)
        _refresh_digest(study)
        _write_yaml_atomic(study_path, study, overwrite=True)
        raise ValueError(f'study no longer matches its frozen Query Set binding: {binding_errors}')
    source = _resolve_ref(study['source_ref'], study_dir)
    block = _resolve_ref(study['block_set_ref'], study_dir)
    topology_path = (_resolve_ref(study['annotation_ref'], study_dir)
                     if study.get('annotation_ref') else None)
    total = sum(bool(query.get('enabled', True)) for query in query_set['queries']) * len(study['query_accumulation_frames'])
    study['status'] = 'RUNNING'
    study['job'].update(cancel_requested=False, completed=0, failed=0, total=total,
                        current_query=None, started_utc=_now(), finished_utc=None)
    study['results'] = []
    _refresh_digest(study)
    _write_yaml_atomic(study_path, study, overwrite=True)
    cancellation = {'requested': False}
    previous_handlers = {}

    def request_cancel(signum, _frame):
        cancellation['requested'] = True
        study['job']['cancel_requested'] = True
        study['job']['cancel_requested_utc'] = _now()
        _refresh_digest(study)
        _write_yaml_atomic(study_path, study, overwrite=True)

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, request_cancel)
    try:
        for query in query_set['queries']:
            if not query.get('enabled', True):
                continue
            for frames in study['query_accumulation_frames']:
                if cancellation['requested']:
                    break
                identifier = f"{query['query_id']}/f{frames}"
                study['job']['current_query'] = identifier
                _refresh_digest(study)
                _write_yaml_atomic(study_path, study, overwrite=True)
                output_root = study_dir / 'evidence' / query['query_id'] / f'f{frames}'
                output_root.mkdir(parents=True, exist_ok=True)
                args = argparse.Namespace(
                    map_package=source, block_dir=block, output_root=output_root,
                    query_keyframe=int(query['resolved_keyframe_id']), query_x=None, query_y=None,
                    row_position=False, row_id=None, along_row_s_m=None, topology=topology_path,
                    query_accumulation_frames=int(frames), candidate_top_k=int(study['candidate_top_k']),
                    max_snap_distance_m=3.0,
                    time_budget_s=float(study['algorithm']['time_budget_s']),
                    ros_install=ros_install.resolve(), native_localizer_path=native_localizer_path)
                started = time.monotonic()
                row: dict[str, Any] = {
                    'query_id': query['query_id'], 'scene': query['scene'], 'row_id': query.get('row_id', 'UNKNOWN'),
                    'along_row_s_m': query.get('along_row_s_m'),
                    'query_accumulation_frames': int(frames), 'keyframe': int(query['resolved_keyframe_id']),
                    'reference_level': query.get('reference_level', 'UNKNOWN'),
                    'requested_location': query['requested_location'],
                    'classification': 'NOT_RUN', 'evidence_ref': None,
                    'selected_candidate_rank': None, 'elapsed_s': None,
                }
                try:
                    evidence_path = run_query(args)
                    evidence_path = Path(evidence_path).resolve(strict=True)
                    verified = verify_evidence_bundle(evidence_path, source_package=source,
                                                      block_dir=block, topology_path=topology_path)
                    if verified.get('revision_state') != 'CURRENT':
                        raise ValueError(f'evidence did not verify CURRENT: {verified}')
                    evidence = _load_yaml(evidence_path / 'manifest.yaml')
                    empirical = evidence['evidence']['empirical_global_result']
                    row['classification'] = empirical.get('classification', 'REFERENCE_UNKNOWN')
                    selected = evidence['evidence'].get('candidate_ambiguity', {}).get('candidates', [])
                    selected_candidate = next((item for item in selected if item.get('classification') == row['classification']), None)
                    row['selected_candidate_rank'] = selected_candidate.get('rank') if selected_candidate else None
                    row['evidence_ref'] = _relative_ref(evidence_path, study_dir)
                    row['evidence_manifest_sha256'] = sha256(evidence_path / 'manifest.yaml')
                    row['gicp_converged'] = empirical.get('converged')
                    row['gicp_fitness'] = (selected_candidate or {}).get('gicp_fitness')
                    row['reference_error'] = empirical.get('reference_error', {})
                    row['candidate_count'] = len(selected)
                    if row['classification'] == 'FALSE_ACCEPT':
                        candidates = evidence['evidence'].get('candidate_ambiguity', {}).get('candidates', [])
                        query_row = query.get('row_id', 'UNKNOWN')
                        rank_one = next((item for item in candidates if item.get('rank') == 1), {})
                        candidate_row = rank_one.get('row_id', 'UNKNOWN')
                        if query_row != 'UNKNOWN' and candidate_row != 'UNKNOWN':
                            row['alias_type'] = ('CROSS_ROW_ALIAS' if query_row != candidate_row
                                                 else 'SAME_ROW_LONGITUDINAL_ALIAS')
                except Exception as exc:  # one query failure must not discard completed evidence
                    row['classification'] = 'NO_DATA'
                    row['error'] = str(exc)
                    study['job']['failed'] += 1
                row['elapsed_s'] = time.monotonic() - started
                study['results'].append(row)
                study['job']['completed'] += 1
                _refresh_digest(study)
                _write_yaml_atomic(study_path, study, overwrite=True)
            if cancellation['requested']:
                break
        remaining = max(0, total - study['job']['completed'])
        for query in query_set['queries']:
            if not query.get('enabled', True):
                continue
            for frames in study['query_accumulation_frames']:
                if not any(item['query_id'] == query['query_id'] and item['query_accumulation_frames'] == frames
                           for item in study['results']):
                    if not cancellation['requested']:
                        continue
                    study['results'].append({
                        'query_id': query['query_id'], 'scene': query['scene'],
                        'row_id': query.get('row_id', 'UNKNOWN'), 'along_row_s_m': query.get('along_row_s_m'),
                        'query_accumulation_frames': frames, 'keyframe': query['resolved_keyframe_id'],
                        'reference_level': query.get('reference_level', 'UNKNOWN'),
                        'classification': 'NOT_RUN', 'evidence_ref': None,
                    })
        has_cancelled = cancellation['requested']
        failures = int(study['job']['failed'])
        study['status'] = 'CANCELED' if has_cancelled else ('PARTIAL' if failures and study['job']['completed'] > failures
                                                            else ('FAILED' if failures else 'SUCCEEDED'))
        study['job']['current_query'] = None
        study['job']['finished_utc'] = _now()
        study['job']['cancel_requested'] = bool(has_cancelled)
        study['job']['not_run'] = remaining if has_cancelled else 0
        _refresh_digest(study)
        _write_yaml_atomic(study_path, study, overwrite=True)
        _write_summary(study_dir, study)
        return study
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def validate_study(path: Path, *, verify_evidence: bool = True,
                   ros_install: Path | None = None) -> dict[str, Any]:
    study_path = path.resolve(strict=True)
    study = _load_yaml(study_path)
    errors = _validate_study_schema(study)
    stale: list[str] = []
    try:
        query_set_path = _resolve_ref(study['query_set_ref'], study_path.parent)
        check = validate_query_set(query_set_path)
        if not check['valid']:
            stale.extend(check['errors'])
        query_set = _load_yaml(query_set_path)
        stale.extend(_study_query_set_binding_errors(study, study_path.parent, query_set_path, query_set))
        source_path = _resolve_ref(study['source_ref'], study_path.parent)
        block_path = _resolve_ref(study['block_set_ref'], study_path.parent)
        topology_path = (_resolve_ref(study['annotation_ref'], study_path.parent)
                         if study.get('annotation_ref') else None)
        current = _source_ref(source_path, study_path.parent)
        if current.get('manifest_sha256') != study['source_ref'].get('manifest_sha256'):
            stale.append('source manifest changed')
        if _block_ref(block_path, source_path, study_path.parent).get('manifest_sha256') != study['block_set_ref'].get('manifest_sha256'):
            stale.append('block set manifest changed')
        for result in study.get('results', []):
            reference = result.get('evidence_ref')
            if not reference:
                continue
            try:
                evidence_path = _resolve_ref(reference, study_path.parent)
                if sha256(evidence_path / 'manifest.yaml') != result.get('evidence_manifest_sha256'):
                    stale.append(f"{result.get('query_id')}: evidence manifest changed")
                if verify_evidence:
                    check_evidence = verify_evidence_bundle(evidence_path, source_package=source_path,
                                                           block_dir=block_path,
                                                           topology_path=topology_path)
                    if check_evidence.get('revision_state') != 'CURRENT':
                        stale.extend(f"{result.get('query_id')}: {reason}"
                                     for reason in check_evidence.get('stale_reasons', []))
            except (OSError, ValueError) as exc:
                stale.append(f"{result.get('query_id')}: evidence unavailable: {exc}")
        if ros_install is not None and study.get('status') in ('SUCCEEDED', 'PARTIAL', 'FAILED'):
            try:
                evidence_with_algorithm = next((
                    _load_yaml(_resolve_ref(item['evidence_ref'], study_path.parent) / 'manifest.yaml')
                    for item in study.get('results', []) if item.get('evidence_ref')), None)
                if evidence_with_algorithm is not None:
                    algorithm_state, reasons = current_algorithm_state(evidence_with_algorithm, ros_install)
                    if algorithm_state != 'CURRENT':
                        stale.extend(reasons)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                stale.append(f'algorithm staleness check failed: {exc}')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        stale.append(f'dependency resolution failed: {exc}')
    return {'valid': not errors, 'state': 'STALE' if stale else ('INVALID' if errors else 'CURRENT'),
            'errors': errors, 'stale_reasons': stale, 'study_id': study.get('study_id'),
            'status': study.get('status'), 'result_count': len(study.get('results', []))}


def export_study(study_path: Path, output_dir: Path) -> None:
    checked = validate_study(study_path)
    if not checked['valid'] or checked['state'] == 'STALE':
        raise ValueError(f'cannot export invalid/stale study: {checked}')
    study_dir = study_path.resolve().parent
    study = _load_yaml(study_path)
    output_dir.mkdir(parents=True, exist_ok=False)
    for name in ('summary.yaml', 'summary.csv'):
        source = study_dir / name
        if source.is_file():
            shutil.copy2(source, output_dir / name)
    with (output_dir / 'queries.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['query_id', 'scene', 'row_id', 'along_row_s_m',
                                                    'query_accumulation_frames', 'keyframe', 'classification',
                                                    'reference_level', 'evidence_ref'])
        writer.writeheader()
        for item in study.get('results', []):
            row = {key: item.get(key) for key in writer.fieldnames}
            row['evidence_ref'] = item.get('evidence_ref', {}).get('path') if item.get('evidence_ref') else ''
            writer.writerow(row)
    candidate_rows: list[dict[str, Any]] = []
    for result in study.get('results', []):
        if not result.get('evidence_ref'):
            continue
        evidence_path = _resolve_ref(result['evidence_ref'], study_dir)
        evidence = _load_yaml(evidence_path / 'manifest.yaml')
        for candidate in evidence['evidence'].get('candidate_ambiguity', {}).get('candidates', []):
            candidate_rows.append({**candidate,
                                   'query_id': result['query_id'],
                                   'frames': result['query_accumulation_frames']})
    base_candidate_fields = ['query_id', 'frames', 'rank', 'candidate_keyframe',
                             'candidate_block_ids', 'row_id', 'along_row_s_m',
                             'bbs_score', 'gicp_attempted', 'gicp_converged',
                             'gicp_fitness', 'classification']
    extra_candidate_fields = sorted({key for row in candidate_rows for key in row}
                                    - set(base_candidate_fields))
    with (output_dir / 'candidates.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=base_candidate_fields + extra_candidate_fields)
        writer.writeheader()
        for row in candidate_rows:
            writer.writerow({key: json.dumps(value, sort_keys=True) if isinstance(value, (dict, list))
                             else '' if value is None else value
                             for key, value in row.items()})
    _write_yaml_atomic(output_dir / 'study_manifest.yaml', study, overwrite=False)


def create_project(*, project_path: Path, map_package: Path, block_dir: Path,
                   topology_path: Path | None = None, query_set_paths: list[Path] | None = None,
                   study_paths: list[Path] | None = None) -> dict[str, Any]:
    root = project_path.resolve().parent
    source = _source_ref(map_package, root)
    block = _block_ref(block_dir, map_package, root)
    annotation = _annotation_ref(topology_path, map_package, root) if topology_path else None
    project = {
        'schema_version': 1, 'asset_type': PROJECT_TYPE,
        'project_id': project_path.stem, 'created_utc': _now(),
        'source_ref': source, 'block_set_ref': block, 'annotation_ref': annotation,
        'query_sets': [_relative_ref(path.resolve(strict=True), root) for path in (query_set_paths or [])],
        'studies': [_relative_ref(path.resolve(strict=True), root) for path in (study_paths or [])],
        'asset_status': 'CURRENT',
    }
    project['content_sha256'] = _asset_digest(project)
    _write_yaml_atomic(project_path, project, overwrite=False)
    return project


def validate_project(project_path: Path) -> dict[str, Any]:
    project = _load_yaml(project_path)
    errors, stale = [], []
    if project.get('schema_version') != 1 or project.get('asset_type') != PROJECT_TYPE:
        errors.append('unsupported MapStudio project manifest')
    if project.get('content_sha256') != _asset_digest(project):
        errors.append('project content_sha256 mismatch')
    root = project_path.resolve().parent
    try:
        source_path = _resolve_ref(project['source_ref'], root)
        current_source = _source_ref(source_path, root)
        if current_source['manifest_sha256'] != project['source_ref'].get('manifest_sha256'):
            stale.append('source manifest changed')
        block_path = _resolve_ref(project['block_set_ref'], root)
        current_block = _block_ref(block_path, source_path, root)
        if current_block['manifest_sha256'] != project['block_set_ref'].get('manifest_sha256'):
            stale.append('block set manifest changed')
        if project.get('annotation_ref'):
            annotation_path = _resolve_ref(project['annotation_ref'], root)
            if sha256(annotation_path) != project['annotation_ref'].get('sha256'):
                stale.append('annotation revision changed')
        for query_ref in project.get('query_sets', []):
            query_path = _resolve_ref(query_ref, root)
            check = validate_query_set(query_path)
            if not check['valid']:
                stale.append(f'Query Set {query_path.name}: {check["errors"]}')
        for study_ref in project.get('studies', []):
            study_path = _resolve_ref(study_ref, root)
            check = validate_study(study_path)
            if not check['valid'] or check['state'] == 'STALE':
                stale.append(f'Study {study_path.parent.name}: {check}')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        stale.append(f'project dependency could not be resolved: {exc}')
    return {'valid': not errors, 'state': 'INVALID' if errors else ('STALE' if stale else 'CURRENT'),
            'errors': errors, 'stale_reasons': stale,
            'project_id': project.get('project_id'),
            'source_identity': project.get('source_ref', {}).get('identity'),
            'query_set_count': len(project.get('query_sets', [])),
            'study_count': len(project.get('studies', []))}


def add_project_assets(project_path: Path, *, query_set_paths: list[Path] | None = None,
                       study_paths: list[Path] | None = None) -> dict[str, Any]:
    """Update mutable project membership without mutating referenced assets."""
    project_path = project_path.resolve(strict=True)
    project = _load_yaml(project_path)
    if project.get('asset_type') != PROJECT_TYPE or project.get('content_sha256') != _asset_digest(project):
        raise ValueError('cannot update an invalid project manifest')
    root = project_path.parent
    for field, paths in (('query_sets', query_set_paths or []), ('studies', study_paths or [])):
        refs = project.setdefault(field, [])
        known = {item.get('path') for item in refs}
        for candidate in paths:
            path = candidate.resolve(strict=True)
            ref = _relative_ref(path, root)
            if ref['path'] not in known:
                refs.append(ref)
                known.add(ref['path'])
    _refresh_digest(project)
    _write_yaml_atomic(project_path, project, overwrite=True)
    return project


def set_project_dependencies(project_path: Path, *, map_package: Path, block_dir: Path,
                             topology_path: Path | None = None) -> dict[str, Any]:
    """Rebind the mutable workspace project to a new exact core asset revision."""
    project_path = project_path.resolve(strict=True)
    project = _load_yaml(project_path)
    if project.get('asset_type') != PROJECT_TYPE or project.get('content_sha256') != _asset_digest(project):
        raise ValueError('cannot update an invalid project manifest')
    root = project_path.parent
    source = _source_ref(map_package, root)
    block = _block_ref(block_dir, map_package, root)
    annotation = _annotation_ref(topology_path, map_package, root) if topology_path else None
    if block.get('row_annotation_sha256') is not None and (
            annotation is None or annotation.get('sha256') != block['row_annotation_sha256']):
        raise ValueError('selected block set requires the exact selected row annotation revision')
    if annotation is not None and block.get('row_annotation_sha256') != annotation.get('sha256'):
        raise ValueError('selected block set was not built against the selected annotation revision')
    project['source_ref'] = source
    project['block_set_ref'] = block
    project['annotation_ref'] = annotation
    project['dependencies_updated_utc'] = _now()
    _refresh_digest(project)
    _write_yaml_atomic(project_path, project, overwrite=True)
    return project


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='MapStudio Query Set / Relocalization Study application services.')
    commands = parser.add_subparsers(dest='command', required=True)
    manual = commands.add_parser('query-set-manual')
    manual.add_argument('--id', required=True)
    manual.add_argument('--map-package', type=Path, required=True)
    manual.add_argument('--block-dir', type=Path, required=True)
    manual.add_argument('--locations-json', type=Path, required=True,
                        help='JSON list of {query_id?, scene?, xy:[map_x,map_y]}')
    manual.add_argument('--topology', type=Path)
    manual.add_argument('--output', type=Path, required=True)
    sample = commands.add_parser('query-set-sample')
    sample.add_argument('--id', required=True)
    sample.add_argument('--map-package', type=Path, required=True)
    sample.add_argument('--block-dir', type=Path, required=True)
    sample.add_argument('--topology', type=Path, required=True)
    sample.add_argument('--output', type=Path, required=True)
    sample.add_argument('--ratios', default='0,0.25,0.5,0.75,1')
    validate_qs = commands.add_parser('query-set-validate')
    validate_qs.add_argument('query_set', type=Path)
    freeze_qs = commands.add_parser('query-set-freeze')
    freeze_qs.add_argument('query_set', type=Path)
    freeze_qs.add_argument('--output', type=Path, required=True)
    clone_qs = commands.add_parser('query-set-clone-draft')
    clone_qs.add_argument('query_set', type=Path)
    clone_qs.add_argument('--output', type=Path, required=True)
    add_q = commands.add_parser('query-set-add-manual')
    add_q.add_argument('query_set', type=Path)
    add_q.add_argument('--id')
    add_q.add_argument('--scene', choices=sorted(VALID_SCENES), default='OTHER')
    add_q.add_argument('--xy', type=float, nargs=2, required=True)
    enabled_q = commands.add_parser('query-set-set-enabled')
    enabled_q.add_argument('query_set', type=Path)
    enabled_q.add_argument('--id', required=True)
    enabled_q.add_argument('--enabled', choices=('true', 'false'), required=True)
    delete_q = commands.add_parser('query-set-delete-draft-query')
    delete_q.add_argument('query_set', type=Path)
    delete_q.add_argument('--id', required=True)
    create = commands.add_parser('study-create')
    create.add_argument('--id', required=True)
    create.add_argument('--query-set', type=Path, required=True)
    create.add_argument('--output-dir', type=Path, required=True)
    create.add_argument('--frames', default='1,3,5')
    create.add_argument('--candidate-top-k', type=int, default=5)
    create.add_argument('--time-budget-s', type=float, default=18.0)
    validate_s = commands.add_parser('study-validate')
    validate_s.add_argument('study', type=Path)
    run = commands.add_parser('run-study')
    run.add_argument('study', type=Path)
    run.add_argument('--ros-install', type=Path, default=Path.home() / 'ros2_ws/install')
    run.add_argument('--native-localizer-path', type=Path)
    export = commands.add_parser('study-export')
    export.add_argument('study', type=Path)
    export.add_argument('--output-dir', type=Path, required=True)
    project = commands.add_parser('project-init')
    project.add_argument('--project', type=Path, required=True)
    project.add_argument('--map-package', type=Path, required=True)
    project.add_argument('--block-dir', type=Path, required=True)
    project.add_argument('--topology', type=Path)
    project_v = commands.add_parser('project-validate')
    project_v.add_argument('project', type=Path)
    project_add = commands.add_parser('project-add-assets')
    project_add.add_argument('project', type=Path)
    project_add.add_argument('--query-set', type=Path, action='append', default=[])
    project_add.add_argument('--study', type=Path, action='append', default=[])
    project_set = commands.add_parser('project-set-dependencies')
    project_set.add_argument('project', type=Path)
    project_set.add_argument('--map-package', type=Path, required=True)
    project_set.add_argument('--block-dir', type=Path, required=True)
    project_set.add_argument('--topology', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == 'query-set-manual':
            value = create_manual_query_set(query_set_id=args.id, map_package=args.map_package,
                                            block_dir=args.block_dir,
                                            locations=json.loads(args.locations_json.read_text(encoding='utf-8')),
                                            output=args.output, topology_path=args.topology)
            result = {'status': 'PASS', 'path': str(args.output), 'query_count': len(value['queries'])}
        elif args.command == 'query-set-sample':
            ratios = tuple(float(value) for value in args.ratios.split(','))
            value = create_structure_query_set(query_set_id=args.id, map_package=args.map_package,
                                               block_dir=args.block_dir, topology_path=args.topology,
                                               output=args.output, ratios=ratios)
            result = {'status': 'PASS', 'path': str(args.output), 'query_count': len(value['queries'])}
        elif args.command == 'query-set-validate':
            result = validate_query_set(args.query_set)
        elif args.command == 'query-set-freeze':
            value = freeze_query_set(args.query_set, args.output)
            result = {'status': 'PASS', 'path': str(args.output), 'content_sha256': value['content_sha256']}
        elif args.command == 'query-set-clone-draft':
            value = clone_query_set_as_draft(args.query_set, args.output)
            result = {'status': 'PASS', 'path': str(args.output), 'content_sha256': value['content_sha256']}
        elif args.command == 'query-set-add-manual':
            value = add_manual_query(args.query_set, xy=(args.xy[0], args.xy[1]),
                                     scene=args.scene, query_id=args.id)
            result = {'status': 'PASS', 'query_id': value['query_id'],
                      'resolved_keyframe_id': value['resolved_keyframe_id'],
                      'snap_distance_m': value['snap_distance_m']}
        elif args.command == 'query-set-set-enabled':
            set_query_enabled(args.query_set, args.id, args.enabled == 'true')
            result = {'status': 'PASS', 'query_id': args.id, 'enabled': args.enabled == 'true'}
        elif args.command == 'query-set-delete-draft-query':
            delete_draft_query(args.query_set, args.id)
            result = {'status': 'PASS', 'query_id': args.id, 'deleted': True}
        elif args.command == 'study-create':
            frames = tuple(int(value) for value in args.frames.split(','))
            value = create_study(study_id=args.id, query_set_path=args.query_set,
                                 output_dir=args.output_dir, frames=frames,
                                 candidate_top_k=args.candidate_top_k,
                                 time_budget_s=args.time_budget_s)
            result = {'status': 'PASS', 'path': str(args.output_dir / 'study.yaml'),
                      'job_total': value['job']['total']}
        elif args.command == 'study-validate':
            result = validate_study(args.study, ros_install=Path.home() / 'ros2_ws/install')
        elif args.command == 'run-study':
            value = run_study(args.study, ros_install=args.ros_install,
                              native_localizer_path=args.native_localizer_path)
            result = {'status': value['status'], 'study_path': str(args.study),
                      'summary': _study_summary(value)}
        elif args.command == 'study-export':
            export_study(args.study, args.output_dir)
            result = {'status': 'PASS', 'output_dir': str(args.output_dir)}
        elif args.command == 'project-init':
            value = create_project(project_path=args.project, map_package=args.map_package,
                                   block_dir=args.block_dir, topology_path=args.topology)
            result = {'status': 'PASS', 'project_path': str(args.project),
                      'content_sha256': value['content_sha256']}
        elif args.command == 'project-validate':
            result = validate_project(args.project)
        elif args.command == 'project-add-assets':
            value = add_project_assets(args.project, query_set_paths=args.query_set,
                                       study_paths=args.study)
            result = {'status': 'PASS', 'content_sha256': value['content_sha256'],
                      'query_set_count': len(value.get('query_sets', [])),
                      'study_count': len(value.get('studies', []))}
        elif args.command == 'project-set-dependencies':
            value = set_project_dependencies(args.project, map_package=args.map_package,
                                             block_dir=args.block_dir, topology_path=args.topology)
            result = {'status': 'PASS', 'content_sha256': value['content_sha256'],
                      'source_identity': value['source_ref']['identity'],
                      'block_set_id': value['block_set_ref']['block_set_id']}
        else:
            raise ValueError(f'unsupported command: {args.command}')
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result.get('status', 'PASS') not in ('FAIL',) and result.get('valid', True) else 2
    except Exception as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc)}, indent=2), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
