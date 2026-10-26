"""Application-layer single-query operation for MapStudio Relocalization MVP.

The service binds the current validated map source, immutable block revision,
query window, existing GLOBAL/Top-K/GICP pipeline and an evidence bundle. It
does not implement registration or write into the source or block assets.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from typing import Any
import uuid

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from agt_mapping_artifacts.frontend_package import require_output_outside, sha256
from agt_mapping_artifacts.keyframe_blocks import verify_keyframe_blocks
from agt_mapping_artifacts.relocalization_evidence import publish_evidence_bundle, verify_evidence_bundle

from .backends import GLOBAL_SETTINGS, NativePrograms, dependency_fingerprints
from .greenhouse import MapPackage, Scene, SceneConfig, run_global
from .pcd import sha256_file, write_pcd


def canonical_digest(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def supports_candidate_trace(binary: Path) -> bool:
    """Probe the existing native CLI before paying the cost of building map assets."""
    try:
        # This native CLI does not implement --help; an empty invocation emits
        # its usage after argument validation without loading map data.
        result = subprocess.run([str(binary)], text=True, capture_output=True,
                                timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return '--trace-candidates-json' in (result.stdout + result.stderr)


def git_provenance(repository: Path) -> dict[str, Any]:
    def call(*args: str) -> str | None:
        try:
            return subprocess.check_output(['git', '-C', str(repository), *args], text=True,
                                           stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    head = call('rev-parse', 'HEAD')
    status = call('status', '--porcelain=v1', '--untracked-files=all')
    tracked_diff = call('diff', '--binary', 'HEAD', '--')
    untracked_state = []
    try:
        paths = subprocess.check_output(['git', '-C', str(repository), 'ls-files', '--others',
                                         '--exclude-standard', '-z'], stderr=subprocess.DEVNULL).split(b'\0')
        for raw in filter(None, paths):
            path = repository / os.fsdecode(raw)
            if path.is_file() and not path.is_symlink():
                untracked_state.append({'path': os.fsdecode(raw), 'sha256': sha256(path)})
    except (OSError, subprocess.CalledProcessError):
        untracked_state = []
    source_state_sha256 = (canonical_digest({
        'head': head,
        'status': status,
        'tracked_diff_sha256': hashlib.sha256((tracked_diff or '').encode()).hexdigest(),
        'untracked_files': untracked_state,
    }) if head is not None and status is not None and tracked_diff is not None else None)
    return {'repository': str(repository), 'head': head,
            'dirty': bool(status) if status is not None else None,
            'source_state_sha256': source_state_sha256}


def mapping_repository(ros_install: Path) -> Path:
    """Resolve the source checkout behind a normal ROS workspace install."""
    installed_source = Path(ros_install).expanduser().resolve().parent / 'src' / 'agt_mapping_framework'
    if (installed_source / '.git').exists():
        return installed_source
    source_checkout = Path(__file__).resolve().parents[2]
    return source_checkout


def native_dependency_fingerprints(ros_install: Path, repository: Path) -> dict[str, Any]:
    """Fingerprint vendored sources from the ROS workspace src root."""
    return dependency_fingerprints(ros_install.expanduser().resolve(), repository.parent)


def algorithm_stale_reasons(record: dict[str, Any], *, current_binary_sha256: dict[str, str],
                            current_code_provenance: dict[str, Any],
                            current_global_settings: dict[str, Any]) -> list[str]:
    algorithm = record.get('algorithm', {})
    reasons = []
    if algorithm.get('binary_sha256') != current_binary_sha256:
        reasons.append('native algorithm binary revision changed')
    config = algorithm.get('config', {})
    if config.get('global_settings') != current_global_settings:
        reasons.append('native GLOBAL analysis configuration changed')
    recorded_code = algorithm.get('code_provenance', {})
    if (recorded_code.get('head') != current_code_provenance.get('head') or
            recorded_code.get('source_state_sha256') is None or
            recorded_code.get('source_state_sha256') != current_code_provenance.get('source_state_sha256')):
        reasons.append('mapping analysis source revision changed')
    return reasons


def current_algorithm_state(record: dict[str, Any], ros_install: Path) -> tuple[str, list[str]]:
    native = NativePrograms.discover(ros_install.expanduser().resolve())
    if not native.global_ready:
        return 'STALE', ['current native GLOBAL/Top-K/GICP programs are unavailable']
    binaries = dict(native.sha256)
    selection = record.get('algorithm', {}).get('binary_selection', {})
    if selection.get('candidate_localizer_override') is True:
        selected = Path(selection.get('candidate_localizer_path', '')).expanduser()
        try:
            selected = selected.resolve(strict=True)
        except OSError:
            return 'STALE', ['recorded trace-capable native candidate binary is unavailable']
        if not selected.is_file() or not os.access(selected, os.X_OK):
            return 'STALE', ['recorded trace-capable native candidate binary is not executable']
        binaries['candidate_bbs_gicp_localizer'] = sha256(selected)
    algorithm = record.get('algorithm', {})
    saved_settings = algorithm.get('config', {}).get('global_settings', {})
    settings = dict(GLOBAL_SETTINGS)
    settings['candidate_top_k'] = algorithm.get('candidate_top_k')
    settings['descriptor_prefilter'] = algorithm.get('descriptor_prefilter')
    settings['timeout_sec'] = algorithm.get('time_budget_s')
    reasons = algorithm_stale_reasons(
        record, current_binary_sha256=binaries,
        current_code_provenance=git_provenance(mapping_repository(ros_install)),
        current_global_settings=settings)
    current_dependencies = native_dependency_fingerprints(
        ros_install, mapping_repository(ros_install))
    if algorithm.get('dependency_fingerprints') != current_dependencies:
        reasons.append('native algorithm dependency library/source revision changed')
    if saved_settings != settings and 'native GLOBAL analysis configuration changed' not in reasons:
        reasons.append('native GLOBAL analysis configuration changed')
    if algorithm.get('config', {}).get('native_binary_sha256') != binaries:
        if 'native algorithm binary revision changed' not in reasons:
            reasons.append('native algorithm binary revision changed')
    return ('STALE' if reasons else 'CURRENT'), reasons


def _frame_id(dataset: MapPackage, keyframe: int) -> int:
    if keyframe < 0 or keyframe >= len(dataset.poses):
        raise ValueError(f'query keyframe outside source: {keyframe}')
    patch_stem = Path(dataset.poses[keyframe].patch).stem
    if not patch_stem.isdigit():
        raise ValueError(f'query patch has no numeric frame identity: {patch_stem}')
    return int(patch_stem)


def _load_topology(topology_path: Path, source: Path):
    from agt_greenhouse_annotation.topology import labels_for_package, load_map_package, load_topology
    from agt_greenhouse_annotation.validator import validate_topology

    topology = load_topology(topology_path)
    annotation_map = load_map_package(source)
    label_file = topology_path.parent / 'keyframe_topology_labels.csv'
    frozen = (topology.get('annotation', {}).get('status') == 'frozen' and
              topology.get('annotation', {}).get('manual_review_confirmed') is True)
    validation = validate_topology(topology, package=annotation_map,
                                   annotation_path=topology_path if frozen else None,
                                   labels_path=label_file if label_file.exists() else None,
                                   verify_map_files=True)
    if not validation.get('valid'):
        raise ValueError(f'topology/source validation failed: {validation.get("errors", [])}')
    labels = labels_for_package(topology, annotation_map)
    return topology, labels


def _resolve_query(dataset: MapPackage, args, topology_labels=None) -> tuple[int, dict[str, Any]]:
    if args.query_keyframe is not None:
        matches = [pose.index for pose in dataset.poses
                   if Path(pose.patch).stem == str(args.query_keyframe)]
        if not matches:
            raise ValueError(f'query keyframe ID not found: {args.query_keyframe}')
        center = matches[0]
        selection = {'kind': 'keyframe'}
    elif args.query_x is not None and args.query_y is not None:
        distances = [float(np.linalg.norm(p.t[:2] - np.asarray([args.query_x, args.query_y])))
                     for p in dataset.poses]
        center = int(np.argmin(distances))
        selection = {'kind': 'map_point', 'click_xy_m': [args.query_x, args.query_y],
                     'snap_distance_m': distances[center]}
        if args.max_snap_distance_m is not None and distances[center] > args.max_snap_distance_m:
            raise ValueError(f'nearest query frame is {distances[center]:.3f} m from the selected map point '
                             f'(limit {args.max_snap_distance_m:.3f} m)')
    else:
        if not args.row_id or args.along_row_s_m is None or topology_labels is None:
            raise ValueError('row query requires --topology, --row-id and --along-row-s-m')
        choices = [(abs(float(item['along_row_s_m']) - args.along_row_s_m), item)
                   for item in topology_labels
                   if item.get('physical_row_id') == args.row_id and
                   item.get('label_confidence') == 'confirmed' and item.get('along_row_s_m') is not None]
        if not choices:
            raise ValueError(f'no reviewed keyframes exist on physical row {args.row_id!r}')
        distance, match = min(choices, key=lambda pair: pair[0])
        center = next(p.index for p in dataset.poses if Path(p.patch).stem == str(match['keyframe']))
        selection = {'kind': 'row_position', 'row_id': args.row_id,
                     'requested_along_row_s_m': args.along_row_s_m,
                     'actual_along_row_s_m': float(match['along_row_s_m']),
                     'along_row_snap_distance_m': distance}
    if center < 2 or center >= len(dataset.poses) - 2:
        raise ValueError('query center must have a complete legal five-frame window')
    selection.update({'keyframe': _frame_id(dataset, center), 'pose_index': center,
                      'patch_id': dataset.poses[center].patch,
                      'timestamp': dataset.poses[center].stamp})
    selected_pose = dataset.poses[center]
    selection['map_pose'] = {
        'x_m': float(selected_pose.t[0]), 'y_m': float(selected_pose.t[1]),
        'z_m': float(selected_pose.t[2]),
        'yaw_deg': float(Rotation.from_quat(selected_pose.quat_xyzw).as_euler('xyz', degrees=True)[2]),
        'qx': float(selected_pose.quat_xyzw[0]), 'qy': float(selected_pose.quat_xyzw[1]),
        'qz': float(selected_pose.quat_xyzw[2]), 'qw': float(selected_pose.quat_xyzw[3]),
    }
    return center, selection


def _query_block_exclusion(block_root: Path, window_frame_ids: list[int],
                           dataset: MapPackage) -> tuple[list[str], list[int]]:
    manifest = yaml.safe_load((block_root / 'manifest.yaml').read_text(encoding='utf-8'))
    index = json.loads((block_root / manifest['index_path']).read_text(encoding='utf-8'))
    keyframe_to_blocks = index.get('keyframe_to_blocks', {})
    touched = sorted({block_id for frame in window_frame_ids
                      for block_id in keyframe_to_blocks.get(str(frame), [])})
    by_id = {block['block_id']: block for block in manifest['blocks']}
    excluded = set(window_frame_ids)
    for block_id in touched:
        for patch_id in by_id[block_id]['ordered_patch_ids']:
            stem = Path(patch_id).stem
            if stem.isdigit():
                excluded.add(int(stem))
    frame_to_index = {Path(pose.patch).stem: pose.index for pose in dataset.poses}
    excluded_indices = sorted({frame_to_index[str(frame)] for frame in excluded
                               if str(frame) in frame_to_index})
    return touched, excluded_indices


def _classification(row: dict[str, Any]) -> str:
    if row.get('global_failure_code') == 'TIMEOUT':
        return 'TIMEOUT'
    if not row.get('global_backend_success'):
        return 'REJECTED'
    if row.get('final_nominal_success'):
        return 'CORRECT'
    return 'FALSE_ACCEPT'


def _ambiguity_status(candidates: list[dict[str, Any]]) -> str:
    def has_score(candidate: dict[str, Any]) -> bool:
        for key in ('descriptor_sector_similarity', 'bbs_score'):
            value = candidate.get(key)
            if (isinstance(value, (int, float)) and not isinstance(value, bool) and
                    math.isfinite(float(value))):
                return True
        return False

    return 'KNOWN' if sum(has_score(candidate) for candidate in candidates) >= 2 else 'NO_DATA'


def _candidate_rows(trace: dict[str, Any], block_index: dict[str, Any], block_manifest: dict[str, Any],
                    dataset: MapPackage, top_k: int) -> list[dict[str, Any]]:
    index_map = block_index.get('keyframe_to_blocks', {})
    blocks = {b['block_id']: b for b in block_manifest['blocks']}
    output = []
    for candidate in trace.get('ranked_candidates', [])[:top_k]:
        keyframe = candidate.get('keyframe')
        patch_id = Path(str(candidate.get('patch', ''))).name
        block_ids = index_map.get(str(keyframe), []) if isinstance(keyframe, int) else []
        matching_blocks = [blocks[b] for b in block_ids if b in blocks]
        descriptor = candidate.get('descriptor', {})
        bbs = candidate.get('bbs', {})
        gicp = candidate.get('gicp', {})
        output.append({
            'rank': candidate.get('rank'), 'candidate_patch_id': patch_id,
            'candidate_source_patch_sha256': (sha256_file(dataset.root / 'patches' / str(patch_id))
                                              if patch_id in dataset.patch_to_index else None),
            'candidate_keyframe': keyframe, 'candidate_block_ids': block_ids,
            'row_id': matching_blocks[0].get('row_id', 'UNKNOWN') if matching_blocks else 'UNKNOWN',
            'along_row_s_m': matching_blocks[0].get('along_row_s_m') if matching_blocks else None,
            'descriptor_sector_similarity': descriptor.get('sector_similarity'),
            'descriptor_ring_distance': descriptor.get('ring_distance'),
            'bbs_attempted': bbs.get('attempted'), 'bbs_valid': bbs.get('valid'),
            'bbs_score': bbs.get('score'), 'coarse_pose': bbs.get('coarse_pose'),
            'gicp_attempted': gicp.get('attempted'), 'gicp_converged': gicp.get('converged'),
            'gicp_fitness': gicp.get('fitness'), 'gicp_final_pose': gicp.get('final_pose'),
            'classification': 'UNKNOWN',
        })
    return output


def run_query(args) -> Path:
    source = args.map_package.expanduser().resolve(strict=True)
    block_root = args.block_dir.expanduser().resolve(strict=True)
    repository = mapping_repository(args.ros_install)
    output_root = require_output_outside(args.output_root, [source, block_root],
                                         label='evidence output root')
    source_check = verify_keyframe_blocks(block_root, source_package=source)
    dataset = MapPackage(source)
    topology, topology_labels = (None, None)
    topology_path = None
    topology_digest = None
    if args.topology:
        topology_path = args.topology.expanduser().resolve(strict=True)
        topology, topology_labels = _load_topology(topology_path, source)
        topology_digest = sha256(topology_path)
    if args.row_id and (topology is None or topology.get('annotation', {}).get('status') != 'frozen' or
                        topology.get('annotation', {}).get('manual_review_confirmed') is not True):
        raise ValueError('row query requires a frozen manually reviewed topology')
    if topology_path is not None and sha256(topology_path) != topology_digest:
        raise ValueError('topology annotation changed while preparing the query')
    center, selection = _resolve_query(dataset, args, topology_labels)
    if center - args.query_accumulation_frames // 2 < 0 or center + args.query_accumulation_frames // 2 >= len(dataset.poses):
        raise ValueError('query accumulation window exceeds source keyframes')
    block_manifest = yaml.safe_load((block_root / 'manifest.yaml').read_text(encoding='utf-8'))
    block_annotation_digest = block_manifest.get('row_annotation_sha256')
    if block_annotation_digest is not None:
        if (not isinstance(block_annotation_digest, str) or len(block_annotation_digest) != 64 or
                any(char not in '0123456789abcdef' for char in block_annotation_digest)):
            raise ValueError('block set has an invalid row annotation digest')
        if topology is None or topology_digest != block_annotation_digest:
            raise ValueError('block labels require the exact frozen topology revision used to build this block set')
    block_index = json.loads((block_root / block_manifest['index_path']).read_text(encoding='utf-8'))
    window = list(range(center - args.query_accumulation_frames // 2,
                        center + args.query_accumulation_frames // 2 + 1))
    frame_ids = [_frame_id(dataset, item) for item in window]
    excluded_blocks, extra_excluded = _query_block_exclusion(block_root, frame_ids, dataset)
    extra_excluded_frame_ids = [_frame_id(dataset, item) for item in extra_excluded]
    block_records_by_id = {item['block_id']: item for item in block_manifest['blocks']}
    query_block_details = [{
        'block_id': block_id,
        'revision': block_records_by_id[block_id]['revision'],
        'center_keyframe_id': block_records_by_id[block_id]['center_keyframe_id'],
        'ordered_patch_ids': block_records_by_id[block_id]['ordered_patch_ids'],
        'bbox': block_records_by_id[block_id]['bbox'],
        'output_content_sha256': block_records_by_id[block_id]['output_content_sha256'],
        'row_id': block_records_by_id[block_id].get('row_id', 'UNKNOWN'),
        'along_row_s_m': block_records_by_id[block_id].get('along_row_s_m'),
        'scene_type': block_records_by_id[block_id].get('scene_type', 'UNKNOWN'),
    } for block_id in excluded_blocks]
    window_material = []
    for i in window:
        pose = dataset.poses[i]
        patch_path = source / 'patches' / pose.patch
        window_material.append({'frame_id': _frame_id(dataset, i), 'patch_id': pose.patch,
                                'patch_sha256': sha256_file(patch_path),
                                'timestamp': pose.stamp,
                                'pose': [*map(float, pose.t), *map(float, pose.quat_xyzw)]})
    query_label = {'row_id': 'UNKNOWN', 'along_row_s_m': None, 'scene_type': 'UNKNOWN'}
    if topology_labels is not None and topology.get('annotation', {}).get('status') == 'frozen' and topology.get('annotation', {}).get('manual_review_confirmed') is True:
        selected_label = next((v for v in topology_labels if int(v['keyframe']) == _frame_id(dataset, center)), None)
        if selected_label and selected_label.get('label_confidence') == 'confirmed':
            query_label = {'row_id': selected_label.get('physical_row_id', 'UNKNOWN'),
                           'along_row_s_m': selected_label.get('along_row_s_m'),
                           'scene_type': selected_label.get('zone_type', 'UNKNOWN')}
    elif selection.get('kind') == 'row_position':
        query_label.update(row_id=selection['row_id'], along_row_s_m=selection['actual_along_row_s_m'])

    output_root.mkdir(parents=True, exist_ok=True)
    evidence_id = 'reloc-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    destination = output_root / evidence_id
    staging = Path(tempfile.mkdtemp(prefix=f'.{evidence_id}.analysis-', dir=output_root))
    analysis = staging / 'analysis'
    analysis.mkdir()
    result_row = None
    trace = {}
    classification = 'NOT_RUN'
    candidate_records = []
    try:
        native = NativePrograms.discover(args.ros_install.expanduser().resolve())
        if not native.global_ready:
            raise RuntimeError('existing GLOBAL / Top-K / GICP native programs are unavailable')
        if args.native_localizer_path is not None:
            override = args.native_localizer_path.expanduser().resolve(strict=True)
            if not override.is_file() or not os.access(override, os.X_OK):
                raise ValueError(f'native localizer override is not an executable file: {override}')
            native = NativePrograms(
                native.paths | {'candidate_bbs_gicp_localizer': override},
                native.sha256 | {'candidate_bbs_gicp_localizer': sha256(override)},
                native.available | {'candidate_bbs_gicp_localizer': True})
        if not supports_candidate_trace(native.paths['candidate_bbs_gicp_localizer']):
            raise RuntimeError('selected existing native localizer does not support --trace-candidates-json; '
                               'select an audited trace-capable binary with --native-localizer-path')
        settings = dict(GLOBAL_SETTINGS)
        settings['candidate_top_k'] = args.candidate_top_k
        settings['descriptor_prefilter'] = max(int(settings['descriptor_prefilter']), args.candidate_top_k)
        settings['timeout_sec'] = args.time_budget_s
        native_dependencies = native_dependency_fingerprints(args.ros_install, repository)
        scene_id = f'query_{_frame_id(dataset, center):06d}_f{args.query_accumulation_frames}'
        # row_middle is an evaluator-compatible internal enum only. The saved
        # evidence scene_type stays UNKNOWN unless a frozen annotation confirms it.
        config = SceneConfig((Scene(scene_id, 'row_middle', center, None,
                                    'technical query; physical scene is UNKNOWN unless frozen annotation confirms it'),),
                             (), 'physical row labels require frozen manual annotation')
        rows = run_global(dataset, config, native, analysis, (args.query_accumulation_frames,), None,
                          trace_candidates=True, global_settings=settings,
                          extra_excluded_indices=extra_excluded)
        result_row = rows[0]
        trace_path = analysis / 'global' / 'traces' / f'{scene_id}_f{args.query_accumulation_frames}.json'
        trace = json.loads(trace_path.read_text(encoding='utf-8'))
        classification = _classification(result_row)
        candidate_records = _candidate_rows(trace, block_index, block_manifest, dataset,
                                             args.candidate_top_k)
        # A selected Top-K candidate is marked only by reference-relative pose
        # error; the whole run remains explicitly non-independent evidence.
        selected_rank = (trace.get('selected') or {}).get('candidate_rank')
        if selected_rank is not None:
            for item in candidate_records:
                if item.get('rank') == selected_rank:
                    item['classification'] = classification

        visualization = analysis / 'visualization'
        visualization.mkdir()
        display_files = {}
        estimated = result_row.get('global_final_pose') or result_row.get('coarse_pose')
        if estimated:
            local_query = dataset.query_body(center, args.query_accumulation_frames).astype('f8')
            quat = [estimated[k] for k in ('qx', 'qy', 'qz', 'qw')]
            mapped_query = Rotation.from_quat(quat).apply(local_query) + np.asarray(
                [estimated[k] for k in ('x', 'y', 'z')], dtype='f8')
            query_overlay = visualization / 'query_at_estimated_pose_display_only.pcd'
            write_pcd(query_overlay, mapped_query.astype('<f4'))
            display_files['query_estimated_pose'] = str(query_overlay.relative_to(analysis))
        candidate_index = result_row.get('candidate_keyframe')
        if candidate_index is not None:
            candidate_cloud = dataset.patch_map(int(candidate_index)).astype('<f4')
            candidate_overlay = visualization / 'selected_candidate_map_frame_display_only.pcd'
            write_pcd(candidate_overlay, candidate_cloud)
            display_files['candidate_source_patch'] = str(candidate_overlay.relative_to(analysis))
        run_meta = {
            'job_state': 'SUCCEEDED' if result_row.get('global_backend_success') else 'FAILED',
            'result_row': result_row, 'requested_query_selection': selection,
            'window_keyframes': frame_ids, 'query_excluded_block_ids': excluded_blocks,
            'query_block_excluded_keyframes': extra_excluded_frame_ids,
            'query_display_files': display_files,
            'display_files_are_not_analysis_inputs': True,
            'analysis_input_semantics': 'existing native pipeline uses source body patches and poses; block assets provide exclusion and candidate-to-block association only',
        }
        (analysis / 'analysis_job.json').write_text(json.dumps(run_meta, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        source_manifest = source / 'manifest.yaml'
        block_manifest_path = block_root / 'manifest.yaml'
        query_pc = analysis / 'global' / 'queries' / f'{scene_id}_f{args.query_accumulation_frames}.pcd'
        trace_rel = trace_path.relative_to(analysis).as_posix()
        topk_ambiguity = {
            'score_semantics': 'ranked native descriptor/BBS candidate scores; not probability',
            'candidate_count': len(candidate_records),
            'candidate_top_k': args.candidate_top_k,
            'native_ambiguity_valid': result_row.get('ambiguity_valid'),
            'native_ambiguity_margin': result_row.get('ambiguity_margin'),
            'native_second_bbs_score': result_row.get('ambiguity_second_bbs_score'),
            'native_normalized_score_margin': result_row.get('score_margin_normalized'),
            'ranked_candidates': [{key: item.get(key) for key in (
                'rank', 'candidate_keyframe', 'descriptor_sector_similarity',
                'descriptor_ring_distance', 'bbs_score')} for item in candidate_records],
        }
        ambiguity_status = _ambiguity_status(candidate_records)
        empirical = {
            'classification': classification,
            'converged': result_row.get('global_backend_success'),
            'reference_relative_classification': classification,
            'reference_relative_only': True,
            'reference_error': {'xy_m': result_row.get('final_xy_error_m'),
                                'translation_3d_m': result_row.get('final_translation_3d_error_m'),
                                'yaw_deg': result_row.get('final_yaw_error_deg')},
            'native_failure_code': result_row.get('global_failure_code'),
            'wall_ms': result_row.get('global_wall_ms'),
            'raw_trace': f'analysis/{trace_rel}',
        }
        algorithm_config = {'global_settings': settings, 'query_accumulation_frames': args.query_accumulation_frames,
                            'candidate_top_k': args.candidate_top_k,
                            'block_keyframe_count': block_manifest['builder_config']['block_keyframe_count'],
                            'extra_excluded_query_block_keyframes': extra_excluded_frame_ids,
                            'native_binary_sha256': native.sha256,
                            'native_dependency_fingerprints': native_dependencies,
            'native_localizer_path': str(native.paths['candidate_bbs_gicp_localizer']),
            'native_localizer_trace_capable': True,
            'native_reference': 'agt_map_localization_benchmark greenhouse.run_global'}
        record = {
            'schema_version': 1, 'asset_type': 'agt.relocalization_evidence/v1',
            'evidence_id': evidence_id, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'job_state': run_meta['job_state'], 'source': {
                'identity': f"frontend_mapping_map_package:{dataset.metadata.get('mapping_backend', {}).get('id', 'UNKNOWN')}",
                'path_at_run': str(source), 'manifest_sha256': sha256(source_manifest),
                'checksums_sha256': sha256(source / 'checksums.sha256'),
                'pose_revision': sha256(source / 'poses_timed.txt'),
            },
            'block_set': {'block_set_id': block_manifest['block_set_id'], 'revision': block_manifest['revision'],
                          'parent_source_digest': block_manifest['parent_source_digest'],
                          'manifest_sha256': sha256(block_manifest_path),
                          'index_sha256': sha256(block_root / block_manifest['index_path']),
                          'checksums_sha256': sha256(block_root / 'checksums.sha256'),
                          'builder_config_digest': block_manifest['builder_config_digest'],
                          'row_annotation_sha256': block_annotation_digest,
                          'path_at_run': str(block_root),
                          'block_keyframe_count': block_manifest['builder_config']['block_keyframe_count'],
                          'query_excluded_block_ids': excluded_blocks,
                          'query_excluded_keyframes': extra_excluded_frame_ids,
                          'query_block_details': query_block_details},
            'query': {'query_id': scene_id, **selection, 'keyframe': _frame_id(dataset, center),
                      'timestamp': dataset.poses[center].stamp, 'window_keyframes': frame_ids,
                      'query_accumulation_frames': args.query_accumulation_frames,
                      'window_sha256': canonical_digest(window_material),
                      'query_cloud_sha256': sha256(query_pc),
                      'row_id': query_label['row_id'], 'along_row_s_m': query_label['along_row_s_m'],
                      'scene_type': query_label['scene_type'], 'row_annotation_sha256': topology_digest},
            'algorithm': {'name': 'GLOBAL descriptor prefilter + native 3D-BBS Top-K + native GICP',
                          'config': algorithm_config, 'config_digest': canonical_digest(algorithm_config),
                          'candidate_top_k': args.candidate_top_k,
                          'descriptor_prefilter': settings['descriptor_prefilter'],
                          'threads': settings['threads'], 'time_budget_s': args.time_budget_s,
                          'binary_sha256': native.sha256,
                          'dependency_fingerprints': native_dependencies,
            'binary_selection': {'candidate_localizer_path': str(native.paths['candidate_bbs_gicp_localizer']),
                                 'candidate_localizer_override': args.native_localizer_path is not None,
                                 'candidate_trace_supported': True},
            'code_provenance': git_provenance(repository),
                          'parameters_are_offline_diagnostics': True},
            'reference': {'type': dataset.reference_type, 'same_session': bool(dataset.reference.get('same_session')),
                          'independent_ground_truth': False,
                          'absolute_ground_truth': False,
                          'acceptance_semantics': 'reference-relative; no independent absolute ground truth'},
            'evidence': {
                'geometry_observability': {'status': 'NO_DATA', 'metrics': {},
                                           'note': 'not computed by this single-query GLOBAL operation'},
                'candidate_ambiguity': {'status': ambiguity_status, 'metrics': topk_ambiguity,
                                        'candidate_count': len(candidate_records),
                                        'candidates': candidate_records},
                'local_convergence_basin': {'status': 'NO_DATA', 'metrics': {},
                                            'note': 'GICP basin evaluator is reusable but not run in this query job'},
                'empirical_global_result': empirical,
            },
            'analysis_artifacts': {'job': 'analysis/analysis_job.json',
                                   'summary': 'analysis/global/global.csv',
                                   'raw_trace': f'analysis/{trace_rel}',
                                   'assets_manifest': 'analysis/global/assets_manifest.json',
                                   'query_cloud': f'analysis/{query_pc.relative_to(analysis).as_posix()}',
                                   'display_only': {key: f'analysis/{value}'
                                                    for key, value in display_files.items()}},
            'data_leakage_control': {'exact_query_window_excluded': True,
                                     'every_block_containing_query_constituent_excluded': True,
                                     'excluded_block_ids': excluded_blocks,
                                     'excluded_keyframes': extra_excluded_frame_ids},
            'provenance': {'block_validation': source_check,
                           'topology_validation': None if topology is None else 'PASS',
                           'asset_sha256': {}},
        }
        # Evidence only references small, high-value, directly inspectable
        # outputs. Native assets are reproducible from the frozen inputs/config;
        # their exact generated hashes are preserved in assets_manifest.json.
        selected_files = {}
        for path in (analysis / 'analysis_job.json', analysis / 'global' / 'global.csv',
                     analysis / 'global' / 'assets_manifest.json', trace_path, query_pc):
            if path.is_file():
                relative = path.relative_to(analysis).as_posix()
                selected_files[f'analysis/{relative}'] = path
        for path in sorted((analysis / 'visualization').glob('*.pcd')):
            selected_files[f'analysis/{path.relative_to(analysis).as_posix()}'] = path
        final_source_check = verify_keyframe_blocks(block_root, source_package=source)
        if final_source_check != source_check:
            raise ValueError('source or block asset revision changed during analysis; refusing to publish evidence')
        final_binary_hashes = {name: sha256(path) for name, path in native.paths.items()
                               if name in native.sha256 and path.is_file()}
        final_dependencies = native_dependency_fingerprints(args.ros_install, repository)
        if final_binary_hashes != native.sha256 or final_dependencies != native_dependencies:
            raise ValueError('native algorithm binary/dependency changed during analysis; refusing to publish evidence')
        if topology_path is not None and sha256(topology_path) != topology_digest:
            raise ValueError('topology annotation changed during analysis; refusing to publish evidence')
        record['provenance']['asset_sha256'] = {name: sha256(path) for name, path in selected_files.items()}
        published = publish_evidence_bundle(destination, record, selected_files,
                                            source_package=source, block_dir=block_root,
                                            topology_path=topology_path)
        print(json.dumps({'status': 'PASS', 'evidence_path': str(destination),
                          'evidence_id': evidence_id, 'classification': classification,
                          'candidate_count': len(candidate_records),
                          'verification': published}, indent=2), flush=True)
        return destination
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run one sparse offline MapStudio relocalization query and publish immutable evidence.')
    parser.add_argument('--map-package', type=Path, required=True)
    parser.add_argument('--block-dir', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    query = parser.add_mutually_exclusive_group(required=True)
    query.add_argument('--query-keyframe', type=int)
    query.add_argument('--map-x', dest='query_x', type=float)
    query.add_argument('--row-position', action='store_true',
                       help='use --row-id and --along-row-s-m from a frozen topology')
    parser.add_argument('--query-y', type=float)
    parser.add_argument('--row-id')
    parser.add_argument('--along-row-s-m', type=float)
    parser.add_argument('--topology', type=Path)
    parser.add_argument('--query-accumulation-frames', type=int, choices=(1, 3, 5), required=True)
    parser.add_argument('--candidate-top-k', type=int, required=True)
    parser.add_argument('--max-snap-distance-m', type=float, default=3.0)
    parser.add_argument('--time-budget-s', type=float, default=18.0)
    parser.add_argument('--ros-install', type=Path, default=Path.home() / 'ros2_ws/install')
    parser.add_argument('--native-localizer-path', type=Path,
                        help='explicit existing trace-capable native candidate localizer; no algorithm changes')
    return parser


def main(argv=None) -> int:
    parser = _parser()
    # argparse cannot express a paired map x/y option with a keyframe group;
    # parse a normalized argument list then enforce complete pair semantics.
    args = parser.parse_args(argv)
    if args.query_x is None and args.query_y is not None:
        parser.error('--query-y requires --map-x')
    if args.query_x is not None and args.query_y is None:
        parser.error('--map-x requires --query-y')
    if not 1 <= args.candidate_top_k <= 50 or args.time_budget_s <= 0:
        parser.error('candidate-top-k must be between 1 and 50 and time-budget-s positive')
    if args.max_snap_distance_m is not None and args.max_snap_distance_m < 0:
        parser.error('max-snap-distance-m must be nonnegative')
    if args.query_keyframe is not None and (args.query_x is not None or args.query_y is not None or args.row_position):
        parser.error('select exactly one query mode: keyframe, map point, or row position')
    if args.row_position and (not args.row_id or args.along_row_s_m is None or not args.topology):
        parser.error('--row-position requires --topology, --row-id and --along-row-s-m')
    if not args.row_position and (args.row_id is not None or args.along_row_s_m is not None):
        parser.error('--row-id/--along-row-s-m require --row-position')
    if args.row_position and (args.query_x is not None or args.query_y is not None):
        parser.error('row position may not be combined with map point selection')
    try:
        run_query(args)
        return 0
    except Exception as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc)}, indent=2), file=sys.stderr)
        return 2


def verify_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Verify a saved Relocalization Evidence bundle and optionally check staleness.')
    parser.add_argument('evidence_dir', type=Path)
    parser.add_argument('--map-package', type=Path)
    parser.add_argument('--block-dir', type=Path)
    parser.add_argument('--topology', type=Path)
    parser.add_argument('--ros-install', type=Path, default=Path.home() / 'ros2_ws/install')
    args = parser.parse_args(argv)
    try:
        result = verify_evidence_bundle(args.evidence_dir, source_package=args.map_package,
                                        block_dir=args.block_dir, topology_path=args.topology)
        record = yaml.safe_load((args.evidence_dir / 'manifest.yaml').read_text(encoding='utf-8'))
        algorithm_state, algorithm_reasons = current_algorithm_state(record, args.ros_install)
        result['algorithm_state'] = algorithm_state
        if algorithm_reasons:
            result['stale_reasons'] = list(result['stale_reasons']) + algorithm_reasons
            result['revision_state'] = 'STALE'
        print(json.dumps(result, indent=2))
        return 0
    except Exception as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc)}, indent=2), file=sys.stderr)
        return 2


def candidate_overlay_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Materialize one verified candidate patch in the source map frame for display only.')
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--map-package', type=Path, required=True)
    parser.add_argument('--block-dir', type=Path, required=True)
    parser.add_argument('--topology', type=Path)
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--output-pcd', type=Path, required=True)
    parser.add_argument('--ros-install', type=Path, default=Path.home() / 'ros2_ws/install')
    args = parser.parse_args(argv)
    try:
        check = verify_evidence_bundle(args.evidence_dir, source_package=args.map_package,
                                       block_dir=args.block_dir, topology_path=args.topology)
        record_path = args.evidence_dir / 'manifest.yaml'
        record = yaml.safe_load(record_path.read_text(encoding='utf-8'))
        algorithm_state, algorithm_reasons = current_algorithm_state(record, args.ros_install)
        if check.get('revision_state') != 'CURRENT' or algorithm_state != 'CURRENT':
            raise ValueError(f"evidence is stale: {check.get('stale_reasons', []) + algorithm_reasons}")
        if check.get('revision_state') != 'CURRENT':
            raise ValueError(f"evidence source is stale: {check.get('stale_reasons')}")
        candidates = record['evidence']['candidate_ambiguity'].get('candidates', [])
        selected = next((item for item in candidates if item.get('rank') == args.rank), None)
        if selected is None:
            raise ValueError(f'candidate rank not present in evidence: {args.rank}')
        dataset = MapPackage(args.map_package)
        patch_id = selected['candidate_patch_id']
        if patch_id not in dataset.patch_to_index:
            raise ValueError(f'candidate patch no longer exists in source: {patch_id}')
        patch_path = dataset.root / 'patches' / patch_id
        if sha256_file(patch_path) != selected.get('candidate_source_patch_sha256'):
            raise ValueError(f'candidate patch hash changed: {patch_id}')
        output = args.output_pcd.expanduser().absolute()
        output = require_output_outside(output, [args.map_package, args.block_dir, args.evidence_dir],
                                        label='candidate display output')
        if output.exists():
            raise FileExistsError(f'refusing to overwrite display overlay: {output}')
        output.parent.mkdir(parents=True, exist_ok=True)
        write_pcd(output, dataset.patch_map(dataset.patch_to_index[patch_id]).astype('<f4'))
        print(json.dumps({'status': 'PASS', 'display_only': True, 'rank': args.rank,
                          'candidate_patch_id': patch_id, 'candidate_keyframe': selected['candidate_keyframe'],
                          'output_pcd': str(output), 'output_sha256': sha256_file(output)}, indent=2))
        return 0
    except Exception as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc)}, indent=2), file=sys.stderr)
        return 2
