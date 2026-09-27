"""Re-evaluate immutable native trial records using corrected Phase 3B metrics.

This command NEVER reruns a native registration. Use it only when native binary,
inputs, algorithms, maps, queries and raw trial records are source-locked. The
output is an independent *postprocess*, never a rewritten original experiment.
"""
from __future__ import annotations

import argparse
import json
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .backends import NativePrograms, dependency_fingerprints
from .cli import (CONFIDENCE, EXPERIMENTS, GEOMETRY, PARENT, REVIEWED, ROS_INSTALL,
                  _algorithm_source_hashes, _benchmark_source_hashes,
                  _source_version, write_json)
from .dataset import PgoEvidence, lookup_sorted_keys, point_keys
from .geometry import alignment_and_recovery, query_geometry
from .metrics import FAILURE_CODES, SUCCESS_RULES, classify_failure, measure_pose, pose_record, write_outputs
from .pcd import read_pcd, sha256_file, xyz
from .selection import COVERAGE_CELL_M, spatial_coverage, xy_cells


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Source-locked POSTPROCESS of completed native trials; no native rerun')
    p.add_argument('--source-run', required=True, type=Path)
    p.add_argument('--output-dir', required=True, type=Path,
                   help='fresh, isolated analysis directory; never an existing native run')
    p.add_argument('--map-package', type=Path, default=PARENT)
    p.add_argument('--confidence-source', type=Path, default=CONFIDENCE)
    p.add_argument('--geometry-source', type=Path, default=GEOMETRY)
    p.add_argument('--reviewed-source', type=Path, default=REVIEWED)
    p.add_argument('--ros-install', type=Path, default=ROS_INSTALL)
    return p


def _source_file_hashes(run: Path, manifest: dict) -> dict[str, str]:
    """Every query and candidate PCD from the prior run is verified in place."""
    result = {}
    for tier, entries in manifest['candidates'].items():
        for name, record in entries.items():
            path = Path(record['path']).resolve(strict=True)
            if not path.is_relative_to(run):
                raise ValueError('old candidate PCD is outside the frozen source run')
            digest = sha256_file(path)
            if digest != record['coverage']['pcd_sha256']:
                raise ValueError(f'original candidate bytes changed: {tier}/{name}')
            result[str(path.relative_to(run))] = digest
        query_file = run / 'real' / tier / 'queries.json'
        result[str(query_file.relative_to(run))] = sha256_file(query_file)
        for name, record in json.loads(query_file.read_text()).items():
            path = Path(record['path']).resolve(strict=True)
            if not path.is_relative_to(run):
                raise ValueError('old query PCD is outside the frozen source run')
            digest = sha256_file(path)
            if digest != record['sha256']:
                raise ValueError(f'original query bytes changed: {tier}/{name}')
            result[str(path.relative_to(run))] = digest
    for label, record in manifest.get('global_assets', {}).items():
        if record['status'] != 'READY':
            continue
        asset_dir = Path(record['assets_dir']).resolve(strict=True)
        if not asset_dir.is_relative_to(run):
            raise ValueError('original GLOBAL asset directory is outside frozen source run')
        db = asset_dir / 'polar_context.db'
        db_digest = sha256_file(db)
        if db_digest != record['descriptor_database_sha256']:
            raise ValueError(f'original descriptor database changed: {label}')
        result[str(db.relative_to(run))] = db_digest
        # Baseline a82495a recorded the descriptor DB, but not every coarse
        # asset. Future native runs record asset_files_sha256 for each builder
        # output; only those can be retrospectively verified byte-for-byte.
        for filename, expected in record.get('asset_files_sha256', {}).items():
            path = (asset_dir / filename).resolve(strict=True)
            if not path.is_relative_to(asset_dir) or sha256_file(path) != expected:
                raise ValueError(f'original BBS/descriptor asset changed: {label}/{filename}')
            result[str(path.relative_to(run))] = expected
    return result


def _pose_matches(reference: dict, actual: dict) -> bool:
    if not np.allclose([reference[k] for k in ('x', 'y', 'z')],
                       [actual[k] for k in ('x', 'y', 'z')], atol=1e-6, rtol=0):
        return False
    q1 = Rotation.from_quat([reference[k] for k in ('qx', 'qy', 'qz', 'qw')])
    q2 = Rotation.from_quat([actual[k] for k in ('qx', 'qy', 'qz', 'qw')])
    return (q1 * q2.inv()).magnitude() < 1e-6


def _recompute(run: Path, original: dict, data: PgoEvidence) -> tuple[list[dict], dict]:
    raw_rows = [json.loads(line) for line in (run / 'results.jsonl').read_text().splitlines()]
    if len(raw_rows) != original['real_trial_count'] or len(raw_rows) < 1:
        raise ValueError('native JSONL row count does not match completed manifest')
    cache: dict = {}
    candidates = {}
    for tier, records in original['candidates'].items():
        if tier not in ('tier0', 'tier1'):
            raise ValueError('unknown leakage tier in original run')
        raw_file = Path(records['A_RAW']['path'])
        raw_coords = xyz(read_pcd(raw_file, ('x', 'y', 'z')))
        xy_ref = set(map(tuple, xy_cells(raw_coords)))
        xyz_ref = set(map(tuple, np.floor(raw_coords / np.float32(COVERAGE_CELL_M)).astype('<i8')))
        candidates[tier] = {}
        for name, record in records.items():
            pts = xyz(read_pcd(Path(record['path']), ('x', 'y', 'z')))
            coverage = spatial_coverage(pts, xy_ref, xyz_ref)
            for key in ('points', 'occupied_xy_1m_cells', 'fraction_of_raw_xy_1m_cells'):
                if not np.isclose(coverage[key], record['coverage'][key], rtol=0, atol=1e-9):
                    raise ValueError(f'candidate spatial evidence changed: {tier}/{name}/{key}')
            coverage.update({'pcd_sha256': record['coverage']['pcd_sha256'],
                             'source_map_keyframe_count': record['coverage']['source_map_keyframe_count']})
            candidates[tier][name] = {'name': name, 'path': record['path'],
                                      'detail': record['detail'], 'coverage': coverage}
            lookup, found = lookup_sorted_keys(data.keys, point_keys(pts, data.voxel_size))
            cache[tier, name] = (pts, lookup, found)
    geometry = {}
    out = []
    query_files = {tier: json.loads((run / 'real' / tier / 'queries.json').read_text())
                   for tier in candidates}
    for row in raw_rows:
        tier, name, center, frames = (row['tier'], row['candidate'], row['center'], row['frames'])
        ref = data.poses[center]
        reference = pose_record(ref.t, ref.quat_xyzw)
        if (not _pose_matches(row['reference_pose'], reference) or
                row['reference_label'] != 'optimized_PGO_pose'):
            raise ValueError('native row reference differs from locked PGO optimized pose')
        query = query_files[tier][f'kf{center:03d}_f{frames}']
        if row['query_pcd'] != query['path'] or row['candidate_pcd'] != candidates[tier][name]['path']:
            raise ValueError('native row points to a different map or query PCD')
        if tier == 'tier1' and set(query['source_patch_indices']) & set(original['splits'][tier]['map_keyframes']):
            raise ValueError('Tier1 query patch entered original candidate map subset')
        key = (tier, name, center)
        if key not in geometry:
            points, lookup, found = cache[tier, name]
            geometry[key] = query_geometry(data, points, lookup, found,
                                            np.arange(len(points)), ref.t)
        g = geometry[key]
        for old, new in (('qt_mapping_view_median', 'qt_mapping_view_median'),
                         ('qr_mapping_view_median', 'qr_mapping_view_median')):
            previous, recomputed = row['geometry'].get(old), g[new]
            if previous is not None and not np.isclose(previous, recomputed, rtol=0, atol=1e-6):
                raise ValueError(f'original sidecar Q differs under replay: {tier}/{name}/{old}')
        for key_q in ('qt_query_local', 'qr_query_conditioned'):
            previous, recomputed = row['geometry'][key_q]['q'], g[key_q]['q']
            if previous is not None and recomputed is not None and not np.isclose(
                    previous, recomputed, rtol=0, atol=1e-6):
                raise ValueError(f'original query Q differs under replay: {tier}/{name}/{key_q}')
        native = row['native']
        pose = native.get('pose')
        result_t = np.array([pose[k] for k in ('x', 'y', 'z')]) if pose else None
        result_q = np.array([pose[k] for k in ('qx', 'qy', 'qz', 'qw')]) if pose else None
        if row['algorithm'] == 'LOCAL':
            if not row['initial_pose'] or not row['perturbation']:
                raise ValueError('local row missing predeclared initial pose')
            init = np.array([row['initial_pose'][k] for k in ('x', 'y', 'z')])
            yaw = row['perturbation']['dyaw_deg']
        elif row['algorithm'] == 'GLOBAL':
            init, yaw = ref.t, 0.
        else:
            raise ValueError('unrecognized native backend')
        row['geometry'] = g
        row['weak_alignment'] = alignment_and_recovery(
            ref.t, init, result_t, g, yaw, reference_xyzw=ref.quat_xyzw,
            result_xyzw=result_q)
        row.update(measure_pose(reference, native))
        row['failure_code'] = classify_failure(row)
        out.append(row)
    return out, candidates


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    run = args.source_run.resolve(strict=True)
    out = args.output_dir.resolve()
    if out.exists() or out.is_relative_to(run):
        raise SystemExit('postprocess output must be NEW and outside the original run')
    if out.parent != EXPERIMENTS.parent / 'reanalysis':
        raise SystemExit('postprocess output must be a fresh Phase 3B reanalysis subdirectory')
    data = PgoEvidence(args.map_package, args.confidence_source, args.geometry_source,
                       args.reviewed_source if args.reviewed_source.is_dir() else None)
    for protected in (data.pgo, data.confidence, data.geometry, data.reviewed,
                      Path(__file__).resolve().parents[3], args.ros_install.resolve()):
        if protected is not None and out.is_relative_to(protected):
            raise SystemExit('postprocess output inside a protected input/source tree')
    original = json.loads((run / 'manifest.json').read_text())
    hashes_before = data.validate_sources()
    binary = NativePrograms.discover(args.ros_install)
    dependencies = dependency_fingerprints(args.ros_install, Path(__file__).resolve().parents[3].parent)
    algorithms = _algorithm_source_hashes()
    prior_dependencies = original.get('linked_registration_dependencies_before')
    if prior_dependencies is not None and (prior_dependencies != dependencies or
                                            original.get('linked_registration_dependencies_after') != dependencies):
        raise SystemExit('original run and current linked registration dependencies differ')
    if (original['status'] != 'COMPLETED' or original['source_sha256_before'] != hashes_before
            or original['source_sha256_after'] != hashes_before
            or original['native_executable_sha256'] != binary.sha256
            or original['algorithm_source_sha256'] != algorithms
            or original['benchmark_source_sha256_before'] != original['benchmark_source_sha256_after']):
        raise SystemExit('source-lock mismatch: cannot relabel native results from this run')
    prior_files = _source_file_hashes(run, original)
    old_manifest_sha = sha256_file(run / 'manifest.json')
    old_results_sha = sha256_file(run / 'results.jsonl')
    code_hashes = _benchmark_source_hashes()
    out.mkdir(parents=True, exist_ok=False)
    manifest = {
        'schema_version': 2, 'status': 'IN_PROGRESS', 'kind': 'POSTPROCESS_NO_NATIVE_RERUN',
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_native_run': str(run),
        'source_native_manifest_sha256': old_manifest_sha,
        'source_native_results_jsonl_sha256': old_results_sha,
        'source_candidate_query_sha256_before': prior_files,
        'source_candidate_query_sha256_after': None,
        'source_sha256_before': hashes_before, 'source_sha256_after': None,
        'native_executable_sha256': binary.sha256,
        'linked_registration_dependencies_current': dependencies,
        'linked_registration_dependencies_original_recorded': prior_dependencies is not None,
        'original_full_bbs_asset_hashes_recorded': all(
            'asset_files_sha256' in asset for asset in original.get('global_assets', {}).values()
            if asset['status'] == 'READY'),
        'algorithm_source_sha256': algorithms,
        'benchmark_source_sha256_before': code_hashes, 'benchmark_source_sha256_after': None,
        'mapping_source_git': _source_version(Path(__file__).resolve().parents[3]),
        'navigation_source_git': _source_version(Path('/home/yangxuan/ros2_ws/src/agt_navigation_v3')),
        'parameters': dict(original['parameters'], success_criteria_exploratory=SUCCESS_RULES,
                           case_failure_codes=FAILURE_CODES),
        'splits': original['splits'], 'candidates': None, 'not_run': original['not_run'],
        'native_trial_count': original['real_trial_count'], 'new_native_trials': 0,
        'original_synthetic': original.get('synthetic'),
        'note': 'reruns ONLY metric/weak-axis analysis on unchanged raw native JSON; '
                'not a new localization trial or independent validation',
        'original_global_dependency_limit': (
            'If original_full_bbs_asset_hashes_recorded is false, only the '
            'originally hashed descriptor DB can be retrospectively byte-verified. '
            'If linked_registration_dependencies_original_recorded is false, '
            'current library hashes are descriptive, not retrospective proof.'),
    }
    for tier, split in manifest['splits'].items():
        split['pose_graph_leakage_possible'] = tier == 'tier1'
        split['full_session_evidence_label_leakage_possible'] = True
    write_json(out / 'manifest.json', manifest)
    try:
        rows, candidates = _recompute(run, original, data)
        manifest['candidates'] = candidates
        with (out / 'results.jsonl').open('x', encoding='utf-8') as f:
            for row in rows:
                f.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')
        write_outputs(out, rows, candidates, manifest)
        prefix = ('**POSTPROCESS ONLY:** this file recalculates metrics from previously '
                  f'completed native output in `{run}`. **Zero new registrations** were '
                  'performed. The original run and its source hashes remain available; '
                  'new success thresholds cannot be mistaken for its original analysis.\n\n')
        report = out / 'report.md'
        report.write_text(report.read_text().replace('\n\n', '\n\n' + prefix, 1),
                          encoding='utf-8')
        if (data.validate_sources() != hashes_before
                or _algorithm_source_hashes() != algorithms
                or NativePrograms.discover(args.ros_install).sha256 != binary.sha256
                or dependency_fingerprints(args.ros_install, Path(__file__).resolve().parents[3].parent)
                   != dependencies
                or _source_file_hashes(run, original) != prior_files
                or sha256_file(run / 'manifest.json') != old_manifest_sha
                or sha256_file(run / 'results.jsonl') != old_results_sha
                or _benchmark_source_hashes() != code_hashes):
            raise ValueError('source bytes changed DURING postprocess; discard analysis')
        manifest['source_sha256_after'] = hashes_before
        manifest['source_candidate_query_sha256_after'] = prior_files
        manifest['benchmark_source_sha256_after'] = code_hashes
        manifest['candidates'] = candidates
        manifest['status'] = 'COMPLETED'
        write_json(out / 'manifest.json', manifest)
        print(f'POSTPROCESS COMPLETED: {out} ({len(rows)} previously completed native rows)')
        return 0
    except BaseException as exc:
        manifest['status'] = 'INCOMPLETE'
        manifest['failure'] = f'{type(exc).__name__}: {exc}'
        write_json(out / 'manifest.json', manifest)
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
