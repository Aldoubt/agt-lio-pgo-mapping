"""Phase 3B offline-only benchmark orchestration. No ROS node or publisher."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .backends import (GLOBAL_SETTINGS, LOCAL_SETTINGS, NativePrograms,
                       build_candidate_assets, dependency_fingerprints, global_command,
                       invoke, local_command)
from .dataset import MAX_MAP_PATCHES_PER_CENTER, PgoEvidence, fixed_split
from .geometry import alignment_and_recovery, query_geometry
from .metrics import (FAILURE_CODES, SUCCESS_RULES, classify_failure, initial_pose, measure_pose,
                      perturbations, pose_record, write_outputs)
from .pcd import sha256_file, write_pcd
from .quality_analysis import write_phase3c_outputs
from .quality_features import (NATIVE_MIN_MAP_POINTS, NATIVE_MIN_MAP_POINTS_SOURCE,
                               local_crop_support, localization_quality_features)
from .selection import (CONTROL_SEED, SWEEP_QUANTILES, make_candidates)
from .coverage_sampling import (COVERAGE_CELL_M, COVERAGE_FRACTIONS,
                                MIN_POINTS_PER_CELL, MIN_VOXELS_PER_CELL)
from .synthetic import run_synthetic


PARENT = Path('/home/yangxuan/ros2_ws/experiments/artifacts/output/'
              'live_mid360_20260922_143427_fixed_replay/map_package')
CONFIDENCE = Path('/home/yangxuan/ros2_ws/experiments/'
                  'agt_spatial_confidence_20260927/real_derivative')
GEOMETRY = Path('/home/yangxuan/ros2_ws/experiments/'
                'agt_spatial_geometry_phase3a_20260927/real_geometry_evidence')
REVIEWED = Path('/home/yangxuan/ros2_ws/experiments/'
                'agt_spatial_confidence_phase2b_20260927/studio_reviewed_agent_final')
ROS_INSTALL = Path('/home/yangxuan/ros2_ws/install')
EXPERIMENTS = Path('/home/yangxuan/ros2_ws/experiments/'
                   'agt_map_localization_phase3c_20260928/runs')
SOURCE = Path(__file__).resolve().parents[2]  # agt_mapping_framework/benchmarks


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Offline, non-publishing localization A/B; never overwrite an old run.')
    p.add_argument('--profile', choices=('smoke', 'standard'), required=True)
    p.add_argument('--run-id', required=True, help='unique output directory name, e.g. smoke_20260927_01')
    p.add_argument('--output-root', type=Path, default=EXPERIMENTS)
    p.add_argument('--map-package', type=Path, default=PARENT)
    p.add_argument('--confidence-source', type=Path, default=CONFIDENCE)
    p.add_argument('--geometry-source', type=Path, default=GEOMETRY)
    p.add_argument('--reviewed-source', type=Path, default=REVIEWED)
    p.add_argument('--ros-install', type=Path, default=ROS_INSTALL)
    p.add_argument('--global-backend', choices=('auto', 'off'), default='auto')
    p.add_argument('--smoke-run', type=Path, help='required STANDARD predecessor manifest directory')
    return p


def write_json(path: Path, obj: dict | list) -> None:
    path.write_text(json.dumps(obj, indent=2, allow_nan=False, sort_keys=True) + '\n',
                    encoding='utf-8')


def safe_run_id(value: str) -> str:
    if not value or value in ('.', '..') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in value):
        raise ValueError('run-id must contain only ASCII letters, digits, _ or -')
    return value


def _source_version(path: Path) -> dict:
    try:
        commit = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                                         text=True).strip()
        branch = subprocess.check_output(['git', '-C', str(path), 'branch', '--show-current'],
                                         text=True).strip()
        return {'commit': commit, 'branch': branch}
    except (subprocess.CalledProcessError, FileNotFoundError):
        return {'commit': None, 'branch': None}


def _benchmark_source_hashes() -> dict[str, str]:
    package = Path(__file__).resolve().parents[1]
    paths = sorted(package.glob('agt_map_localization_benchmark/*.py'))
    paths += sorted(package.glob('test/*.py'))
    paths += [package / 'README.md', package / 'package.xml', package / 'setup.py', package / 'setup.cfg']
    return {str(path.relative_to(package)): sha256_file(path) for path in paths}


def _algorithm_source_hashes() -> dict[str, str]:
    src = Path('/home/yangxuan/ros2_ws/src/agt_navigation_v3/navigation/localization')
    files = [
        src / 'agt_global_relocalization_native/src/map_gicp_tracker.cpp',
        src / 'agt_global_relocalization_native/src/candidate_bbs_gicp_localizer.cpp',
        src / 'agt_global_relocalization_native/src/build_relocalization_assets.cpp',
        src / 'agt_global_relocalization_native/src/build_relocalization_candidates.cpp',
        src / 'agt_global_relocalization_native/include/agt_global_relocalization_native/polar_context.hpp',
        src / 'agt_global_relocalization/config/global_relocalization.yaml',
    ]
    return {str(path): sha256_file(path) for path in files}


def _profile_plan(profile: str) -> dict:
    names = [p['name'] for p in perturbations()]
    if profile == 'smoke':
        return {'tier0': {'centers': [350], 'frames': [1],
                          'scenario_names': ['zero', 'x_2m', 'yaw_45deg']},
                'tier1': {'centers': [350], 'frames': [1, 3, 5],
                          'scenario_names': ['zero', 'x_1m', 'y_1m',
                                             'yaw_20deg', 'x_1m_yaw_20deg']},
                'global': {'tiers': ['tier1'], 'frames': [1], 'centers': [350]}}
    return {'tier1': {'centers': [175, 350], 'frames': [1, 3, 5],
                      'scenario_names': names},
            'global': {'tiers': ['tier1'], 'frames': [1, 5], 'centers': [175, 350]}}


def _check_standard_predecessor(args: argparse.Namespace, source_hashes: dict,
                                benchmark_hashes: dict, algorithm_hashes: dict,
                                native_hashes: dict, dependencies: dict) -> dict | None:
    if args.profile == 'smoke':
        return None
    if args.smoke_run is None:
        raise ValueError('STANDARD requires a completed same-code-and-data SMOKE manifest')
    file = args.smoke_run / 'manifest.json'
    smoke = json.loads(file.read_text())
    settings = smoke.get('parameters', {})
    if (smoke.get('status') != 'COMPLETED' or smoke.get('profile') != 'smoke'
            or smoke.get('source_sha256_before') != source_hashes
            or smoke.get('source_sha256_after') != source_hashes
            or smoke.get('benchmark_source_sha256_after') != benchmark_hashes
            or smoke.get('algorithm_source_sha256') != algorithm_hashes
            or smoke.get('native_executable_sha256') != native_hashes
            or smoke.get('linked_registration_dependencies_after') != dependencies
            or settings.get('local_native') != LOCAL_SETTINGS
            or settings.get('global_native') != GLOBAL_SETTINGS
            or settings.get('success_criteria_exploratory') != SUCCESS_RULES
            or settings.get('qt_quantile_sweep_predeclared') != list(SWEEP_QUANTILES)
            or settings.get('control_seed') != CONTROL_SEED
            or settings.get('coverage_cell_m') != COVERAGE_CELL_M
            or settings.get('coverage_fractions_predeclared') != list(COVERAGE_FRACTIONS)
            or settings.get('coverage_min_points_per_cell') != MIN_POINTS_PER_CELL
            or settings.get('coverage_min_voxels_per_cell') != MIN_VOXELS_PER_CELL
            or smoke.get('synthetic', {}).get('local_cases', 0) < 15
            or smoke.get('real_trial_count', 0) < 1):
        raise ValueError('SMOKE predecessor incomplete or inputs/code/algorithms/settings differ')
    return {'manifest': str(file), 'manifest_sha256': sha256_file(file)}


def _candidate_record(c) -> dict:
    return {'name': c.name, 'path': str(c.path), 'detail': c.detail, 'coverage': c.coverage}


def _run_real(run: Path, data: PgoEvidence, native: NativePrograms,
              plan: dict, manifest: dict, *, use_global: bool) -> tuple[list[dict], dict, list[dict]]:
    base = run / 'real'
    base.mkdir()
    results = run / 'results.jsonl'
    rows: list[dict] = []
    candidates: dict = {}
    scenario_map = {p['name']: p for p in perturbations()}
    prepared = {}
    coverage_rows: list[dict] = []
    for tier, settings in plan.items():
        if tier == 'global':
            continue
        stage = base / tier
        stage.mkdir()
        centers = tuple(settings['centers'])
        split = fixed_split(data.poses, centers, tier, radius=30.0)
        # For every Tier1 query window, all source patches are held out.
        for c in centers:
            for f in settings['frames']:
                frame_ids = set(range(c - f // 2, c + f // 2 + 1))
                if tier == 'tier1' and frame_ids.intersection(split.map_indices):
                    raise AssertionError('Tier1 query frames overlap candidate map subset')
        selected, raw = make_candidates(data, split.map_indices, stage / 'candidates')
        coords, lookup, found = raw['points'], raw['lookups'], raw['found']
        baseline_b = raw['baseline_B_indices']
        for entry in raw['coverage_field']:
            coverage_rows.append({'tier': tier, **entry})
        candidates[tier] = {item.name: _candidate_record(item) for item in selected}
        manifest['splits'][tier] = {
            'map_keyframes': list(split.map_indices), 'heldout_keyframes': list(split.heldout_indices),
            'keyframes_outside_map_subset_not_all_executed': list(split.all_query_indices),
            'reserved_query_window_keyframes': list(split.heldout_indices),
            'actual_executed_query_keyframes': sorted({i for c in centers for f in settings['frames']
                                                       for i in range(c - f // 2, c + f // 2 + 1)}),
            'centers': list(centers), 'radius_xy_m': split.max_map_radius_xy_m,
            'max_nearest_map_patches_per_center': MAX_MAP_PATCHES_PER_CENTER,
            'oracle_reference_centered_roi_evaluation_only': True,
            'label': 'SELF_QUERY/DATA_LEAKAGE_EXPECTED' if tier == 'tier0' else
                     'SINGLE_SESSION_PGO_AND_FULL_SESSION_EVIDENCE_LABEL_LEAKAGE',
            'pose_graph_leakage_possible': tier == 'tier1',
            'full_session_evidence_label_leakage_possible': True,
            'candidate_selection': raw['meta'],
        }
        manifest['candidates'][tier] = candidates[tier]
        write_json(stage / 'split.json', manifest['splits'][tier])
        write_json(stage / 'candidates.json', candidates[tier])
        query_dir = stage / 'queries'
        query_dir.mkdir()
        queries = {}
        for center in centers:
            ref = data.poses[center]
            for frames in settings['frames']:
                qid = f'kf{center:03d}_f{frames}'
                path = query_dir / f'{qid}.pcd'
                body = data.query_body(center, frames)
                write_pcd(path, body)
                queries[qid] = {
                    'path': str(path), 'sha256': sha256_file(path), 'points_before_native_voxel': len(body),
                    'source_patch_indices': list(range(center - frames // 2, center + frames // 2 + 1)),
                    'reference_pose': pose_record(ref.t, ref.quat_xyzw),
                    'reference_label': 'optimized_PGO_pose',
                    'map_subset_disjoint': not bool(set(range(center - frames // 2, center + frames // 2 + 1))
                                                    & set(split.map_indices)),
                }
        write_json(stage / 'queries.json', queries)
        prepared[tier] = (stage, split, selected, coords, lookup, found, baseline_b, queries)
        print(f'REAL PREP {tier}: map keyframes={len(split.map_indices)} '
              f'candidates={len(selected)} queries={len(queries)}', flush=True)
    write_json(run / 'manifest.json', manifest)

    def append(row: dict) -> None:
        rows.append(row)
        with results.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, allow_nan=False, sort_keys=True) + '\n')

    # LOCAL is completed BEFORE trying GLOBAL. One unchanged native program and
    # identical query/seed/parameters per candidate within each case.
    for tier, (stage, split, selected, coords, lookup, found, baseline_b, queries) in prepared.items():
        settings = plan[tier]
        for center in settings['centers']:
            ref = data.poses[center]
            ref_pose = pose_record(ref.t, ref.quat_xyzw)
            for frames in settings['frames']:
                query = queries[f'kf{center:03d}_f{frames}']['path']
                for candidate in selected:
                    geom = query_geometry(data, coords, lookup, found, candidate.indices, ref.t,
                                          radius_xy=LOCAL_SETTINGS['radius_xy_m'],
                                          half_height=LOCAL_SETTINGS['half_height_m'])
                    support = local_crop_support(
                        candidate.indices, baseline_b, coords, lookup, found, data.geom, ref.t,
                        radius_xy=LOCAL_SETTINGS['radius_xy_m'],
                        half_height=LOCAL_SETTINGS['half_height_m'])
                    eps = data.geom_meta['parameters']['epsilon']
                    quality = localization_quality_features(
                        geom, support, translation_epsilon=float(eps['translation']),
                        rotation_epsilon_m2=float(eps['rotation_m2']))
                    for name in settings['scenario_names']:
                        scenario = scenario_map[name]
                        t, quat = initial_pose(ref.t, ref.quat_xyzw, scenario)
                        native_result = invoke(local_command(native.paths['map_gicp_tracker'],
                                                            candidate.path, Path(query), t, quat),
                                               timeout=35)
                        pose = native_result['pose']
                        final_t = np.array([pose[k] for k in ('x', 'y', 'z')]) if pose else None
                        recovery = alignment_and_recovery(
                            ref.t, t, final_t, geom, scenario['dyaw_deg'],
                            reference_xyzw=ref.quat_xyzw,
                            result_xyzw=np.array([pose[k] for k in ('qx', 'qy', 'qz', 'qw')])
                            if pose else None)
                        row = {'tier': tier, 'algorithm': 'LOCAL', 'candidate': candidate.name,
                               'center': center, 'frames': frames, 'scenario': name,
                               'reference_label': 'optimized_PGO_pose',
                               'leakage_label': manifest['splits'][tier]['label'],
                               'query_pcd': query, 'candidate_pcd': str(candidate.path),
                               'reference_pose': ref_pose, 'initial_pose': pose_record(t, quat),
                               'perturbation': scenario, 'native': native_result,
                               'geometry': geom, 'local_support': support,
                               'quality_features': quality, 'weak_alignment': recovery,
                               **measure_pose(ref_pose, native_result)}
                        row['failure_code'] = classify_failure(row)
                        append(row)
                    if len(rows) % 100 < len(settings['scenario_names']):
                        print(f'LOCAL {tier}: {len(rows)} trials; {candidate.name} '
                              f'kf={center} frames={frames}', flush=True)
    manifest['local_trial_count'] = len(rows)
    print(f'LOCAL COMPLETED: {len(rows)} native trials', flush=True)
    write_json(run / 'manifest.json', manifest)

    if use_global:
        for tier in plan['global']['tiers']:
            stage, split, selected, coords, lookup, found, baseline_b, queries = prepared[tier]
            assets_root = stage / 'assets'
            assets_root.mkdir()
            for candidate in selected:
                record = build_candidate_assets(native, data, split.map_indices, candidate,
                                                coords, assets_root / candidate.name)
                manifest['global_assets'][f'{tier}/{candidate.name}'] = record
                write_json(run / 'manifest.json', manifest)
                if record['status'] != 'READY':
                    manifest['not_run'][f'GLOBAL/{tier}/{candidate.name}'] = record['reason']
                    continue
                asset_dir = Path(record['assets_dir'])
                for center in plan['global']['centers']:
                    ref = data.poses[center]
                    ref_pose = pose_record(ref.t, ref.quat_xyzw)
                    geom = query_geometry(data, coords, lookup, found, candidate.indices, ref.t)
                    support = local_crop_support(
                        candidate.indices, baseline_b, coords, lookup, found, data.geom, ref.t,
                        radius_xy=LOCAL_SETTINGS['radius_xy_m'],
                        half_height=LOCAL_SETTINGS['half_height_m'])
                    eps = data.geom_meta['parameters']['epsilon']
                    quality = localization_quality_features(
                        geom, support, translation_epsilon=float(eps['translation']),
                        rotation_epsilon_m2=float(eps['rotation_m2']))
                    for frames in plan['global']['frames']:
                        query = queries[f'kf{center:03d}_f{frames}']['path']
                        native_result = invoke(global_command(native.paths['candidate_bbs_gicp_localizer'],
                                                             candidate.path, Path(query), asset_dir),
                                               timeout=25)
                        row = {'tier': tier, 'algorithm': 'GLOBAL', 'candidate': candidate.name,
                               'center': center, 'frames': frames, 'scenario': 'NO_INITIAL_POSE',
                               'reference_label': 'optimized_PGO_pose',
                               'leakage_label': manifest['splits'][tier]['label'],
                               'query_pcd': query, 'candidate_pcd': str(candidate.path),
                               'reference_pose': ref_pose, 'initial_pose': None,
                               'perturbation': None, 'native': native_result, 'geometry': geom,
                               'local_support': support, 'quality_features': quality,
                               'weak_alignment': alignment_and_recovery(
                                   ref.t, ref.t, np.array([native_result['pose'][k] for k in ('x', 'y', 'z')])
                                   if native_result['pose'] else None, geom, 0),
                               **measure_pose(ref_pose, native_result)}
                        row['failure_code'] = classify_failure(row)
                        append(row)
                print(f'GLOBAL {tier}/{candidate.name}: {len(rows)} accumulated trials', flush=True)
    else:
        manifest['not_run']['GLOBAL'] = 'native descriptor/BBS programs absent or explicitly disabled'
    manifest['global_trial_count'] = sum(row['algorithm'] == 'GLOBAL' for row in rows)
    write_json(run / 'manifest.json', manifest)
    return rows, candidates, coverage_rows


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    safe_run_id(args.run_id)
    if not args.map_package.is_dir() or not args.geometry_source.is_dir():
        raise SystemExit('missing verified PGO/geometry source; no run created')
    native = NativePrograms.discover(args.ros_install)
    dependencies = dependency_fingerprints(args.ros_install, Path(__file__).resolve().parents[3].parent)
    if not native.local_ready:
        raise SystemExit('runtime map_gicp_tracker is absent; LOCAL cannot use the actual implementation')
    data = PgoEvidence(args.map_package, args.confidence_source, args.geometry_source,
                       args.reviewed_source if args.reviewed_source.is_dir() else None)
    digests = data.validate_sources()
    code_hashes = _benchmark_source_hashes()
    algorithm_hashes = _algorithm_source_hashes()
    predecessor = _check_standard_predecessor(
        args, digests, code_hashes, algorithm_hashes, native.sha256, dependencies)
    use_global = args.global_backend != 'off' and native.global_ready
    root = args.output_root.resolve()
    # This tool is never allowed to write to an input, source checkout,
    # installed runtime, or arbitrary production/active map. Even a mistakenly
    # supplied --output-root must remain in the dedicated experiment tree.
    experiment_root = EXPERIMENTS.parent.resolve()
    if root.parent != experiment_root:
        raise SystemExit('--output-root must be a direct child of the dedicated Phase 3B experiment root')
    root.mkdir(parents=True, exist_ok=True)
    run = root / args.run_id
    if run.exists():
        raise SystemExit(f'old experiment exists, refusing overwrite: {run}')
    run.mkdir(mode=0o755)
    plan = _profile_plan(args.profile)
    manifest = {
        'schema_version': 3, 'status': 'IN_PROGRESS',
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'run_id': args.run_id, 'profile': args.profile,
        'offline_only': True, 'publish_production_map': False,
        'mapping_source_git': _source_version(Path(__file__).resolve().parents[3]),
        'navigation_source_git': _source_version(Path('/home/yangxuan/ros2_ws/src/agt_navigation_v3')),
        'native_executable_sha256': native.sha256,
        'linked_registration_dependencies_before': dependencies,
        'linked_registration_dependencies_after': None,
        'algorithm_source_sha256': algorithm_hashes,
        'benchmark_source_sha256_before': code_hashes,
        'benchmark_source_sha256_after': None,
        'input_paths': {'pgo': str(data.pgo), 'v1': str(data.confidence),
                        'geometry': str(data.geometry), 'reviewed': str(data.reviewed) if data.reviewed else None},
        'source_sha256_before': digests, 'source_sha256_after': None,
        'smoke_predecessor': predecessor,
        'parameters': {
            'local_native': LOCAL_SETTINGS, 'global_native': GLOBAL_SETTINGS,
            'success_criteria_exploratory': SUCCESS_RULES,
            'case_failure_codes': FAILURE_CODES,
            'qt_quantile_sweep_predeclared': SWEEP_QUANTILES,
            'control_seed': CONTROL_SEED,
            'coverage_cell_m': COVERAGE_CELL_M,
            'coverage_fractions_predeclared': list(COVERAGE_FRACTIONS),
            'coverage_min_points_per_cell': MIN_POINTS_PER_CELL,
            'coverage_min_voxels_per_cell': MIN_VOXELS_PER_CELL,
            'native_min_map_points': NATIVE_MIN_MAP_POINTS,
            'native_min_map_points_source': NATIVE_MIN_MAP_POINTS_SOURCE,
            'map_subset_rule': 'within 30m of fixed optimized PGO centers, i%3!=2; '
                               'nearest 96 patches per center; Tier1 holds out every ±2 query patch; '
                               'oracle ROI bias, offline resource control only',
            'candidate_selection': 'A raw, B V1 stable, C reviewed stable, legacy global Qt negative '
                                   'control, plus B-derived coverage-preserving Qt/random/uniform '
                                   'samplers with the same 1m XY cell set and exact per-cell quota',
            'reference_frame': 'optimized_PGO_pose T_map_body, not absolute ground truth',
            'query_accumulation': 'inverse(T_map_body_ref)*T_map_body_i applied to body patch i',
        },
        'plan': plan, 'synthetic': None, 'splits': {}, 'candidates': {},
        'global_assets': {}, 'local_trial_count': 0, 'global_trial_count': 0,
        'not_run': {
            'Tier2': 'no verified same-place independent-session inputs',
            'FULL': 'profile not supported; explicit user approval required',
            'online_robot_nav_tf': 'offline CLI only, no ROS node / map->odom / Guardian / robot',
            'negative_session_false_relocation_rate': 'no verified independent negative-session query',
            'confidence_v2_or_product_threshold': 'forbidden in Phase 3C; evidence remains experimental',
        },
    }
    if data.rev is None:
        manifest['not_run']['C_REVIEWED_STABLE'] = 'verified reviewed derivative unavailable'
    write_json(run / 'manifest.json', manifest)
    try:
        if args.profile == 'smoke':
            manifest['synthetic'] = run_synthetic(native, run / 'synthetic')
            write_json(run / 'manifest.json', manifest)
        rows, candidate_meta, coverage_rows = _run_real(run, data, native, plan, manifest,
                                                        use_global=use_global)
        # Source integrity is verified *again* after every offline native call.
        final_digests = data.validate_sources()
        if final_digests != digests:
            raise RuntimeError('input source bytes changed during benchmark; discard this run')
        if (_algorithm_source_hashes() != manifest['algorithm_source_sha256']
                or native.sha256 != NativePrograms.discover(args.ros_install).sha256
                or dependencies != dependency_fingerprints(
                    args.ros_install, Path(__file__).resolve().parents[3].parent)):
            raise RuntimeError('native algorithm sources/binaries/linked dependencies changed during benchmark')
        manifest['linked_registration_dependencies_after'] = dependencies
        code_after = _benchmark_source_hashes()
        if code_after != manifest['benchmark_source_sha256_before']:
            raise RuntimeError('benchmark source code changed during benchmark; discard this run')
        manifest['benchmark_source_sha256_after'] = code_after
        manifest['source_sha256_after'] = final_digests
        manifest['real_trial_count'] = len(rows)
        manifest['status'] = 'COMPLETED'
        write_outputs(run, rows, candidate_meta, manifest)
        phase3c = write_phase3c_outputs(run, rows, coverage_rows, candidate_meta)
        manifest['phase3c_analysis'] = {
            'summary': 'coverage_geometry_summary.json',
            'report': 'coverage_geometry_report.md',
            'coverage_field': 'coverage_field.csv',
            'semantics': phase3c['semantics'],
        }
        write_json(run / 'manifest.json', manifest)
        print(f'BENCHMARK COMPLETED: {run} ({len(rows)} real registrations)', flush=True)
        return 0
    except BaseException as exc:
        manifest['status'] = 'INCOMPLETE'
        manifest['failure'] = f'{type(exc).__name__}: {exc}'
        write_json(run / 'manifest.json', manifest)
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
