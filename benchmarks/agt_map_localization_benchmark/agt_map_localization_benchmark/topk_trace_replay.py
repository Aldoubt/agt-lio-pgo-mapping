"""Replay held-out GLOBAL queries through the authoritative native localizer.

This tool reuses a completed greenhouse benchmark's maps, descriptor/BBS assets
and query PCDs. It implements no retrieval, registration or mapping algorithm.
Input bytes are frozen before execution and verified afterwards; raw native
traces and enriched offline-reference traces are saved separately in a new run.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import shutil

from .backends import GLOBAL_SETTINGS, enrich_candidate_trace, global_command, invoke
from .greenhouse import MapPackage, _safe_id, load_scene_config, parse_frames
from .metrics import measure_pose, pose_record
from .pcd import sha256_file, verify_checksum_index
from .topk_ambiguity_analysis import validate_trace


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json(path: Path, value: dict) -> None:
    """Atomically update run-owned metadata only."""
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)
                         + '\n', encoding='utf-8')
    temporary.replace(path)


def _file(path: Path) -> Path:
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'missing/unsafe input file: {path}')
    return path.resolve(strict=True)


def _tree_files(root: Path) -> dict[str, str]:
    """Hash every file actually presented as a descriptor/BBS asset."""
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'symlink is not a frozen asset: {path}')
        if path.is_file():
            result[path.relative_to(root).as_posix()] = sha256_file(path)
    if not result:
        raise ValueError(f'empty assets directory: {root}')
    return result


def parse_scene_ids(value: str) -> tuple[str, ...]:
    ids = tuple(_safe_id(part.strip(), 'scene id') for part in value.split(','))
    if not ids or len(set(ids)) != len(ids):
        raise ValueError('--scene-ids requires unique comma-separated scene IDs')
    return ids


def _library_candidates() -> dict[str, Path]:
    """Freeze configured native library bytes without claiming ldd resolution."""
    paths = {}
    for directory in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep):
        if not directory:
            continue
        for name in ('libsmall_gicp.so', 'libcpu_bbs3d.so'):
            candidate = Path(directory) / name
            if candidate.is_file():
                resolved = candidate.resolve(strict=True)
                paths[str(resolved)] = resolved
    return paths


def _snapshot(data: MapPackage, tracked: dict[str, Path], assets: Path,
              libraries: dict[str, Path], *, reverify_map: bool) -> dict:
    indexed = (verify_checksum_index(data.root, ('map.pcd', 'poses_timed.txt', 'manifest.yaml'))
               if reverify_map else dict(data.checksums))
    return {
        'map_package_indexed_files_sha256': indexed,
        'tracked_files_sha256': {name: sha256_file(_file(path)) for name, path in sorted(tracked.items())},
        'asset_files_sha256': _tree_files(assets),
        'configured_native_libraries_sha256': {name: sha256_file(path) for name, path in sorted(libraries.items())},
    }


def _prepare(map_package: Path, scenes: Path, benchmark_assets_root: Path,
             native_localizer_path: Path, output: Path, scene_ids: tuple[str, ...],
             frames: tuple[int, ...], backend_id: str, candidate_top_k: int) -> dict:
    output = Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise ValueError(f'output already exists; choose a new isolated run: {output}')
    if not scene_ids or len(set(scene_ids)) != len(scene_ids):
        raise ValueError('scene IDs must be nonempty and unique')
    for scene_id in scene_ids:
        _safe_id(scene_id, 'scene id')
    _safe_id(backend_id, 'backend id')
    if parse_frames(','.join(str(n) for n in frames)) != frames:
        raise ValueError('invalid frames')
    if isinstance(candidate_top_k, bool) or not isinstance(candidate_top_k, int) or candidate_top_k <= 0:
        raise ValueError('candidate_top_k must be a positive integer')

    data = MapPackage(map_package)
    if data.reference_type not in ('FRONTEND_SAME_SESSION_REFERENCE', 'FASTLIO2_SAME_SESSION_REFERENCE'):
        raise ValueError('replay requires a declared same-session frontend reference; PGO/undeclared maps are excluded')
    declared_backend = data.metadata.get('mapping_backend', {}).get('id')
    if declared_backend is not None and declared_backend != backend_id:
        raise ValueError(f'backend_id={backend_id} differs from map backend={declared_backend}')
    scenes = _file(scenes)
    config = load_scene_config(scenes, pose_count=len(data.poses))
    by_id = {scene.scene_id: scene for scene in config.scenes}
    missing = set(scene_ids) - set(by_id)
    if missing:
        raise ValueError(f'scene IDs absent from scene config: {sorted(missing)}')
    selected_scenes = tuple(by_id[scene_id] for scene_id in scene_ids)
    root = Path(benchmark_assets_root).resolve(strict=True)
    if not root.is_dir():
        raise ValueError('benchmark assets root must be a completed benchmark run directory')
    if any(output.resolve().is_relative_to(source) or source.is_relative_to(output.resolve())
           for source in (data.root, root)):
        raise ValueError('output must be outside source map package and benchmark run')
    native = _file(native_localizer_path)
    if not os.access(native, os.X_OK):
        raise ValueError(f'native localizer is not executable: {native}')
    prior_manifest_path = _file(root / 'manifest.json')
    prior = json.loads(prior_manifest_path.read_text(encoding='utf-8'))
    if prior.get('status') != 'COMPLETED':
        raise ValueError('source benchmark manifest must have status COMPLETED')
    if Path(prior.get('map_package', '')).resolve() != data.root:
        raise ValueError('source benchmark was built from a different map package')
    if prior.get('scene_yaml_sha256') != sha256_file(scenes):
        raise ValueError('scene YAML hash differs from source benchmark query selection')
    if prior.get('map_checksums_sha256') != sha256_file(data.root / 'checksums.sha256'):
        raise ValueError('map package checksum index differs from source benchmark')
    if not set(frames).issubset(set(prior.get('parameters', {}).get('frames', []))):
        raise ValueError('requested frames were not generated by the source benchmark')

    assets = root / 'global' / 'assets'
    if assets.is_symlink() or not assets.is_dir():
        raise ValueError(f'missing/unsafe descriptor/BBS assets: {assets}')
    target = _file(root / 'global' / 'global_target_map.pcd')
    assets_manifest_path = _file(root / 'global' / 'assets_manifest.json')
    asset_manifest = json.loads(assets_manifest_path.read_text(encoding='utf-8'))
    if asset_manifest.get('target', {}).get('sha256') != sha256_file(target):
        raise ValueError('held-out GLOBAL target hash differs from source assets manifest')
    asset_hashes = _tree_files(assets)
    if asset_manifest.get('asset_files_sha256') != asset_hashes:
        raise ValueError('descriptor/BBS asset set or hashes differ from source assets manifest')
    for required in ('polar_context.db', 'polar_context.yaml', 'global_map_downsampled.pcd'):
        _file(assets / required)
    heldout = set(asset_manifest.get('heldout_query_indices', []))
    target_indices = set(asset_manifest.get('target', {}).get('patch_indices', []))
    if not heldout or not target_indices or heldout & target_indices:
        raise ValueError('source assets must document nonoverlapping held-out query and target patch indices')

    queries = []
    for scene in selected_scenes:
        for count in frames:
            window = set(range(scene.keyframe - count // 2, scene.keyframe + count // 2 + 1))
            if not window.issubset(heldout):
                raise ValueError(f'{scene.scene_id}/f{count}: query window was not excluded from source assets')
            path = _file(root / 'global' / 'queries' / f'{scene.scene_id}_f{count}.pcd')
            queries.append((scene, count, path))

    settings = GLOBAL_SETTINGS | prior.get('parameters', {}).get('global_native', {})
    settings['candidate_top_k'] = candidate_top_k
    if candidate_top_k > settings['descriptor_prefilter']:
        raise ValueError('candidate_top_k exceeds frozen descriptor_prefilter')
    tracked = {
        'map_metadata': _file(data.root / 'metadata.yaml'),
        'map_manifest': _file(data.root / 'manifest.yaml'),
        'map_checksums': _file(data.root / 'checksums.sha256'),
        'scenes': scenes,
        'native_executable': native,
        'source_benchmark_manifest': prior_manifest_path,
        'source_assets_manifest': assets_manifest_path,
        'global_target_map': target,
    }
    for scene, count, path in queries:
        tracked[f'query:{scene.scene_id}:f{count}'] = path
    return {'data': data, 'config': config, 'root': root, 'assets': assets, 'target': target,
            'native': native, 'output': output, 'queries': queries, 'settings': settings,
            'tracked': tracked, 'libraries': _library_candidates()}


def run_replay(*, map_package: Path, scenes: Path, benchmark_assets_root: Path,
               native_localizer_path: Path, output: Path, backend_id: str,
               scene_ids: tuple[str, ...] = ('MIDDLE_01', 'HEADLAND_01'),
               frames: tuple[int, ...] = (1, 3, 5), candidate_top_k: int = 10) -> dict:
    inputs = _prepare(map_package, scenes, benchmark_assets_root, native_localizer_path,
                      output, scene_ids, frames, backend_id, candidate_top_k)
    data, root = inputs['data'], inputs['output']
    before = _snapshot(data, inputs['tracked'], inputs['assets'], inputs['libraries'], reverify_map=False)
    root.mkdir(parents=True, exist_ok=False)
    for directory in ('native_traces', 'traces', 'invocations'):
        (root / directory).mkdir()
    _json(root / 'input_hashes_before.json', before)
    manifest = {
        'schema_version': 1, 'status': 'RUNNING', 'created_utc': _utc(),
        'backend_id': backend_id, 'map_package': str(data.root),
        'benchmark_assets_root': str(inputs['root']), 'scene_ids': list(scene_ids),
        'frames': list(frames), 'global_settings': inputs['settings'],
        'subprocess_timeout_sec': 30.0, 'reference_type': data.reference_type,
        'reference': data.reference, 'physical_row_status': 'NOT_INFERRED_FROM_TRAJECTORY',
        'offline_reference_tolerance': {'xy_m': 0.5, 'yaw_deg': 5.0},
        'runtime_acceptance_observed': False,
        'heldout_control': 'reuse checksum-frozen source target/descriptor assets; all selected query windows excluded',
        'source_metadata_erratum': 'source assets same_session_pgo_reference/remaining_bias boilerplate is not authoritative; validated map metadata declares frontend reference and pgo_applied=false',
        'tracked_input_paths': {name: str(path) for name, path in inputs['tracked'].items()},
        'library_hash_scope': 'configured LD_LIBRARY_PATH candidates; not a claim of complete dynamic linker resolution',
        'query_count': len(inputs['queries']), 'queries': [], 'immutable_inputs_verified': False,
    }
    _json(root / 'manifest.json', manifest)
    errors = []
    try:
        for scene, count, query_path in inputs['queries']:
            query_id = f'{scene.scene_id}_f{count}'
            native_trace = root / 'native_traces' / f'{query_id}.json'
            trace_path = root / 'traces' / f'{query_id}.json'
            ref = data.poses[scene.keyframe]
            reference = pose_record(ref.t, ref.quat_xyzw)
            forward = ref.rotation[:, 0]
            reference['yaw_deg'] = math.degrees(math.atan2(float(forward[1]), float(forward[0])))
            query_metadata = {
                'frames': count, 'timestamp': ref.stamp, 'keyframe': scene.keyframe,
                'scene_id': scene.scene_id, 'scene_type': scene.scene_type.upper(), 'backend_id': backend_id,
            }
            command = global_command(inputs['native'], inputs['target'], query_path, inputs['assets'],
                                     settings=inputs['settings'], trace_candidates_json=native_trace,
                                     query_metadata=query_metadata)
            invocation = {'schema_version': 1, 'created_utc': _utc(), 'argv': command,
                          'shell_display': shlex.join(command), 'cwd': str(Path.cwd()),
                          'timeout_sec': 30.0, 'environment': {name: os.environ.get(name) for name in
                              ('LD_LIBRARY_PATH', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'OMP_NUM_THREADS')},
                          'query': query_metadata, 'status': 'RUNNING'}
            invocation_path = root / 'invocations' / f'{query_id}.json'
            _json(invocation_path, invocation)
            native = invoke(command, timeout=30.0)
            measured = measure_pose(reference, native)
            nominal = bool(native.get('backend_success') and measured['xy_error_m'] is not None
                           and measured['xy_error_m'] <= 0.5 and measured['yaw_error_deg'] <= 5.0)
            invocation.update(completed_utc=_utc(), status='COMPLETED', native_result=native,
                              offline_pose_errors=measured, offline_reference_nominal_success=nominal)
            record = {'query_id': query_id, **query_metadata, 'native_backend_success': native.get('backend_success'),
                      'native_exit_code': native.get('backend_exit_code'), 'offline_reference_nominal_success': nominal,
                      'invocation': str(invocation_path), 'native_trace': str(native_trace),
                      'enriched_trace': None, 'trace_error': None}
            try:
                if not native_trace.is_file():
                    raise ValueError('native trace missing; preserve this invocation as unavailable rather than invent candidates')
                original = json.loads(native_trace.read_text(encoding='utf-8'))
                validate_trace(original)
                for name, expected in query_metadata.items():
                    observed = original['query'].get(name)
                    if name == 'timestamp':
                        matches = (isinstance(observed, (int, float)) and math.isfinite(observed)
                                   and abs(observed - expected) <= 1e-6)
                    else:
                        matches = observed == expected
                    if not matches:
                        raise ValueError(f'native query metadata {name} differs from invocation')
                if original['descriptor']['candidate_top_k'] != candidate_top_k:
                    raise ValueError('native trace candidate_top_k differs from requested K')
                if original['descriptor']['prefilter'] != inputs['settings']['descriptor_prefilter']:
                    raise ValueError('native trace prefilter differs from frozen config')
                shutil.copyfile(native_trace, trace_path)
                enrich_candidate_trace(trace_path, query={**query_metadata, 'query_id': query_id,
                    'scan': str(query_path), 'map_package': str(data.root), 'reference_pose': reference},
                    nominal_success=nominal, provenance={
                        'input_hashes_before_sha256': sha256_file(root / 'input_hashes_before.json'),
                        'map_metadata_sha256': before['tracked_files_sha256']['map_metadata'],
                        'map_manifest_sha256': before['tracked_files_sha256']['map_manifest'],
                        'map_checksums_sha256': before['tracked_files_sha256']['map_checksums'],
                        'map_package_indexed_files_sha256': before['map_package_indexed_files_sha256'],
                        'scene_yaml_sha256': before['tracked_files_sha256']['scenes'],
                        'query_pcd_sha256': before['tracked_files_sha256'][f'query:{scene.scene_id}:f{count}'],
                        'native_executable_sha256': before['tracked_files_sha256']['native_executable'],
                        'descriptor_database_sha256': before['asset_files_sha256']['polar_context.db'],
                        'asset_files_sha256': before['asset_files_sha256'],
                        'configured_native_libraries_sha256': before['configured_native_libraries_sha256'],
                        'global_target_map_sha256': before['tracked_files_sha256']['global_target_map'],
                        'source_assets_manifest_sha256': before['tracked_files_sha256']['source_assets_manifest'],
                        'source_benchmark_manifest_sha256': before['tracked_files_sha256']['source_benchmark_manifest'],
                        'native_trace_sha256': sha256_file(native_trace), 'command_argv': command,
                        'reference_type': data.reference_type, 'absolute_ground_truth': False,
                    })
                validate_trace(json.loads(trace_path.read_text(encoding='utf-8')))
                record['enriched_trace'] = str(trace_path)
                record['trace_sha256'] = sha256_file(trace_path)
                record['ranked_candidate_count'] = len(original['ranked_candidates'])
                record['selected_candidate_rank'] = original['selected'].get('candidate_rank')
            except (ValueError, KeyError, TypeError, OSError) as exc:
                record['trace_error'] = str(exc)
                errors.append(f'{query_id}: {exc}')
            invocation['trace_record'] = record
            _json(invocation_path, invocation)
            manifest['queries'].append(record)
            _json(root / 'manifest.json', manifest)
            line = (f'{query_id}: exit={record["native_exit_code"]} native_success={record["native_backend_success"]}'
                    f' reference_nominal={nominal} ranked={record.get("ranked_candidate_count", 0)}'
                    f' trace_error={record["trace_error"]}')
            with (root / 'run.log').open('a', encoding='utf-8') as log:
                log.write(f'{_utc()} {line}\n')
            print(line, flush=True)
        after = _snapshot(data, inputs['tracked'], inputs['assets'], inputs['libraries'], reverify_map=True)
        _json(root / 'input_hashes_after.json', after)
        if before != after:
            raise ValueError('frozen input bytes changed during replay')
        manifest['immutable_inputs_verified'] = True
        manifest['status'] = 'COMPLETE' if not errors else 'COMPLETE_WITH_TRACE_ERRORS'
    except BaseException as exc:
        manifest['status'] = 'FAILED'
        manifest['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        manifest['completed_utc'] = _utc()
        manifest['trace_errors'] = errors
        manifest['trace_available_count'] = sum(bool(row['enriched_trace']) for row in manifest['queries'])
        manifest['native_success_count'] = sum(bool(row['native_backend_success']) for row in manifest['queries'])
        manifest['offline_reference_nominal_success_count'] = sum(bool(row['offline_reference_nominal_success'])
                                                                   for row in manifest['queries'])
        _json(root / 'manifest.json', manifest)
    return manifest


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-package', required=True, type=Path)
    parser.add_argument('--scenes', required=True, type=Path)
    parser.add_argument('--benchmark-assets-root', required=True, type=Path)
    parser.add_argument('--native-localizer-path', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--backend-id', required=True)
    parser.add_argument('--scene-ids', type=parse_scene_ids, default=('MIDDLE_01', 'HEADLAND_01'))
    parser.add_argument('--frames', type=parse_frames, default=(1, 3, 5))
    parser.add_argument('--candidate-top-k', type=int, default=10)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    try:
        manifest = run_replay(**vars(args))
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, f'topk_trace_replay: {exc}\n')
    print(json.dumps({name: manifest[name] for name in ('status', 'query_count', 'trace_available_count',
        'native_success_count', 'offline_reference_nominal_success_count', 'immutable_inputs_verified')}, indent=2))
    return 0 if manifest['status'] == 'COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
