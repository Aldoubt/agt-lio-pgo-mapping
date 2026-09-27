"""Thin offline subprocess adapters for the EXACT native localization executables.

No BBS, descriptor, or GICP algorithm is implemented in this package.
"""
from __future__ import annotations

import json
import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .dataset import PgoEvidence
from .pcd import sha256_file, write_pcd
from .selection import Candidate

# Audited: agt_global_relocalization_native/map_gicp_tracker.cpp and
# agt_global_relocalization/manual_seed_relocalization.py runtime defaults.
LOCAL_SETTINGS = {
    'radius_xy_m': 12.0, 'half_height_m': 5.0,
    'map_leaf_m': 0.25, 'scan_leaf_m': 0.25,
    'max_correspondence_distance_m': 1.5, 'threads': 4,
    'constraint_mode': 'full_se3',
}
# Audited: global_relocalization.yaml candidate_sdk_command and native CLI defaults.
GLOBAL_SETTINGS = {
    'timeout_sec': 18.0, 'map_leaf_m': 0.35, 'scan_leaf_m': 0.35,
    'candidate_top_k': 4, 'descriptor_prefilter': 40,
    'candidate_xy_radius_m': 4.0, 'candidate_z_radius_m': 2.0,
    'candidate_yaw_range_deg': 0.0, 'roll_pitch_range_deg': 0.0,
    'per_candidate_timeout_sec': 8.0, 'bbs_score_threshold': 0.05,
    'gicp_max_corr_m': 2.0, 'local_map_radius_xy_m': 35.0,
    'local_map_half_height_m': 8.0, 'min_local_map_points': 800,
    'threads': 8, 'query_frame_mode': 'mapping_body',
    'assets_map_leaf_m': 0.5, 'bbs_min_level_res_m': 0.5, 'bbs_max_level': 5,
    'descriptor_min_patch_points': 300,
}
NATIVE_PACKAGE = 'agt_global_relocalization_native'
PROGRAMS = ('map_gicp_tracker', 'candidate_bbs_gicp_localizer',
            'build_relocalization_assets', 'build_relocalization_candidates')


def dependency_fingerprints(ros_install: Path, src_root: Path) -> dict:
    """Record the linked runtime library bytes AND vendored source versions.

    Compiled executables link small_gicp and CPU BBS dynamically: hashing only
    the executable does not lock the registration/search implementation.
    Missing libraries stay null rather than claiming an unavailable backend.
    """
    libraries = Path(ros_install).resolve().parent / '.agt_native' / 'lib'
    result = {}
    for name, filename in (('small_gicp', 'libsmall_gicp.so'),
                           ('3d_bbs_cpu', 'libcpu_bbs3d.so')):
        path = libraries / filename
        result[name] = {'linked_library_path': str(path),
                        'linked_library_sha256': sha256_file(path) if path.is_file() else None}
    for name, repo in (('small_gicp', 'small_gicp'), ('3d_bbs_cpu', '3d_bbs')):
        path = Path(src_root) / 'external' / repo
        try:
            version = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                                               text=True, stderr=subprocess.DEVNULL).strip()
        except (FileNotFoundError, subprocess.CalledProcessError):
            version = None
        result[name]['vendored_git_commit'] = version
    return result


@dataclass(frozen=True)
class NativePrograms:
    paths: dict[str, Path]
    sha256: dict[str, str]
    available: dict[str, bool]

    @classmethod
    def discover(cls, ros_install: Path) -> 'NativePrograms':
        base = Path(ros_install) / NATIVE_PACKAGE / 'lib' / NATIVE_PACKAGE
        paths = {name: base / name for name in PROGRAMS}
        available = {name: path.is_file() for name, path in paths.items()}
        hashes = {name: sha256_file(path) for name, path in paths.items() if available[name]}
        return cls(paths, hashes, available)

    @property
    def local_ready(self) -> bool:
        return self.available['map_gicp_tracker']

    @property
    def global_ready(self) -> bool:
        return all(self.available[n] for n in PROGRAMS[1:])


def parse_backend(stdout: str, stderr: str, returncode: int, wall_ms: float) -> dict:
    result = None
    for line in reversed(stdout.splitlines()):
        try:
            # C++ may print `nan` for an unavailable Hessian eigenvalue.  Do
            # not serialize fake IEEE NaN into strict JSON output; mark it null.
            result = json.loads(line, parse_constant=lambda _value: None)
        except (ValueError, TypeError):
            continue
        if isinstance(result, dict):
            break
        result = None
    if result is None:
        result = {'success': False, 'message': 'no parseable backend JSON'}
    success = bool(result.get('success', False)) and returncode == 0
    pose = None
    if all(key in result for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')):
        values = [result[k] for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
        if all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
            if sum(float(v) ** 2 for v in values[3:]) > 1e-12:
                pose = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), map(float, values)))
    reason = None if success else str(result.get('message') or stderr.strip()[:500] or f'exit={returncode}')
    if result.get('success') and returncode != 0:
        reason = f'backend returned success JSON but exit={returncode}; {reason}'
    if success and pose is None:
        reason, success = 'invalid result pose from successful backend', False
    def finite(name: str):
        value = result.get(name)
        try:
            f = float(value)
            return f if math.isfinite(f) else None
        except (TypeError, ValueError):
            return None
    return {
        'backend_success': success,
        'converged': True if success else (False if 'did not converge' in (reason or '') else None),
        'backend_exit_code': returncode, 'backend_reason': reason,
        'pose': pose, 'coarse_pose': ({k.replace('coarse_', ''): result['coarse_' + k]
                                     for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')}
                                    if all('coarse_' + k in result for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
                                    else None),
        'fitness_native': finite('fitness'), 'overlap_native': finite('overlap'),
        'inliers_native': None, 'iterations_native': None,
        'map_points_native': result.get('map_points', result.get('gicp_target_points')),
        'query_points_native': result.get('query_points'),
        'wall_ms_external': wall_ms,
        'bbs_score_native': finite('bbs_score'),
        'bbs_elapsed_ms_native': finite('bbs_elapsed_ms'),
        'descriptor_ring_distance_native': finite('descriptor_ring_distance'),
        'descriptor_similarity_native': finite('descriptor_similarity'),
        'descriptor_shift_native': result.get('descriptor_shift'),
        'candidate_patch_native': result.get('candidate_patch'),
        'native_hessian_eigenvalues_mixed_units': result.get('hessian_eigenvalues'),
    }


def invoke(command: list[str], *, timeout: float) -> dict:
    start = time.perf_counter()
    try:
        process = subprocess.run(command, text=True, capture_output=True, timeout=timeout,
                                 check=False)
    except subprocess.TimeoutExpired:
        return parse_backend('', '', 124, (time.perf_counter() - start) * 1000) | {
            'backend_reason': f'offline subprocess exceeded {timeout:.1f}s',
        }
    return parse_backend(process.stdout, process.stderr, process.returncode,
                         (time.perf_counter() - start) * 1000)


def local_command(binary: Path, map_pcd: Path, query_pcd: Path,
                  initial_t: np.ndarray, initial_xyzw: np.ndarray) -> list[str]:
    p = LOCAL_SETTINGS
    return [str(binary), '--map', str(map_pcd), '--scan', str(query_pcd),
            '--x', str(float(initial_t[0])), '--y', str(float(initial_t[1])),
            '--z', str(float(initial_t[2])),
            '--qx', str(float(initial_xyzw[0])), '--qy', str(float(initial_xyzw[1])),
            '--qz', str(float(initial_xyzw[2])), '--qw', str(float(initial_xyzw[3])),
            '--radius', str(p['radius_xy_m']), '--half-height', str(p['half_height_m']),
            '--map-leaf', str(p['map_leaf_m']), '--scan-leaf', str(p['scan_leaf_m']),
            '--max-corr', str(p['max_correspondence_distance_m']),
            '--threads', str(p['threads']), '--constraint-mode', p['constraint_mode']]


def global_command(binary: Path, map_pcd: Path, query_pcd: Path,
                   assets_dir: Path) -> list[str]:
    p = GLOBAL_SETTINGS
    return [str(binary), '--map', str(map_pcd), '--scan', str(query_pcd),
            '--assets-dir', str(assets_dir), '--timeout', str(p['timeout_sec']),
            '--map-leaf', str(p['map_leaf_m']), '--scan-leaf', str(p['scan_leaf_m']),
            '--candidate-top-k', str(p['candidate_top_k']),
            '--descriptor-prefilter', str(p['descriptor_prefilter']),
            '--candidate-xy-radius', str(p['candidate_xy_radius_m']),
            '--candidate-z-radius', str(p['candidate_z_radius_m']),
            '--candidate-yaw-range-deg', str(p['candidate_yaw_range_deg']),
            '--roll-pitch-range-deg', str(p['roll_pitch_range_deg']),
            '--per-candidate-timeout', str(p['per_candidate_timeout_sec']),
            '--bbs-score-threshold', str(p['bbs_score_threshold']),
            '--gicp-max-corr', str(p['gicp_max_corr_m']),
            '--local-map-radius-xy', str(p['local_map_radius_xy_m']),
            '--local-map-half-height', str(p['local_map_half_height_m']),
            '--min-local-map-points', str(p['min_local_map_points']),
            '--threads', str(p['threads']),
            '--bbs-query-frame-mode', p['query_frame_mode']]


def build_candidate_assets(native: NativePrograms, data: PgoEvidence,
                           map_indices: tuple[int, ...], candidate: Candidate,
                           map_points: np.ndarray, destination: Path) -> dict:
    """Call original asset/descriptor builders with candidate-only map patches.

    Reprojection map->body only prepares PCD input. Neither builder, descriptor,
    BBS nor GICP math is duplicated. Query indices are checked before writing.
    """
    if not native.global_ready:
        return {'status': 'NOT_RUN', 'reason': 'native descriptor/BBS programs not installed'}
    if set(map_indices) - set(range(len(data.poses))):
        raise ValueError('out-of-range map index')
    destination.mkdir(parents=True, exist_ok=False)
    map_dir = destination / 'map_subset'
    patches = map_dir / 'patches'
    patches.mkdir(parents=True)
    pose_rows = []
    start = 0
    for i in map_indices:
        pose = data.poses[i]
        a, b = data.patch_ranges[i]
        # raw_map_subset filters nonfinite parent points before concatenation.
        # Use the identical finite-point lengths for descriptor patch slicing.
        parent = data.map_data[a:b]
        size = int(np.isfinite(np.column_stack((parent['x'], parent['y'], parent['z']))).all(axis=1).sum())
        subset = candidate.indices[(candidate.indices >= start) & (candidate.indices < start + size)]
        start += size
        if len(subset) < GLOBAL_SETTINGS['descriptor_min_patch_points']:
            continue
        map_coord = np.asarray(map_points[subset], dtype='f8')
        body = ((map_coord - pose.t) @ pose.rotation).astype('<f4')
        write_pcd(patches / pose.patch, body)
        qw, qx, qy, qz = pose.quat_xyzw[[3, 0, 1, 2]]
        pose_rows.append(f'{pose.patch} {pose.t[0]:.17g} {pose.t[1]:.17g} '
                         f'{pose.t[2]:.17g} {qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g}\n')
    if start != len(map_points):
        raise ValueError('candidate descriptor patch offsets differ from finite map-subset order')
    if not pose_rows:
        return {'status': 'NOT_RUN', 'reason': 'candidate has no >=300-point map-only descriptor patch',
                'descriptor_patch_count': 0}
    (map_dir / 'poses.txt').write_text(''.join(pose_rows), encoding='ascii')
    args = [str(native.paths['build_relocalization_assets']), '--map', str(candidate.path),
            '--output', str(destination / 'assets'), '--map-leaf',
            str(GLOBAL_SETTINGS['assets_map_leaf_m']), '--bbs-min-level-res',
            str(GLOBAL_SETTINGS['bbs_min_level_res_m']), '--bbs-max-level',
            str(GLOBAL_SETTINGS['bbs_max_level'])]
    try:
        coarse = subprocess.run(args, text=True, capture_output=True, timeout=150, check=False)
        desc = subprocess.run([str(native.paths['build_relocalization_candidates']),
                               '--map-dir', str(map_dir), '--output', str(destination / 'assets'),
                               '--min-patch-points', str(GLOBAL_SETTINGS['descriptor_min_patch_points'])],
                              text=True, capture_output=True, timeout=150, check=False)
    except subprocess.TimeoutExpired as exc:
        return {'status': 'NOT_RUN', 'reason': f'asset builder timeout: {exc}',
                'descriptor_patch_count': len(pose_rows)}
    result = {'status': 'READY' if coarse.returncode == 0 and desc.returncode == 0 else 'NOT_RUN',
              'reason': None if coarse.returncode == 0 and desc.returncode == 0 else
              f'asset builder exit={coarse.returncode} descriptor builder exit={desc.returncode}',
              'descriptor_patch_count': len(pose_rows),
              'asset_build_stdout_tail': coarse.stdout[-600:],
              'descriptor_build_stdout_tail': desc.stdout[-600:],
              'asset_build_stderr_tail': coarse.stderr[-400:],
              'descriptor_build_stderr_tail': desc.stderr[-400:],
              'assets_dir': str(destination / 'assets')}
    if result['status'] == 'READY':
        db = destination / 'assets' / 'polar_context.db'
        if not db.is_file():
            result['status'], result['reason'] = 'NOT_RUN', 'descriptor database missing'
        else:
            result['descriptor_database_sha256'] = sha256_file(db)
            asset_dir = destination / 'assets'
            result['asset_files_sha256'] = {
                str(path.relative_to(asset_dir)): sha256_file(path)
                for path in sorted(asset_dir.rglob('*')) if path.is_file()
            }
    return result
