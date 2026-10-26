"""Deterministic idealized plane/wall/corner/pole/parallel-wall adversarial cases."""
from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import numpy as np

from .backends import GLOBAL_SETTINGS, NativePrograms, global_command, invoke, local_command
from .geometry import spectrum
from .metrics import initial_pose, measure_pose, pose_record
from .pcd import write_pcd


def scenes() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Nx3 positions plus exact synthetic surface normals; no sensor noise."""
    a = np.arange(-8.0, 8.01, 0.20)
    b = np.arange(-3.8, 3.81, 0.20)
    ya, zb = np.meshgrid(a, b)
    wall_x = np.column_stack((np.zeros(ya.size), ya.ravel(), zb.ravel()))
    wall_x_n = np.tile([1.0, 0.0, 0.0], (len(wall_x), 1))
    xa, zb2 = np.meshgrid(a, b)
    wall_y = np.column_stack((xa.ravel(), np.zeros(xa.size), zb2.ravel()))
    wall_y_n = np.tile([0.0, 1.0, 0.0], (len(wall_y), 1))
    xg, yg = np.meshgrid(a, a)
    ground = np.column_stack((xg.ravel(), yg.ravel(), np.zeros(xg.size)))
    ground_n = np.tile([0.0, 0.0, 1.0], (len(ground), 1))
    angle, height = np.meshgrid(np.linspace(0, 2 * np.pi, 180, endpoint=False),
                                np.arange(-3.8, 3.81, 0.16))
    pole = np.column_stack((1.2 * np.cos(angle).ravel(),
                            1.2 * np.sin(angle).ravel(), height.ravel()))
    pole_n = np.column_stack((np.cos(angle).ravel(), np.sin(angle).ravel(),
                              np.zeros(angle.size)))
    parallel = np.concatenate([wall_x + np.array([v, 0, 0]) for v in (-4.0, 0.0, 4.0)])
    return {
        'plane_ground': (ground, ground_n),
        'wall': (wall_x, wall_x_n),
        'corner': (np.concatenate((wall_x, wall_y)), np.concatenate((wall_x_n, wall_y_n))),
        'pole': (pole, pole_n),
        'repeated_parallel_walls': (parallel, np.tile(wall_x_n, (3, 1))),
    }


SCENARIO = {
    'plane_ground': [('weak_x_1m', (1., 0, 0), 0.), ('strong_z_0.5m', (0, 0, .5), 0.)],
    'wall': [('weak_y_1m', (0, 1., 0), 0.), ('strong_x_0.5m', (.5, 0, 0), 0.)],
    'corner': [('weak_z_1m', (0, 0, 1.), 0.), ('strong_x_0.5m', (.5, 0, 0), 0.)],
    'pole': [('weak_yaw_45deg', (0, 0, 0), 45.), ('strong_x_0.5m', (.5, 0, 0), 0.)],
    'repeated_parallel_walls': [('weak_y_1m', (0, 1., 0), 0.),
                                ('aliased_x_2m', (2., 0, 0), 0.)],
}


def synthetic_geometry(coords: np.ndarray, normals: np.ndarray) -> dict:
    """Exact normal diagnostic only; never mistaken for frozen geometry_v1."""
    voxel = np.floor(coords / .2).astype('i8')
    unique, indices, inv, counts = np.unique(voxel, axis=0, return_index=True,
                                              return_inverse=True, return_counts=True)
    normal = normals[indices]
    ht = normal.T @ normal
    g = np.cross(coords, normals)
    outer = np.einsum('ni,nj->nij', g, g).reshape(-1, 9)
    accum = np.zeros((len(unique), 9), dtype='f8')
    np.add.at(accum, inv, outer)
    hr = (accum / counts[:, None]).sum(axis=0).reshape(3, 3)
    return {'normal_voxels': len(unique),
            'qt_analytic': spectrum(ht, len(unique), 6, 1e-6),
            'qr_query_conditioned_analytic': spectrum(hr, len(unique), 6, 1e-6),
            'qr_mapping_view': None, 'note': 'analytic synthetic normals, not Phase 3A sidecar'}


def _synthetic_global(native: NativePrograms, name: str, map_file: Path, query_file: Path,
                      directory: Path) -> dict:
    if not native.global_ready:
        return {'status': 'NOT_RUN', 'reason': 'descriptor/BBS executable unavailable'}
    map_dir = directory / 'map_dir'
    (map_dir / 'patches').mkdir(parents=True)
    # One exact known synthetic patch; no real PGO or localization feed is involved.
    (map_dir / 'patches' / 'synthetic.pcd').write_bytes(map_file.read_bytes())
    (map_dir / 'poses.txt').write_text('synthetic.pcd 0 0 0 1 0 0 0\n')
    assets = directory / 'assets'
    coarse = subprocess.run([str(native.paths['build_relocalization_assets']), '--map',
                             str(map_file), '--output', str(assets)],
                            text=True, capture_output=True, timeout=150, check=False)
    desc = subprocess.run([str(native.paths['build_relocalization_candidates']),
                           '--map-dir', str(map_dir), '--output', str(assets)],
                          text=True, capture_output=True, timeout=150, check=False)
    if coarse.returncode or desc.returncode:
        return {'status': 'NOT_RUN', 'reason': 'synthetic BBS/descriptor asset builder failed',
                'bbs_stderr': coarse.stderr[-300:], 'descriptor_stderr': desc.stderr[-300:]}
    result = invoke(global_command(native.paths['candidate_bbs_gicp_localizer'],
                                   map_file, query_file, assets), timeout=23)
    return {'status': 'RUN', 'native': result, 'metric': measure_pose(
        pose_record(np.zeros(3), np.array([0, 0, 0, 1])), result)}


def run_synthetic(native: NativePrograms, output: Path) -> dict:
    if not native.local_ready:
        raise RuntimeError('LOCAL native map_gicp_tracker is not available')
    output.mkdir(parents=True, exist_ok=False)
    cases = []
    reference = pose_record(np.zeros(3), np.array([0., 0., 0., 1.]))
    for name, (map_points, normals) in scenes().items():
        case_dir = output / name
        case_dir.mkdir()
        query = map_points[::2]  # same ideal geometry, deterministic sampling
        map_file, query_file = case_dir / 'map.pcd', case_dir / 'query_body.pcd'
        write_pcd(map_file, map_points)
        write_pcd(query_file, query)
        geom = synthetic_geometry(map_points, normals)
        for label, dxyz, yaw in [('zero', (0., 0., 0.), 0.)] + SCENARIO[name]:
            initial_t, initial_xyzw = initial_pose(np.zeros(3), np.array([0., 0., 0., 1.]),
                                                   {'dxyz_m': dxyz, 'dyaw_deg': yaw})
            native_result = invoke(local_command(native.paths['map_gicp_tracker'],
                                                map_file, query_file, initial_t, initial_xyzw),
                                   timeout=35)
            cases.append({'shape': name, 'scenario': label, 'algorithm': 'LOCAL',
                          'initial_pose': pose_record(initial_t, initial_xyzw),
                          'reference_label': 'EXACT_SYNTHETIC_DESIGNED_POSE;IDEALIZED_SELF_GEOMETRY',
                          'geometry': geom, 'native': native_result,
                          'metric': measure_pose(reference, native_result)})
        if name in ('corner', 'repeated_parallel_walls'):
            g = _synthetic_global(native, name, map_file, query_file, case_dir)
            cases.append({'shape': name, 'scenario': 'zero_no_initial_pose',
                          'algorithm': 'GLOBAL', 'reference_label': 'EXACT_SYNTHETIC_DESIGNED_POSE',
                          'geometry': geom, **g})
        print(f'SYNTHETIC {name}: {len(query)} query / {len(map_points)} map points', flush=True)
    status = {'cases': len(cases), 'local_cases': sum(c['algorithm'] == 'LOCAL' for c in cases),
              'local_native_success': sum(c['algorithm'] == 'LOCAL' and c['native']['backend_success']
                                          for c in cases),
              'global_cases': sum(c['algorithm'] == 'GLOBAL' for c in cases),
              'global_success': sum(c['algorithm'] == 'GLOBAL' and c.get('status') == 'RUN'
                                    and c['native']['backend_success'] for c in cases),
              'note': 'Synthetic tests share ideal geometry; no extrapolation to live localization.'}
    (output / 'results.json').write_text(json.dumps(cases, indent=2, allow_nan=False))
    (output / 'summary.json').write_text(json.dumps(status, indent=2))
    return status
