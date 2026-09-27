"""Offline diagnostics; NEVER modifies or republishes geometry_v1 evidence."""
from __future__ import annotations

import numpy as np

from .dataset import PgoEvidence


def spectrum(matrix: np.ndarray, n: int, minimum: int, epsilon: float) -> dict:
    """Ht (dimensionless) and Hr (m^2) must be evaluated separately."""
    if n < minimum:
        return {'valid': False, 'q': None, 'eigenvalues': None, 'weak_xyz_map': None,
                'reason': f'valid normal voxels {n} < {minimum}'}
    mat = 0.5 * (matrix + matrix.T)
    if not np.isfinite(mat).all() or np.trace(mat) <= epsilon:
        return {'valid': False, 'q': None, 'eigenvalues': None, 'weak_xyz_map': None,
                'reason': 'nonfinite or near-zero trace'}
    eigen, vectors = np.linalg.eigh(mat)
    if not np.isfinite(eigen).all():
        return {'valid': False, 'q': None, 'eigenvalues': None, 'weak_xyz_map': None,
                'reason': 'nonfinite spectrum'}
    eigen = np.maximum(eigen, 0.0)
    return {'valid': True, 'q': float(np.clip(3.0 * eigen[0] / (eigen.sum() + epsilon), 0, 1)),
            'eigenvalues': [float(v) for v in eigen],
            'weak_xyz_map': [float(v) for v in vectors[:, 0]], 'reason': None}


def query_geometry(data: PgoEvidence, map_points: np.ndarray, key_lookup: np.ndarray,
                   key_found: np.ndarray, selected: np.ndarray, reference_t: np.ndarray,
                   *, radius_xy: float = 12.0, half_height: float = 5.0) -> dict:
    """Qr_query_conditioned uses current query body origin, not mapping-era body origins.

    Inputs are candidate-map points ONLY. Voxel normals come from the immutable
    full-session geometry_v1 sidecar; that normal-source leakage is disclosed.
    Qr_mapping_view is the *median sidecar per-voxel Qr*, NOT this new spectrum.
    """
    coords = np.asarray(map_points[selected], dtype='f8')
    delta = coords - np.asarray(reference_t, dtype='f8')
    within = ((np.sum(delta[:, :2] ** 2, axis=1) <= radius_xy ** 2)
              & (np.abs(delta[:, 2]) <= half_height))
    indices = selected[within]
    lookup = key_lookup[indices]
    valid = key_found[indices] & (data.geom['normal_valid'][lookup] != 0)
    indices, lookup = indices[valid], lookup[valid]
    if len(lookup):
        sort = np.argsort(lookup, kind='stable')
        indices, lookup = indices[sort], lookup[sort]
        voxel, starts, counts = np.unique(lookup, return_index=True, return_counts=True)
    else:
        voxel = np.array([], dtype='i8')
        starts = counts = np.array([], dtype='i8')
    n = len(voxel)
    min_normals = int(data.geom_meta['parameters']['min_valid_normals'])
    ep = data.geom_meta['parameters']['epsilon']
    ht = np.zeros((3, 3), dtype='f8')
    hr = np.zeros((3, 3), dtype='f8')
    if n:
        normals = np.column_stack((data.geom['normal_x'][voxel],
                                   data.geom['normal_y'][voxel],
                                   data.geom['normal_z'][voxel])).astype('f8')
        if not np.isfinite(normals).all():
            raise ValueError('geometry_v1 marked a nonfinite normal valid')
        ht = normals.T @ normals  # one equal vote per distinct occupied normal voxel
        per_point_n = np.repeat(normals, counts, axis=0)
        g = np.cross(np.asarray(map_points[indices], dtype='f8') - reference_t, per_point_n)
        per_point_outer = np.einsum('ni,nj->nij', g, g).reshape(-1, 9)
        per_voxel_mean = np.add.reduceat(per_point_outer, starts, axis=0) / counts[:, None]
        hr = per_voxel_mean.sum(axis=0).reshape(3, 3)
    qt = spectrum(ht, n, min_normals, float(ep['translation']))
    qr = spectrum(hr, n, min_normals, float(ep['rotation_m2']))

    def median_sidecar(field: str, valid_field: str) -> float | None:
        included = voxel[(data.geom[valid_field][voxel] != 0)
                         & np.isfinite(data.geom[field][voxel])]
        return float(np.median(data.geom[field][included])) if len(included) else None

    return {
        'evidence_crop_center': 'optimized_PGO_pose.query_body_origin',
        'evidence_crop_radius_xy_m': radius_xy, 'evidence_crop_half_height_m': half_height,
        'candidate_points_in_evidence_crop': int(within.sum()),
        'valid_normal_voxels': int(n), 'normal_supported_map_points': int(len(indices)),
        'qt_mapping_view_median': median_sidecar('translation_q', 'translation_valid'),
        'qr_mapping_view_median': median_sidecar('rotation_q', 'rotation_valid'),
        'qt_query_local': qt,
        'qr_query_conditioned': qr,
    }


def alignment_and_recovery(reference_t: np.ndarray, initial_t: np.ndarray,
                           result_t: np.ndarray | None, geometry: dict,
                           yaw_perturb_deg: float) -> dict:
    weak = geometry['qt_query_local']['weak_xyz_map']
    rotweak = geometry['qr_query_conditioned']['weak_xyz_map']
    direction = initial_t - reference_t
    norm = float(np.linalg.norm(direction))
    output = {'translation_perturb_weak_axis_abs_cos': None,
              'yaw_axis_vs_rotation_weak_abs_cos': None,
              'initial_weak_projection_error_m': None,
              'final_weak_projection_error_m': None,
              'weak_axis_recovery_m': None,
              'strong_plane_recovery_m': None}
    if rotweak is not None and yaw_perturb_deg:
        output['yaw_axis_vs_rotation_weak_abs_cos'] = abs(float(np.dot(rotweak, [0, 0, 1])))
    if weak is None:
        return output
    w = np.asarray(weak, dtype='f8')
    start_weak = abs(float(np.dot(direction, w)))
    output['initial_weak_projection_error_m'] = start_weak
    if norm:
        output['translation_perturb_weak_axis_abs_cos'] = start_weak / norm
    if result_t is None:
        return output
    final = np.asarray(result_t, dtype='f8') - reference_t
    final_weak = abs(float(np.dot(final, w)))
    output['final_weak_projection_error_m'] = final_weak
    output['weak_axis_recovery_m'] = start_weak - final_weak
    start_strong = float(np.linalg.norm(direction - np.dot(direction, w) * w))
    final_strong = float(np.linalg.norm(final - np.dot(final, w) * w))
    output['strong_plane_recovery_m'] = start_strong - final_strong
    return output
