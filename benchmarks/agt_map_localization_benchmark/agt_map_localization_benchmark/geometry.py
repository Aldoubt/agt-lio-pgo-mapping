"""Offline diagnostics; NEVER modifies or republishes geometry_v1 evidence."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

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


def mapping_weak_consensus(sidecar: np.ndarray, voxel: np.ndarray,
                           kind: str, minimum: int) -> dict:
    """Axial consensus of mapping-era per-voxel weak directions, NOT Ht/Hr.

    v and -v are identical axes. A diffuse/ambiguous vote cannot be reported
    as a single meaningful mapping-view weak direction. This diagnostic does
    not reinterpret the sidecar's per-voxel Qt/Qr as query-conditioned Q.
    """
    included = voxel[sidecar[f'{kind}_valid'][voxel] != 0]
    vectors = np.column_stack([sidecar[f'{kind}_weak_{axis}'][included]
                               for axis in 'xyz']).astype('f8')
    lengths = np.linalg.norm(vectors, axis=1)
    keep = np.isfinite(vectors).all(axis=1) & np.isfinite(lengths) & (lengths > 1e-8)
    vectors = vectors[keep] / lengths[keep, None]
    count = len(vectors)
    base = {'valid': False, 'count': count, 'weak_xyz_map': None,
            'axial_coherence': None, 'top_eigen_gap_fraction': None}
    if count < minimum:
        return base | {'reason': f'valid mapping-era weak axes {count} < {minimum}'}
    moment = vectors.T @ vectors / count
    eigen, basis = np.linalg.eigh(moment)
    coherence = float(eigen[-1] / eigen.sum())
    gap = float((eigen[-1] - eigen[-2]) / eigen.sum())
    base.update({'axial_coherence': coherence, 'top_eigen_gap_fraction': gap})
    if gap < 0.10:
        return base | {'reason': 'diffuse/ambiguous sidecar weak-axis population'}
    return base | {'valid': True, 'weak_xyz_map': [float(v) for v in basis[:, -1]],
                   'reason': None,
                   'method': 'principal eigenvector of mean(v*v^T), axial sign invariant'}


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
        'translation_mapping_weak_consensus': mapping_weak_consensus(
            data.geom, voxel, 'translation', min_normals),
        'rotation_mapping_weak_consensus': mapping_weak_consensus(
            data.geom, voxel, 'rotation', min_normals),
        'qt_query_local': qt,
        'qr_query_conditioned': qr,
    }


def _project_recovery(axis: list[float] | None, initial: np.ndarray,
                      final: np.ndarray | None, scale: float) -> dict:
    result = {'alignment_abs_cos': None, 'initial_projection': None,
              'final_projection': None, 'recovery': None}
    if axis is None or float(np.linalg.norm(initial)) < 1e-12:
        return result
    weak = np.asarray(axis, dtype='f8')
    before = abs(float(np.dot(initial, weak)))
    result['alignment_abs_cos'] = before / float(np.linalg.norm(initial))
    result['initial_projection'] = before * scale
    if final is not None:
        after = abs(float(np.dot(final, weak)))
        result['final_projection'] = after * scale
        result['recovery'] = (before - after) * scale
    return result


def alignment_and_recovery(reference_t: np.ndarray, initial_t: np.ndarray,
                           result_t: np.ndarray | None, geometry: dict,
                           yaw_perturb_deg: float, *,
                           reference_xyzw: np.ndarray | None = None,
                           result_xyzw: np.ndarray | None = None) -> dict:
    """Map-frame perturbations vs both mapping-view and query-view weak axes.

    Rotation vectors are map-frame left differences. A positive recovery is a
    decrease in ABSOLUTE weak-axis error; it is not proof of correct basin.
    Missing result orientation, uninformative axis, or zero perturbation gives
    null rather than an invented rotation recovery.
    """
    translation = np.asarray(initial_t, dtype='f8') - reference_t
    final_t = None if result_t is None else np.asarray(result_t, dtype='f8') - reference_t
    initial_rot = np.array([0., 0., np.deg2rad(yaw_perturb_deg)])
    final_rot = None
    if reference_xyzw is not None and result_xyzw is not None:
        final_rot = (Rotation.from_quat(result_xyzw)
                     * Rotation.from_quat(reference_xyzw).inv()).as_rotvec()
    axes = {
        'translation_query': geometry['qt_query_local']['weak_xyz_map'],
        'translation_mapping': geometry['translation_mapping_weak_consensus']['weak_xyz_map'],
        'rotation_query': geometry['qr_query_conditioned']['weak_xyz_map'],
        'rotation_mapping': geometry['rotation_mapping_weak_consensus']['weak_xyz_map'],
    }
    output = {}
    for label, axis in axes.items():
        is_rot = label.startswith('rotation_')
        projection = _project_recovery(axis, initial_rot if is_rot else translation,
                                       final_rot if is_rot else final_t,
                                       180 / np.pi if is_rot else 1.)
        unit = 'deg' if is_rot else 'm'
        output[f'{label}_weak_alignment_abs_cos'] = projection['alignment_abs_cos']
        output[f'{label}_initial_weak_error_{unit}'] = projection['initial_projection']
        output[f'{label}_final_weak_error_{unit}'] = projection['final_projection']
        output[f'{label}_weak_recovery_{unit}'] = projection['recovery']
    # Original explicit query-conditioned aliases, kept for existing readers.
    output['translation_perturb_weak_axis_abs_cos'] = output['translation_query_weak_alignment_abs_cos']
    output['yaw_axis_vs_rotation_weak_abs_cos'] = output['rotation_query_weak_alignment_abs_cos']
    output['initial_weak_projection_error_m'] = output['translation_query_initial_weak_error_m']
    output['final_weak_projection_error_m'] = output['translation_query_final_weak_error_m']
    output['weak_axis_recovery_m'] = output['translation_query_weak_recovery_m']
    output['strong_plane_recovery_m'] = None
    weak = axes['translation_query']
    if weak is not None and final_t is not None and float(np.linalg.norm(translation)) > 1e-12:
        w = np.asarray(weak, dtype='f8')
        output['strong_plane_recovery_m'] = (
            float(np.linalg.norm(translation - np.dot(translation, w) * w))
            - float(np.linalg.norm(final_t - np.dot(final_t, w) * w)))
    return output
