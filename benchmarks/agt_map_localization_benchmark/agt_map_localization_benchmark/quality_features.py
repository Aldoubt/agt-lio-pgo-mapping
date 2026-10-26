"""Per-query local support and non-calibrated localization-quality features.

The outputs are diagnostics only.  They do not modify confidence_v1, geometry_v1,
registration backends, maps, or product localization state.
"""
from __future__ import annotations

import math

import numpy as np

from .coverage_sampling import COVERAGE_CELL_M, xy_cell_keys

# Audited from agt_navigation_v3/navigation/localization/
# agt_global_relocalization_native/src/map_gicp_tracker.cpp:
# local.size() < 1000 -> "local map has fewer than 1000 points".
NATIVE_MIN_MAP_POINTS = 1000
NATIVE_MIN_MAP_POINTS_SOURCE = (
    'agt_navigation_v3/navigation/localization/'
    'agt_global_relocalization_native/src/map_gicp_tracker.cpp'
)


def _crop_indices(indices: np.ndarray, coords: np.ndarray, center: np.ndarray,
                  radius_xy: float, half_height: float) -> np.ndarray:
    selected = np.asarray(indices, dtype='<i8')
    delta = np.asarray(coords[selected], dtype='f8') - np.asarray(center, dtype='f8')
    mask = ((np.sum(delta[:, :2] ** 2, axis=1) <= float(radius_xy) ** 2)
            & (np.abs(delta[:, 2]) <= float(half_height)))
    return selected[mask]


def local_crop_support(candidate_indices: np.ndarray, baseline_indices: np.ndarray,
                       coords: np.ndarray, lookup: np.ndarray, found: np.ndarray,
                       geometry: np.ndarray, reference_t: np.ndarray, *,
                       radius_xy: float, half_height: float,
                       cell_size_m: float = COVERAGE_CELL_M,
                       native_min_map_points: int = NATIVE_MIN_MAP_POINTS) -> dict:
    """Describe candidate support in the same reference-centered crop used for diagnostics.

    The runtime itself crops around the *provided initial pose*.  This function stays
    anchored at the optimized reference pose so it is comparable across candidate
    maps and does not smuggle the supplied perturbation into a quality feature.
    """
    cand = _crop_indices(candidate_indices, coords, reference_t, radius_xy, half_height)
    base = _crop_indices(baseline_indices, coords, reference_t, radius_xy, half_height)

    def summary(indices: np.ndarray) -> dict:
        if not len(indices):
            return {'points': 0, 'voxels': 0, 'cells': 0, 'valid_normal_voxels': 0}
        valid_point = np.asarray(found[indices], dtype=bool)
        voxel = np.unique(np.asarray(lookup[indices[valid_point]], dtype='<i8'))
        normal_voxel = voxel[geometry['normal_valid'][voxel] != 0] if len(voxel) else voxel
        cells = set(map(tuple, xy_cell_keys(np.asarray(coords)[indices], cell_size_m)))
        return {'points': int(len(indices)), 'voxels': int(len(voxel)),
                'cells': int(len(cells)), 'valid_normal_voxels': int(len(normal_voxel))}

    c = summary(cand)
    b = summary(base)

    def ratio(a: int, denominator: int) -> float | None:
        return float(a / denominator) if denominator else None

    return {
        'reference_center': 'optimized_PGO_pose.query_body_origin',
        'radius_xy_m': float(radius_xy),
        'half_height_m': float(half_height),
        'cell_size_m': float(cell_size_m),
        'crop_map_points': c['points'],
        'crop_voxels': c['voxels'],
        'crop_1m_cells': c['cells'],
        'valid_normal_voxels': c['valid_normal_voxels'],
        'baseline_B_crop_points': b['points'],
        'baseline_B_crop_voxels': b['voxels'],
        'baseline_B_crop_cells': b['cells'],
        'fraction_of_B_crop_points': ratio(c['points'], b['points']),
        'fraction_of_B_crop_voxels': ratio(c['voxels'], b['voxels']),
        'fraction_of_B_crop_cells': ratio(c['cells'], b['cells']),
        'crop_support_ratio': ratio(c['points'], b['points']),
        'native_min_map_points': int(native_min_map_points),
        'native_min_map_points_source': NATIVE_MIN_MAP_POINTS_SOURCE,
        'insufficient_map_points_at_reference_crop': bool(c['points'] < native_min_map_points),
        'note': ('reference-centered diagnostic only; native LOCAL crop is centered on each '
                 'perturbed initial pose and remains unchanged'),
    }


def _spectrum_condition(spectrum: dict | None, epsilon: float) -> float | None:
    if not spectrum or not spectrum.get('valid'):
        return None
    eigen = spectrum.get('eigenvalues')
    if eigen is None or len(eigen) != 3:
        return None
    values = np.asarray(eigen, dtype='f8')
    if not np.isfinite(values).all() or values[-1] < 0:
        return None
    return float(values[-1] / max(float(values[0]), float(epsilon)))


def safe_log_condition(value: float | None) -> float | None:
    if value is None or not math.isfinite(float(value)) or float(value) <= 0:
        return None
    return float(math.log(float(value)))


def localization_quality_features(geometry: dict, support: dict, *,
                                  translation_epsilon: float = 1e-6,
                                  rotation_epsilon_m2: float = 1e-6) -> dict:
    """Build an experimental feature vector; never label it confidence/probability."""
    qt = geometry.get('qt_query_local', {})
    qr = geometry.get('qr_query_conditioned', {})
    kt = _spectrum_condition(qt, translation_epsilon)
    kr = _spectrum_condition(qr, rotation_epsilon_m2)
    return {
        'Qt_query': qt.get('q') if qt.get('valid') else None,
        'Qr_query': qr.get('q') if qr.get('valid') else None,
        'translation_condition': kt,
        'rotation_condition': kr,
        'log_translation_condition': safe_log_condition(kt),
        'log_rotation_condition': safe_log_condition(kr),
        'valid_normal_voxels': support.get('valid_normal_voxels'),
        'crop_map_points': support.get('crop_map_points'),
        'crop_occupied_cells': support.get('crop_1m_cells'),
        'crop_support_ratio': support.get('crop_support_ratio'),
        'feature_semantics': 'exploratory localization-quality diagnostics; not calibrated',
    }
