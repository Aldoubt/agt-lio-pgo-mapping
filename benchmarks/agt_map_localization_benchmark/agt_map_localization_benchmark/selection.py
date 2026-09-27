"""Predeclared non-publishing map selections and density-matched controls."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .dataset import PgoEvidence, point_keys, lookup_sorted_keys
from .pcd import sha256_file, write_pcd

SWEEP_QUANTILES = (0.25, 0.50, 0.75)  # declared before examining localization outcomes
CONTROL_SEED = 20260927
COVERAGE_CELL_M = 1.0


@dataclass
class Candidate:
    name: str
    indices: np.ndarray
    path: Path
    detail: dict
    coverage: dict


def xy_cells(points: np.ndarray, step: float = COVERAGE_CELL_M) -> np.ndarray:
    return np.floor(np.asarray(points)[:, :2] / np.float32(step)).astype('<i8')


def spatial_coverage(coords: np.ndarray, reference_cells: set[tuple[int, int]]) -> dict:
    if not len(coords):
        raise ValueError('empty map candidate')
    occupied = set(map(tuple, xy_cells(coords)))
    min_xyz = np.min(coords, axis=0)
    max_xyz = np.max(coords, axis=0)
    return {
        'points': int(len(coords)),
        'occupied_xy_1m_cells': len(occupied),
        'fraction_of_raw_xy_1m_cells': len(occupied & reference_cells) / len(reference_cells),
        'min_xyz_m': [float(v) for v in min_xyz],
        'max_xyz_m': [float(v) for v in max_xyz],
        'xy_bounding_box_area_m2': float(np.prod(max_xyz[:2] - min_xyz[:2])),
        'z_span_m': float(max_xyz[2] - min_xyz[2]),
    }


def voxel_round_robin(indices: np.ndarray, coords: np.ndarray, count: int, seed: int) -> np.ndarray:
    """Evenly pick occupied 1m XY cells, without replacement, from B only."""
    if count < 1 or count > len(indices):
        raise ValueError('control count must fit source map')
    cell = xy_cells(coords[indices])
    _unique, group = np.unique(cell, axis=0, return_inverse=True)
    rng = np.random.default_rng(seed)
    random_order = rng.permutation(len(indices))
    in_group_order = np.argsort(group[random_order], kind='stable')
    groups_sorted = group[random_order][in_group_order]
    starts = np.r_[0, np.flatnonzero(np.diff(groups_sorted)) + 1]
    lengths = np.diff(np.r_[starts, len(indices)])
    tier = np.arange(len(indices)) - np.repeat(starts, lengths)
    fairness_order = np.lexsort((groups_sorted, tier))
    return np.sort(indices[random_order[in_group_order[fairness_order[:count]]]])


def make_candidates(data: PgoEvidence, map_indices: tuple[int, ...], output: Path,
                    *, controls_all_quantiles: bool = True) -> tuple[list[Candidate], dict]:
    """All maps use ONLY the supplied map-index slices of the immutable PGO map.

    B/C are filtered *raw map-subset points* using verified V1/reviewed voxel
    predicates. They are not byte-identical copies of full-session stable_map.pcd.
    """
    points, intensity = data.raw_map_subset(map_indices)
    keys = point_keys(points, data.voxel_size)
    lookup, found = lookup_sorted_keys(data.keys, keys)
    missing = int((~found).sum())
    if missing > max(20, int(0.01 * len(points))):
        raise ValueError(f'{missing} map points lack full-session evidence keys')
    bmask = found & data.v1_stable[lookup]
    cmask = found & data.reviewed_stable[lookup] if data.rev is not None else None
    b = np.flatnonzero(bmask)
    if len(b) < 1000:
        raise ValueError(f'AUTO_STABLE map subset too sparse: {len(b)} points')
    raw_cells = set(map(tuple, xy_cells(points)))
    cands = []
    output.mkdir(parents=True, exist_ok=False)

    def add(name: str, index: np.ndarray, detail: dict) -> None:
        chosen = np.sort(np.asarray(index, dtype='<i8'))
        if not len(chosen) or len(np.unique(chosen)) != len(chosen):
            raise ValueError(f'empty/duplicate candidate: {name}')
        dest = output / f'{name}.pcd'
        write_pcd(dest, points[chosen], intensity[chosen])
        stats = spatial_coverage(points[chosen], raw_cells)
        stats.update({'pcd_sha256': sha256_file(dest),
                      'source_map_keyframe_count': len(map_indices)})
        cands.append(Candidate(name, chosen, dest, detail, stats))

    add('A_RAW', np.arange(len(points)), {'criterion': 'all finite map-subset PGO points'})
    add('B_AUTO_STABLE', b, {'criterion': 'V1 AUTO stable predicate, raw map-subset points'})
    if cmask is not None:
        add('C_REVIEWED_STABLE', np.flatnonzero(cmask),
            {'criterion': 'Phase 2B reviewed stable predicate, raw map-subset points'})
    eligible = b[data.geom['translation_valid'][lookup[b]] != 0]
    eligible = eligible[np.isfinite(data.geom['translation_q'][lookup[eligible]])]
    # One vote per V1 voxel when defining cutoffs, not per raw point density.
    vxl = np.unique(lookup[eligible])
    if len(vxl) < 12:
        raise ValueError('insufficient eligible unique stable voxels for Qt sweep')
    qt_dist = np.asarray(data.geom['translation_q'][vxl], dtype='f8')
    cuts = np.quantile(qt_dist, SWEEP_QUANTILES)
    if not np.isfinite(cuts).all():
        raise ValueError('Qt sweep thresholds nonfinite')
    extra = {
        'unmapped_raw_points': missing,
        'unmapped_raw_fraction': missing / len(points),
        'eligible_unique_auto_stable_voxels': int(len(vxl)),
        'eligible_auto_stable_points': int(len(eligible)),
        'qt_quantiles_declared': list(SWEEP_QUANTILES),
        'cutoffs': {},
        'control_source': 'B_AUTO_STABLE',
        'control_seed': CONTROL_SEED,
        'coverage_grid_xy_m': COVERAGE_CELL_M,
    }
    for q, cutoff in zip(SWEEP_QUANTILES, cuts):
        label = f'q{int(q * 100):02d}'
        qt_map = np.asarray(data.geom['translation_q'][lookup[eligible]], dtype='f8')
        selected = eligible[qt_map >= cutoff]
        extra['cutoffs'][label] = float(cutoff)
        add(f'D_QT_{label}', selected,
            {'criterion': 'B and valid geometry_v1 Qt >= predeclared unique-voxel quantile',
             'quantile': q, 'cutoff': float(cutoff), 'note': 'not confidence_v2; no probability'})
        if controls_all_quantiles or q == 0.50:
            rng = np.random.default_rng(CONTROL_SEED + int(q * 100))
            random = np.sort(rng.choice(b, size=len(selected), replace=False))
            add(f'CONTROL_RANDOM_{label}', random,
                {'criterion': 'deterministic equal-point random subset of B',
                 'target': f'D_QT_{label}', 'seed': CONTROL_SEED + int(q * 100)})
            balanced = voxel_round_robin(b, points, len(selected), CONTROL_SEED + 1000 + int(q * 100))
            add(f'CONTROL_VOXEL_{label}', balanced,
                {'criterion': 'deterministic spatially stratified equal-point subset of B (1m XY)',
                 'target': f'D_QT_{label}', 'seed': CONTROL_SEED + 1000 + int(q * 100)})
    return cands, {'points': points, 'lookups': lookup, 'found': found,
                   'voxel_keys': keys, 'meta': extra}
