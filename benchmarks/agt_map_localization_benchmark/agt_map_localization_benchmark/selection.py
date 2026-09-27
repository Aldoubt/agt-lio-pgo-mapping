"""Predeclared non-publishing map selections and density/coverage-matched controls."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .coverage_sampling import (
    COVERAGE_CELL_M, COVERAGE_FRACTIONS, CONTROL_SEED,
    MIN_POINTS_PER_CELL, MIN_VOXELS_PER_CELL,
    cell_distribution, coverage_field_rows, coverage_matched_subsets, xy_cell_keys,
)
from .dataset import PgoEvidence, point_keys, lookup_sorted_keys
from .pcd import sha256_file, write_pcd

SWEEP_QUANTILES = (0.25, 0.50, 0.75)  # declared before examining localization outcomes


@dataclass
class Candidate:
    name: str
    indices: np.ndarray
    path: Path
    detail: dict
    coverage: dict


def xy_cells(points: np.ndarray, step: float = COVERAGE_CELL_M) -> np.ndarray:
    """Backward-compatible Phase 3B alias."""
    return xy_cell_keys(points, step)


def spatial_coverage(coords: np.ndarray, reference_cells: set[tuple[int, int]],
                     reference_xyz_cells: set[tuple[int, int, int]]) -> dict:
    if not len(coords):
        raise ValueError('empty map candidate')
    occupied = set(map(tuple, xy_cells(coords)))
    xyz_occupied = set(map(tuple, np.floor(np.asarray(coords) / np.float32(COVERAGE_CELL_M))
                           .astype('<i8')))
    min_xyz = np.min(coords, axis=0)
    max_xyz = np.max(coords, axis=0)
    return {
        'points': int(len(coords)),
        'occupied_xy_1m_cells': len(occupied),
        'occupied_xyz_1m_cells': len(xyz_occupied),
        'fraction_of_raw_xy_1m_cells': len(occupied & reference_cells) / len(reference_cells),
        'fraction_of_raw_xyz_1m_cells': len(xyz_occupied & reference_xyz_cells) / len(reference_xyz_cells),
        'points_per_occupied_xyz_1m_cell': len(coords) / len(xyz_occupied),
        'min_xyz_m': [float(v) for v in min_xyz],
        'max_xyz_m': [float(v) for v in max_xyz],
        'xy_bounding_box_area_m2': float(np.prod(max_xyz[:2] - min_xyz[:2])),
        'z_span_m': float(max_xyz[2] - min_xyz[2]),
    }


def voxel_round_robin(indices: np.ndarray, coords: np.ndarray, count: int, seed: int) -> np.ndarray:
    """Legacy Phase 3B global 1m-XY balancing control."""
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
    """Build Phase 3B negative controls plus Phase 3C coverage-preserving maps.

    All maps use ONLY the supplied map-index slices of the immutable PGO map.
    B/C are filtered *raw map-subset points* using verified V1/reviewed voxel
    predicates. Phase 3C coverage candidates all start from B_AUTO_STABLE.
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
    raw_xyz_cells = set(map(tuple, np.floor(points / np.float32(COVERAGE_CELL_M))
                            .astype('<i8')))
    cands: list[Candidate] = []
    output.mkdir(parents=True, exist_ok=False)

    def add(name: str, index: np.ndarray, detail: dict) -> None:
        chosen = np.sort(np.asarray(index, dtype='<i8'))
        if not len(chosen) or len(np.unique(chosen)) != len(chosen):
            raise ValueError(f'empty/duplicate candidate: {name}')
        dest = output / f'{name}.pcd'
        write_pcd(dest, points[chosen], intensity[chosen])
        stats = spatial_coverage(points[chosen], raw_cells, raw_xyz_cells)
        if name != 'A_RAW':
            stats.update(cell_distribution(chosen, points, b, cell_size_m=COVERAGE_CELL_M))
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
    # Phase 3B global Qt filtering is retained as a declared negative control.
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
        'coverage_fractions_predeclared': list(COVERAGE_FRACTIONS),
        'coverage_min_points_per_cell': MIN_POINTS_PER_CELL,
        'coverage_min_voxels_per_cell': MIN_VOXELS_PER_CELL,
        'coverage_candidates': {},
    }
    for q, cutoff in zip(SWEEP_QUANTILES, cuts):
        label = f'q{int(q * 100):02d}'
        qt_map = np.asarray(data.geom['translation_q'][lookup[eligible]], dtype='f8')
        selected = eligible[qt_map >= cutoff]
        extra['cutoffs'][label] = float(cutoff)
        add(f'D_QT_{label}', selected,
            {'criterion': 'B and valid geometry_v1 Qt >= predeclared unique-voxel quantile',
             'quantile': q, 'cutoff': float(cutoff),
             'note': 'Phase 3B global-filter negative control; not confidence_v2'})
        if controls_all_quantiles or q == 0.50:
            rng = np.random.default_rng(CONTROL_SEED + int(q * 100))
            random = np.sort(rng.choice(b, size=len(selected), replace=False))
            add(f'CONTROL_RANDOM_{label}', random,
                {'criterion': 'legacy deterministic equal-point random subset of B',
                 'target': f'D_QT_{label}', 'seed': CONTROL_SEED + int(q * 100)})
            balanced = voxel_round_robin(b, points, len(selected), CONTROL_SEED + 1000 + int(q * 100))
            add(f'CONTROL_VOXEL_{label}', balanced,
                {'criterion': 'legacy global 1m-XY stratified equal-point subset of B',
                 'target': f'D_QT_{label}', 'seed': CONTROL_SEED + 1000 + int(q * 100)})

    # Phase 3C: lock coverage-cell set AND exact point quota per source cell.
    for fraction in COVERAGE_FRACTIONS:
        label = f'q{int(fraction * 100):02d}'
        subsets, meta = coverage_matched_subsets(
            b, points, lookup, data.geom, fraction,
            cell_size_m=COVERAGE_CELL_M,
            min_points_per_cell=MIN_POINTS_PER_CELL,
            min_voxels_per_cell=MIN_VOXELS_PER_CELL,
            seed=CONTROL_SEED + 2000 + int(fraction * 100))
        extra['coverage_candidates'][label] = meta
        add(f'D_COVERAGE_QT_{label}', subsets['geometry'],
            {'criterion': 'B per-1m-cell quota, geometry_v1 Qt-ranked voxels within each cell',
             **meta, 'note': 'coverage-preserving experimental sampler; not confidence_v2'})
        add(f'CONTROL_CELL_RANDOM_{label}', subsets['random'],
            {'criterion': 'same B cell set and exact per-cell point quota; deterministic random',
             'target': f'D_COVERAGE_QT_{label}', **meta})
        add(f'CONTROL_CELL_UNIFORM_{label}', subsets['uniform'],
            {'criterion': 'same B cell set and exact per-cell point quota; voxel round-robin',
             'target': f'D_COVERAGE_QT_{label}', **meta})

    candidate_indices = {candidate.name: candidate.indices for candidate in cands}
    field = coverage_field_rows(points, b, candidate_indices, lookup, data.geom,
                                cell_size_m=COVERAGE_CELL_M)
    return cands, {'points': points, 'lookups': lookup, 'found': found,
                   'voxel_keys': keys, 'baseline_B_indices': b,
                   'coverage_field': field, 'meta': extra}
