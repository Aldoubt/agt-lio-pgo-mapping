"""Coverage-preserving geometry sampling for Phase 3C offline experiments.

This module never writes production maps.  It receives indices into the immutable
Tier map-subset point array and returns deterministic experimental subsets.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

COVERAGE_CELL_M = 1.0
COVERAGE_FRACTIONS = (0.25, 0.50, 0.75)
MIN_POINTS_PER_CELL = 4
MIN_VOXELS_PER_CELL = 1
CONTROL_SEED = 20260927


@dataclass(frozen=True)
class CellQuotaPlan:
    cell_size_m: float
    fraction: float
    source_cells: int
    target_points: int
    quotas: dict[tuple[int, int], int]


def xy_cell_keys(points: np.ndarray, step: float = COVERAGE_CELL_M) -> np.ndarray:
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] < 2 or not np.isfinite(points[:, :2]).all():
        raise ValueError('coverage points must be finite Nx>=2')
    if not np.isfinite(step) or step <= 0:
        raise ValueError('coverage cell size must be positive')
    return np.floor(points[:, :2] / np.float32(step)).astype('<i8')


def _groups(indices: np.ndarray, coords: np.ndarray, step: float) -> dict[tuple[int, int], np.ndarray]:
    chosen = np.asarray(indices, dtype='<i8')
    if chosen.ndim != 1 or len(chosen) == 0 or len(np.unique(chosen)) != len(chosen):
        raise ValueError('coverage source indices must be nonempty and unique')
    if np.any(chosen < 0) or np.any(chosen >= len(coords)):
        raise ValueError('coverage source index outside point array')
    cells = xy_cell_keys(np.asarray(coords)[chosen], step)
    result: dict[tuple[int, int], list[int]] = {}
    for index, cell in zip(chosen.tolist(), cells.tolist()):
        result.setdefault((int(cell[0]), int(cell[1])), []).append(int(index))
    return {key: np.asarray(value, dtype='<i8') for key, value in sorted(result.items())}


def make_quota_plan(indices: np.ndarray, coords: np.ndarray, fraction: float, *,
                    cell_size_m: float = COVERAGE_CELL_M,
                    min_points_per_cell: int = MIN_POINTS_PER_CELL) -> CellQuotaPlan:
    if not np.isfinite(fraction) or not (0.0 < fraction <= 1.0):
        raise ValueError('coverage fraction must be in (0, 1]')
    if min_points_per_cell < 1:
        raise ValueError('min_points_per_cell must be >= 1')
    grouped = _groups(indices, coords, cell_size_m)
    quotas = {
        cell: min(len(source), max(min_points_per_cell, int(np.ceil(fraction * len(source)))))
        for cell, source in grouped.items()
    }
    return CellQuotaPlan(cell_size_m, float(fraction), len(grouped),
                         int(sum(quotas.values())), quotas)


def _voxel_ranked(source: np.ndarray, lookup: np.ndarray, geometry: np.ndarray,
                  quota: int, min_voxels_per_cell: int) -> np.ndarray:
    voxel_ids = np.asarray(lookup[source], dtype='<i8')
    unique = np.unique(voxel_ids)
    if len(unique) == 0:
        raise ValueError('coverage cell has no evidence voxels')
    valid = np.asarray(geometry['translation_valid'][unique] != 0)
    score = np.asarray(geometry['translation_q'][unique], dtype='f8')
    valid &= np.isfinite(score)
    # Valid high-Qt voxels first; deterministic voxel id breaks ties.
    order = sorted(range(len(unique)),
                   key=lambda i: (not bool(valid[i]),
                                  -float(score[i]) if valid[i] else 0.0,
                                  int(unique[i])))
    ranked = unique[np.asarray(order, dtype='<i8')]
    by_voxel = {int(v): np.sort(source[voxel_ids == v]) for v in ranked}
    selected: list[int] = []
    represented: set[int] = set()
    # Guarantee the configured minimum distinct-voxel representation when the
    # quota/cell supports it, before filling from the highest-ranked voxels.
    for voxel in ranked[:min(min_voxels_per_cell, quota, len(ranked))]:
        selected.append(int(by_voxel[int(voxel)][0]))
        represented.add(int(voxel))
    for voxel in ranked:
        for index in by_voxel[int(voxel)]:
            if len(selected) >= quota:
                break
            value = int(index)
            if value not in selected:
                selected.append(value)
                represented.add(int(voxel))
        if len(selected) >= quota:
            break
    if len(selected) != quota:
        raise AssertionError('geometry cell sampler failed exact quota')
    return np.asarray(sorted(selected), dtype='<i8')


def _seed_for_cell(seed: int, cell: tuple[int, int]) -> np.random.Generator:
    words = [int(seed) & 0xffffffff, int(cell[0]) & 0xffffffff, int(cell[1]) & 0xffffffff]
    return np.random.default_rng(np.random.SeedSequence(words))


def _uniform_voxel_round_robin(source: np.ndarray, lookup: np.ndarray, quota: int) -> np.ndarray:
    voxel_ids = np.asarray(lookup[source], dtype='<i8')
    voxels = np.unique(voxel_ids)
    groups = [np.sort(source[voxel_ids == voxel]) for voxel in voxels]
    chosen: list[int] = []
    depth = 0
    while len(chosen) < quota:
        progressed = False
        for group in groups:
            if depth < len(group):
                chosen.append(int(group[depth]))
                progressed = True
                if len(chosen) == quota:
                    break
        if not progressed:
            break
        depth += 1
    if len(chosen) != quota:
        raise AssertionError('uniform cell sampler failed exact quota')
    return np.asarray(sorted(chosen), dtype='<i8')


def coverage_matched_subsets(indices: np.ndarray, coords: np.ndarray, lookup: np.ndarray,
                             geometry: np.ndarray, fraction: float, *,
                             cell_size_m: float = COVERAGE_CELL_M,
                             min_points_per_cell: int = MIN_POINTS_PER_CELL,
                             min_voxels_per_cell: int = MIN_VOXELS_PER_CELL,
                             seed: int = CONTROL_SEED) -> tuple[dict[str, np.ndarray], dict]:
    """Return geometry/random/uniform subsets with identical per-cell point quotas."""
    if min_voxels_per_cell < 1:
        raise ValueError('min_voxels_per_cell must be >= 1')
    grouped = _groups(indices, coords, cell_size_m)
    plan = make_quota_plan(indices, coords, fraction, cell_size_m=cell_size_m,
                           min_points_per_cell=min_points_per_cell)
    geometry_parts: list[np.ndarray] = []
    random_parts: list[np.ndarray] = []
    uniform_parts: list[np.ndarray] = []
    fallback_cells = 0
    for cell, source in grouped.items():
        quota = plan.quotas[cell]
        voxel_ids = np.unique(np.asarray(lookup[source], dtype='<i8'))
        valid = ((geometry['translation_valid'][voxel_ids] != 0)
                 & np.isfinite(np.asarray(geometry['translation_q'][voxel_ids], dtype='f8')))
        if not bool(np.any(valid)):
            fallback_cells += 1
        geometry_parts.append(_voxel_ranked(source, lookup, geometry, quota,
                                            min_voxels_per_cell))
        rng = _seed_for_cell(seed, cell)
        random_parts.append(np.sort(rng.choice(source, size=quota, replace=False)).astype('<i8'))
        uniform_parts.append(_uniform_voxel_round_robin(source, lookup, quota))

    def merge(parts: Iterable[np.ndarray]) -> np.ndarray:
        result = np.sort(np.concatenate(list(parts))).astype('<i8')
        if len(result) != plan.target_points or len(np.unique(result)) != len(result):
            raise AssertionError('coverage sampler output size/uniqueness mismatch')
        return result

    subsets = {
        'geometry': merge(geometry_parts),
        'random': merge(random_parts),
        'uniform': merge(uniform_parts),
    }
    # All three controls must occupy the exact same source XY cells by design.
    source_cells = set(grouped)
    for name, selected in subsets.items():
        cells = set(map(tuple, xy_cell_keys(np.asarray(coords)[selected], cell_size_m)))
        if cells != source_cells:
            raise AssertionError(f'{name} control changed represented coverage cells')
    meta = {
        'cell_size_m': float(cell_size_m),
        'target_fraction_within_cell': float(fraction),
        'min_points_per_cell': int(min_points_per_cell),
        'min_voxels_per_cell': int(min_voxels_per_cell),
        'source_cells': int(plan.source_cells),
        'target_points': int(plan.target_points),
        'geometry_invalid_fallback_cells': int(fallback_cells),
        'seed': int(seed),
        'same_cell_set_and_point_quota': True,
    }
    return subsets, meta


def cell_distribution(indices: np.ndarray, coords: np.ndarray, source_indices: np.ndarray,
                      *, cell_size_m: float = COVERAGE_CELL_M) -> dict:
    source = _groups(source_indices, coords, cell_size_m)
    selected = _groups(indices, coords, cell_size_m)
    counts = np.asarray([len(selected.get(cell, ())) for cell in source], dtype='f8')
    nonzero = counts[counts > 0]
    return {
        'source_occupied_xy_cells': len(source),
        'represented_source_xy_cells': int(np.count_nonzero(counts)),
        'empty_source_cells': int(np.count_nonzero(counts == 0)),
        'median_points_per_source_cell': float(np.median(nonzero)) if len(nonzero) else None,
        'p05_points_per_source_cell': float(np.quantile(nonzero, .05)) if len(nonzero) else None,
        'p95_points_per_source_cell': float(np.quantile(nonzero, .95)) if len(nonzero) else None,
    }


def coverage_field_rows(coords: np.ndarray, baseline: np.ndarray, candidates: dict[str, np.ndarray],
                        lookup: np.ndarray, geometry: np.ndarray, *,
                        cell_size_m: float = COVERAGE_CELL_M) -> list[dict]:
    """Cell-level visualization table; no navigation/occupancy semantics."""
    grouped = _groups(baseline, coords, cell_size_m)
    candidate_cells = {
        name: _groups(indices, coords, cell_size_m) for name, indices in candidates.items()
    }
    rows: list[dict] = []
    for cell, source in grouped.items():
        voxels = np.unique(np.asarray(lookup[source], dtype='<i8'))
        qt_valid = ((geometry['translation_valid'][voxels] != 0)
                    & np.isfinite(np.asarray(geometry['translation_q'][voxels], dtype='f8')))
        qt = np.asarray(geometry['translation_q'][voxels[qt_valid]], dtype='f8')
        normal_valid = np.asarray(geometry['normal_valid'][voxels] != 0)
        row = {
            'cell_x': int(cell[0]), 'cell_y': int(cell[1]),
            'cell_size_m': float(cell_size_m),
            'B_points': int(len(source)),
            'Qt_median': float(np.median(qt)) if len(qt) else None,
            'valid_normals': int(np.count_nonzero(normal_valid)),
        }
        for name, mapping in candidate_cells.items():
            row[f'{name}_points'] = int(len(mapping.get(cell, ())))
        rows.append(row)
    return rows
