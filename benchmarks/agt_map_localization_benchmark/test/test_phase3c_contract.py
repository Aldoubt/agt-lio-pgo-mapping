import csv
from pathlib import Path

import numpy as np
import pytest

from agt_map_localization_benchmark.coverage_sampling import (
    coverage_field_rows, coverage_matched_subsets, make_quota_plan, xy_cell_keys,
)
from agt_map_localization_benchmark.quality_analysis import (
    analyse_phase3c, leave_one_center_out, write_coverage_field,
)
from agt_map_localization_benchmark.quality_features import (
    NATIVE_MIN_MAP_POINTS, local_crop_support, localization_quality_features,
    safe_log_condition,
)


def _fixture():
    # Four occupied 1m cells, each with four 0.2m evidence voxels and ten
    # deterministic raw points per voxel.
    coords = []
    lookup = []
    voxel = 0
    for cx in range(4):
        for vx in range(4):
            for k in range(10):
                coords.append([cx + 0.05 + 0.2 * vx, 0.05 + 0.002 * k, 0.0])
                lookup.append(voxel)
            voxel += 1
    coords = np.asarray(coords, dtype='f4')
    lookup = np.asarray(lookup, dtype='<i8')
    geom = np.zeros(voxel, dtype=[
        ('translation_valid', 'u4'), ('translation_q', 'f4'),
        ('normal_valid', 'u4'),
    ])
    geom['translation_valid'] = geom['normal_valid'] = 1
    geom['translation_q'] = np.linspace(.01, .99, voxel)
    indices = np.arange(len(coords), dtype='<i8')
    return coords, lookup, geom, indices


def test_coverage_quota_is_per_cell_deterministic_and_preserves_every_cell():
    coords, lookup, geom, source = _fixture()
    plan = make_quota_plan(source, coords, .25)
    first, meta = coverage_matched_subsets(source, coords, lookup, geom, .25, seed=20260927)
    second, _ = coverage_matched_subsets(source, coords, lookup, geom, .25, seed=20260927)
    assert plan.source_cells == 4
    assert meta['same_cell_set_and_point_quota'] is True
    source_cells = set(map(tuple, xy_cell_keys(coords[source])))
    for name in ('geometry', 'random', 'uniform'):
        assert np.array_equal(first[name], second[name])
        assert len(first[name]) == plan.target_points
        assert len(np.unique(first[name])) == len(first[name])
        assert set(map(tuple, xy_cell_keys(coords[first[name]]))) == source_cells
    # Exact quota is matched cell-by-cell, not merely globally.
    for cell in source_cells:
        expected = plan.quotas[cell]
        for selected in first.values():
            cells = xy_cell_keys(coords[selected])
            assert int(np.sum(np.all(cells == np.asarray(cell), axis=1))) == expected


def test_geometry_sampler_prefers_high_qt_inside_each_cell_without_dropping_cell():
    coords, lookup, geom, source = _fixture()
    selected, _ = coverage_matched_subsets(source, coords, lookup, geom, .25)
    geometry_voxels = lookup[selected['geometry']]
    random_voxels = lookup[selected['random']]
    assert float(np.median(geom['translation_q'][geometry_voxels])) >= float(
        np.median(geom['translation_q'][random_voxels]))
    assert len(set(map(tuple, xy_cell_keys(coords[selected['geometry']])))) == 4


def test_coverage_field_records_same_cells_and_candidate_counts(tmp_path):
    coords, lookup, geom, source = _fixture()
    selected, _ = coverage_matched_subsets(source, coords, lookup, geom, .5)
    rows = coverage_field_rows(coords, source, {
        'D_COVERAGE_QT_q50': selected['geometry'],
        'CONTROL_CELL_RANDOM_q50': selected['random'],
    }, lookup, geom)
    assert len(rows) == 4
    assert all(row['B_points'] == 40 for row in rows)
    assert all(row['D_COVERAGE_QT_q50_points'] == row['CONTROL_CELL_RANDOM_q50_points']
               for row in rows)
    path = tmp_path / 'coverage_field.csv'
    write_coverage_field(path, rows)
    with path.open(newline='') as stream:
        loaded = list(csv.DictReader(stream))
    assert len(loaded) == 4


def test_local_crop_support_uses_reference_center_and_baseline_ratios():
    coords, lookup, geom, source = _fixture()
    found = np.ones(len(coords), dtype=bool)
    candidate = source[::2]
    support = local_crop_support(
        candidate, source, coords, lookup, found, geom, np.array([1.5, 0., 0.]),
        radius_xy=10., half_height=5.)
    assert support['crop_map_points'] == len(candidate)
    assert support['baseline_B_crop_points'] == len(source)
    assert support['crop_support_ratio'] == pytest.approx(.5)
    assert support['fraction_of_B_crop_cells'] == pytest.approx(1.)
    assert support['native_min_map_points'] == NATIVE_MIN_MAP_POINTS
    assert support['insufficient_map_points_at_reference_crop'] is True


def test_quality_features_keep_invalid_spectrum_null_and_log_safe():
    support = {'valid_normal_voxels': 10, 'crop_map_points': 900,
               'crop_1m_cells': 4, 'crop_support_ratio': .75}
    geometry = {
        'qt_query_local': {'valid': True, 'q': .2, 'eigenvalues': [.1, .4, .5]},
        'qr_query_conditioned': {'valid': False, 'q': None, 'eigenvalues': None},
    }
    features = localization_quality_features(geometry, support)
    assert features['Qt_query'] == pytest.approx(.2)
    assert features['Qr_query'] is None
    assert features['rotation_condition'] is None
    assert features['log_rotation_condition'] is None
    assert safe_log_condition(None) is None
    assert safe_log_condition(0.) is None


def test_phase3c_analysis_groups_query_centers_and_never_random_row_splits():
    rows = []
    for center in (175, 350):
        for candidate, qr, support in (
            ('B_AUTO_STABLE', .2, 1.0),
            ('D_COVERAGE_QT_q50', .4, .55),
            ('CONTROL_CELL_RANDOM_q50', .3, .55),
        ):
            for i in range(30):
                success = (i % 5) < (4 if center == 175 else 3)
                rows.append({
                    'tier': 'tier1', 'algorithm': 'LOCAL', 'candidate': candidate,
                    'center': center, 'frames': 1, 'nominal_success': success,
                    'translation_3d_error_m': .1 if success else 1.2,
                    'yaw_error_deg': 1. if success else 12.,
                    'failure_code': None if success else 'NO_CONVERGENCE',
                    'quality_features': {
                        'Qt_query': qr / 2, 'Qr_query': qr,
                        'log_translation_condition': 2., 'log_rotation_condition': 2.5,
                        'valid_normal_voxels': 50, 'crop_map_points': 2000 * support,
                        'crop_occupied_cells': 100 * support, 'crop_support_ratio': support,
                    },
                    'local_support': {'crop_support_ratio': support},
                })
    summary = analyse_phase3c(rows)
    assert summary['group_count'] == 6
    loco = leave_one_center_out(rows)
    assert loco['status'] == 'EXPLORATORY'
    assert loco['groups'] == [175, 350]
    assert {fold['test_center'] for fold in loco['folds']} == {175, 350}
    assert 'NOT calibrated' in loco['note']
