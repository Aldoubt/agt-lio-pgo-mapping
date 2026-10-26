import numpy as np
import pytest
from shapely.geometry import Polygon

from cross_stage_analysis.pipeline import _inward_buffer
from cross_stage_analysis.p3 import _occupancy_metrics, _roi_cell_mask, _shared_grid_geometry


def test_navigation_interior_shrink_is_parameterized_and_moves_edges_inward():
    boundary = Polygon([[0, 0], [10, 0], [10, 8], [0, 8]])
    one_meter = _inward_buffer(boundary, 1.0)
    two_meters = _inward_buffer(boundary, 2.0)
    assert one_meter.area == pytest.approx(48.0)
    assert two_meters.area == pytest.approx(24.0)
    assert two_meters.area < one_meter.area < boundary.area


def test_navigation_shrink_rejects_nonpositive_and_empty_interior():
    boundary = Polygon([[0, 0], [2, 0], [2, 2], [0, 2]])
    with pytest.raises(ValueError, match="greater than zero"):
        _inward_buffer(boundary, 0.0)
    with pytest.raises(ValueError, match="removes the entire"):
        _inward_buffer(boundary, 2.0)


def test_roi_grid_mask_uses_metric_cell_centers():
    polygon = {
        "type": "Polygon",
        "coordinates_xy_m": [[[-0.1, -0.1], [1.1, -0.1], [1.1, 1.1], [-0.1, 1.1], [-0.1, -0.1]]],
    }
    mask = _roi_cell_mask((2, 2), np.array([0.0, 0.0]), 1.0, polygon)
    np.testing.assert_array_equal(mask, [[True, False], [False, False]])


def test_shared_grid_extent_is_relative_to_negative_origin():
    points = np.array([[-1.1, -2.0, 0.0], [2.1, 1.9, 0.0]])
    origin, shape_xy = _shared_grid_geometry(points, 0.5)
    np.testing.assert_allclose(origin, [-1.5, -2.0])
    assert shape_xy == (8, 8)


def test_occupancy_iou_keeps_one_sided_cells_unknown_and_reports_both_unknown():
    reference = np.array([[1, 0, 2], [0, 4, 0]])
    sparse = np.array([[3, 0, 0], [5, 0, 0]])
    scope = np.ones((2, 3), dtype=bool)
    result = _occupancy_metrics(reference, sparse, scope, 1)
    assert result["intersection_occupied_cells"] == 1
    assert result["union_occupied_cells"] == 4
    assert result["observed_occupancy_iou"] == pytest.approx(0.25)
    assert result["reference_only_observed_occupied_cells_unknown_in_sparse"] == 2
    assert result["sparse_only_observed_occupied_cells_unknown_in_reference"] == 1
    assert result["unknown_in_both_cells"] == 2
    assert result["single_period_only_cells_are_environmental_change"] is False


def test_occupancy_metrics_reject_inconsistent_grids():
    with pytest.raises(ValueError, match="share the same grid"):
        _occupancy_metrics(np.zeros((2, 2)), np.zeros((2, 3)), np.ones((2, 2), bool), 1)
