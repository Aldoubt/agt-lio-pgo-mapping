import numpy as np
import pytest

from agt_greenhouse_annotation import (
    FREE,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    derive_navigation_structure,
)
from agt_greenhouse_annotation.navigation_structure import _auto_row_direction


def _result(
    *,
    width=80,
    height=50,
    resolution=0.10,
    slope_x=0.0,
    obstacle_rows=(),
    terrain_rows=(),
    sparse_patch=False,
):
    columns = np.arange(width, dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)
    xx, yy = np.meshgrid((columns + 0.5) * resolution, (rows + 0.5) * resolution)
    ground = slope_x * xx
    for row in terrain_rows:
        center_y = (float(row) + 0.5) * resolution
        ground += 0.14 * np.exp(-((yy - center_y) ** 2) / (2.0 * 0.14**2))
    # One sharp cell should not dominate the scene-scale plane estimate.
    ground[height // 2, width // 2] += 0.12
    valid = np.ones((height, width), dtype=bool)
    point_count = np.full((height, width), 8, dtype=np.int32)
    ground_support = np.full((height, width), 6, dtype=np.int32)
    if sparse_patch:
        ground_support[10:18, 10:18] = 1
    obstacle_count = np.zeros((height, width), dtype=np.int32)
    for row in obstacle_rows:
        obstacle_count[row, 5:75] = 4
        # Short gaps should be structurally repairable.
        obstacle_count[row, 28:31] = 0
    occupancy = np.full((height, width), FREE, dtype=np.uint8)
    return NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground,
        ground_valid=valid,
        point_count=point_count,
        ground_support_count=ground_support,
        obstacle_count=obstacle_count,
        slope_deg=np.zeros_like(ground),
        step_m=np.zeros_like(ground),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            maximum_slope_deg=10.0,
            minimum_obstacle_points=2,
        ),
    )


def test_robust_local_plane_slope_tracks_scene_scale_plane():
    slope = np.tan(np.deg2rad(5.0))
    result = _result(slope_x=slope)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            robust_slope_window_m=0.50,
            row_direction_mode="provided",
        ),
        row_direction_xy=(1.0, 0.0),
    )
    interior = structure.robust_slope_deg[5:-5, 5:-5]
    finite = interior[np.isfinite(interior)]
    assert np.median(finite) == pytest.approx(5.0, abs=0.35)
    assert np.nanpercentile(finite, 95) < 7.0


def test_ground_confidence_penalizes_sparse_support():
    result = _result(sparse_patch=True)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(row_direction_mode="provided"),
        row_direction_xy=(1.0, 0.0),
    )
    dense = float(np.nanmedian(structure.ground_confidence[25:35, 25:35]))
    sparse = float(np.nanmedian(structure.ground_confidence[10:18, 10:18]))
    assert dense > 0.70
    assert sparse < dense * 0.40


def test_row_regularization_repairs_short_gaps_without_filling_whole_scene():
    result = _result(obstacle_rows=(15, 30))
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_half_width_m=0.16,
            row_max_gap_m=0.50,
            row_minimum_segment_length_m=1.50,
            row_minimum_spacing_m=0.80,
            row_minimum_prominence_ratio=0.05,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    assert len(structure.row_model.centers_v_m) >= 2
    assert np.any(structure.row_regularized_obstacle[14:17, 28:31])
    assert np.any(structure.row_regularized_obstacle[29:32, 28:31])
    assert not np.any(structure.row_regularized_obstacle[21:25, 20:60])


def test_bare_terrain_ridges_are_detected_without_vegetation_obstacles():
    terrain_rows = (8, 18, 28, 38)
    result = _result(terrain_rows=terrain_rows)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_minimum_spacing_m=0.70,
            row_minimum_prominence_ratio=0.05,
            row_terrain_background_sigma_m=0.45,
            row_terrain_prominence_scale_m=0.08,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    centers = np.asarray(structure.row_model.centers_v_m, dtype=np.float64)
    assert centers.size >= len(terrain_rows)
    for row in terrain_rows:
        expected_y = (float(row) + 0.5) * result.resolution_m
        assert np.min(np.abs(centers - expected_y)) < 0.25
        assert np.any(structure.row_regularized_obstacle[max(0, row - 2):row + 3, 10:70])


def test_aisle_candidate_requires_confident_ground_and_no_row_or_raw_obstacle():
    result = _result(obstacle_rows=(15, 30), sparse_patch=True)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_half_width_m=0.16,
            row_minimum_prominence_ratio=0.05,
            aisle_minimum_ground_confidence=0.30,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    assert np.any(structure.aisle_candidate[20:25, 35:55])
    assert not np.any(structure.aisle_candidate[14:17, 20:60])
    assert not np.any(structure.aisle_candidate[10:18, 10:18])


def test_unobserved_obstacle_evidence_cannot_create_crop_row_hypothesis():
    result = _result(width=100, height=70)
    # A real observed crop row and a fake obstacle stripe in an unobserved area.
    result.obstacle_count[25, 8:92] = 5
    result.obstacle_count[55, 8:92] = 5
    result.point_count[50:60, :] = 0
    result.ground_support_count[50:60, :] = 0
    result.ground_valid[50:60, :] = False

    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_minimum_spacing_m=0.50,
            row_minimum_prominence_ratio=0.04,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    centers = np.asarray(structure.row_model.centers_v_m, dtype=np.float64)
    assert np.min(np.abs(centers - 2.55)) < 0.25
    assert np.all(np.abs(centers - 5.55) > 0.35)


def test_double_edge_canopy_peaks_consolidate_to_one_row_hypothesis():
    result = _result(width=120, height=100)
    result.obstacle_count[:] = 0
    true_rows_m = (2.0, 4.0, 6.0, 8.0)
    # Each physical canopy has two strong longitudinal edges 0.60 m apart.
    # The row hypothesis should represent the structure once, near its center.
    for center_m in true_rows_m:
        center_row = int(round(center_m / result.resolution_m - 0.5))
        for edge_row in (center_row - 3, center_row + 3):
            result.obstacle_count[edge_row, 8:112] = 6

    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_profile_smoothing_m=0.08,
            row_minimum_spacing_m=0.45,
            row_minimum_prominence_ratio=0.03,
            row_hypothesis_merge_distance_m=0.80,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    centers = np.asarray(structure.row_model.centers_v_m, dtype=np.float64)
    assert centers.size == len(true_rows_m)
    for expected in true_rows_m:
        assert np.min(np.abs(centers - expected)) < 0.20


def test_auto_direction_ignores_sparse_horizontal_map_fringe():
    result = _result(width=100, height=100)
    evidence = np.zeros((result.height, result.width), dtype=np.float64)
    theta = np.deg2rad(22.0)
    for intercept in (18.0, 34.0, 50.0, 66.0):
        for column in range(10, 90):
            row = int(round(np.tan(theta) * column + intercept))
            if 0 <= row < result.height:
                evidence[row, column] = 1.0
                result.point_count[row, column] = 20
    # A long but poorly supported horizontal fringe would win the untrimmed
    # profile score. It should not override the dense parallel-row evidence.
    evidence[5, 5:95] = 1.0
    result.point_count[5, 5:95] = 1

    direction = _auto_row_direction(result, evidence)
    angle = np.rad2deg(np.arctan2(direction[1], direction[0]))
    assert angle == pytest.approx(22.0, abs=2.1)
