import numpy as np
import pytest
import yaml

from agt_greenhouse_annotation.point_cloud import NavigationPointCloud
from agt_greenhouse_annotation.navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    apply_navigation_overrides,
    derive_ground_relative_navigation_map,
    write_navigation_map_derivation,
)


def _cloud(points):
    return NavigationPointCloud(np.asarray(points, dtype=np.float64))


def _sloped_ground(*, add_obstacle=False):
    points = []
    for ix in range(20):
        for iy in range(12):
            x = 0.05 + ix * 0.10
            y = 0.05 + iy * 0.10
            ground = 0.08 * x + 0.02 * y
            for dz in (-0.01, 0.0, 0.01, 0.015):
                points.append((x, y, ground + dz))
    if add_obstacle:
        x, y = 1.05, 0.55
        ground = 0.08 * x + 0.02 * y
        for dz in (0.25, 0.28, 0.32, 0.36):
            points.append((x, y, ground + dz))
    return _cloud(points)


def _config(**updates):
    values = dict(
        resolution_m=0.10,
        padding_m=0.0,
        ground_quantile=0.10,
        minimum_cell_points=3,
        ground_seed_support_band_m=0.05,
        minimum_ground_seed_support_points=2,
        ground_continuity_radius_m=0.60,
        ground_reference_percentile=20.0,
        ground_seed_max_rise_m=0.10,
        ground_seed_max_drop_m=0.15,
        maximum_ground_fill_distance_m=0.15,
        ground_smoothing_radius_cells=1,
        ground_tolerance_m=0.05,
        minimum_ground_support_points=2,
        obstacle_min_height_m=0.12,
        obstacle_max_height_m=0.80,
        minimum_obstacle_points=2,
        maximum_slope_deg=20.0,
        maximum_step_m=0.15,
        obstacle_padding_m=0.0,
    )
    values.update(updates)
    return GroundRelativeNavigationConfig(**values)


def test_smooth_absolute_height_change_remains_free():
    result = derive_ground_relative_navigation_map(_sloped_ground(), _config())
    assert np.count_nonzero(result.occupancy == FREE) > 150
    assert np.count_nonzero(result.occupancy == OCCUPIED) == 0
    assert np.nanmax(result.ground_height_m) - np.nanmin(result.ground_height_m) > 0.10


def test_relative_obstacle_is_occupied_on_sloped_ground():
    result = derive_ground_relative_navigation_map(
        _sloped_ground(add_obstacle=True), _config()
    )
    column = int(np.floor((1.05 - result.origin_x_m) / result.resolution_m))
    row = int(np.floor((0.55 - result.origin_y_m) / result.resolution_m))
    assert result.obstacle_count[row, column] >= 2
    assert result.occupancy[row, column] == OCCUPIED


def test_raised_step_is_not_treated_as_new_flat_free_ground():
    points = []
    for ix in range(14):
        for iy in range(10):
            x = 0.05 + ix * 0.10
            y = 0.05 + iy * 0.10
            z0 = 0.0 if ix < 7 else 0.30
            for dz in (-0.01, 0.0, 0.01, 0.015):
                points.append((x, y, z0 + dz))
    result = derive_ground_relative_navigation_map(
        _cloud(points), _config(ground_smoothing_radius_cells=2, maximum_step_m=0.12)
    )
    assert np.count_nonzero(result.occupancy == OCCUPIED) > 0


def test_overhead_frame_without_ground_returns_cannot_become_ground():
    points = []
    bridge_columns = {9, 10}
    for ix in range(20):
        for iy in range(12):
            x = 0.05 + ix * 0.10
            y = 0.05 + iy * 0.10
            if ix not in bridge_columns:
                for dz in (-0.01, 0.0, 0.01, 0.015):
                    points.append((x, y, dz))
            else:
                for dz in (2.98, 3.00, 3.02, 3.03):
                    points.append((x, y, dz))
    result = derive_ground_relative_navigation_map(
        _cloud(points),
        _config(maximum_ground_fill_distance_m=0.25, obstacle_max_height_m=1.0),
    )
    row = 5
    column = 9
    assert result.ground_height_m[row, column] < 0.20
    assert result.obstacle_count[row, column] == 0
    assert result.occupancy[row, column] != OCCUPIED
    assert result.ground_seed_rejected_count > 0


def test_occluding_vegetation_is_obstacle_not_ground_surface():
    points = []
    vegetation_columns = {9, 10}
    for ix in range(20):
        for iy in range(12):
            x = 0.05 + ix * 0.10
            y = 0.05 + iy * 0.10
            if ix not in vegetation_columns:
                for dz in (-0.01, 0.0, 0.01, 0.015):
                    points.append((x, y, dz))
            else:
                for dz in (0.42, 0.45, 0.48, 0.52):
                    points.append((x, y, dz))
    result = derive_ground_relative_navigation_map(
        _cloud(points),
        _config(maximum_ground_fill_distance_m=0.25, obstacle_max_height_m=0.80),
    )
    row = 5
    column = 9
    assert result.ground_height_m[row, column] < 0.20
    assert result.obstacle_count[row, column] >= 2
    assert result.occupancy[row, column] == OCCUPIED


def test_unobserved_padding_stays_unknown():
    result = derive_ground_relative_navigation_map(
        _sloped_ground(), _config(padding_m=0.50, maximum_ground_fill_distance_m=0.10)
    )
    assert result.occupancy[0, 0] == UNKNOWN


def test_polygon_overrides_are_explicit_and_replayable():
    result = derive_ground_relative_navigation_map(_sloped_ground(), _config())
    polygon = [[0.0, 0.0], [0.5, 0.0], [0.5, 0.5], [0.0, 0.5]]
    forced = apply_navigation_overrides(
        result, [{"mode": "force_occupied", "polygon_xy": polygon}]
    )
    assert np.count_nonzero(forced == OCCUPIED) > np.count_nonzero(
        result.occupancy == OCCUPIED
    )
    cleared = apply_navigation_overrides(
        result, [{"mode": "force_free", "polygon_xy": polygon}]
    )
    assert np.count_nonzero(cleared == FREE) >= np.count_nonzero(
        result.occupancy == FREE
    )


def test_export_writes_nav2_pgm_yaml_and_evidence(tmp_path):
    result = derive_ground_relative_navigation_map(_sloped_ground(), _config())
    output = write_navigation_map_derivation(
        result,
        tmp_path / "nav_run",
        source_asset="synthetic.pcd",
    )
    assert (output / "navigation_map.pgm").exists()
    nav_yaml = yaml.safe_load((output / "navigation_map.yaml").read_text())
    assert nav_yaml["mode"] == "trinary"
    assert nav_yaml["resolution"] == pytest.approx(0.10)
    record = yaml.safe_load((output / "derivation.yaml").read_text())
    assert record["schema"] == "agt_ground_relative_navigation_map/v1"
    assert record["source_asset"] == "synthetic.pcd"
    assert set(record["counts"]) == {"free", "occupied", "unknown"}
    assert set(record["ground_seeds"]) == {"candidate", "trusted", "rejected"}
    for name in (
        "ground_height.npy",
        "slope_deg.npy",
        "step_m.npy",
        "obstacle_count.npy",
        "ground_support_count.npy",
    ):
        assert (output / name).exists()
