import numpy as np

from agt_greenhouse_annotation import (
    FREE,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
)
from agt_greenhouse_annotation.navigation_row_tracks import (
    LocalRowTrackingConfig,
    derive_local_row_tracks,
)


def _navigation(*, width=140, height=100, resolution=0.10):
    shape = (height, width)
    ground = np.zeros(shape, dtype=np.float64)
    return NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground,
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.full(shape, 8, dtype=np.int32),
        ground_support_count=np.full(shape, 6, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, FREE, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            maximum_slope_deg=10.0,
            minimum_obstacle_points=2,
        ),
    )


def _structure(navigation, evidence):
    shape = navigation.occupancy.shape
    navigation.obstacle_count[:] = np.rint(
        np.clip(np.asarray(evidence, dtype=np.float64), 0.0, 1.0) * 10.0
    ).astype(np.int32)
    return NavigationStructureResult(
        ground_confidence=np.ones(shape, dtype=np.float64),
        robust_slope_deg=np.zeros(shape, dtype=np.float64),
        robust_plane_residual_m=np.zeros(shape, dtype=np.float64),
        row_support=np.zeros(shape, dtype=np.float64),
        row_regularized_obstacle=np.zeros(shape, dtype=bool),
        aisle_candidate=np.zeros(shape, dtype=bool),
        row_model=RowModel(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64),
            angle_deg=0.0,
            centers_v_m=(),
            half_width_m=0.20,
            support_fraction=(),
        ),
        config=NavigationStructureConfig(row_direction_mode="provided"),
    )


def _config(**kwargs):
    values = dict(
        window_length_m=2.0,
        window_stride_m=0.75,
        profile_bin_m=0.10,
        profile_smoothing_m=0.10,
        minimum_peak_spacing_m=0.55,
        minimum_peak_prominence_ratio=0.08,
        minimum_valid_cells_per_bin=3,
        minimum_peak_support=0.15,
        boundary_exclusion_m=0.35,
        association_distance_m=0.40,
        maximum_missed_windows=1,
        minimum_track_observations=3,
        minimum_track_span_m=2.0,
        row_structural_half_width_m=0.20,
        aisle_side_clearance_m=0.12,
        aisle_minimum_width_m=0.45,
        minimum_pair_overlap_m=1.50,
    )
    values.update(kwargs)
    return LocalRowTrackingConfig(**values)


def _paint_row(evidence, navigation, *, y_m, x0_m, x1_m, value=1.0):
    row = int(round(y_m / navigation.resolution_m - 0.5))
    c0 = max(0, int(np.floor(x0_m / navigation.resolution_m)))
    c1 = min(navigation.width, int(np.ceil(x1_m / navigation.resolution_m)))
    evidence[max(0, row - 1) : min(navigation.height, row + 2), c0:c1] = value


def _track_centers(result):
    return np.asarray([track.representative_v_m for track in result.tracks])


def test_local_windows_recover_rows_visible_in_different_longitudinal_regions():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=2.0, x0_m=0.6, x1_m=7.0)
    _paint_row(evidence, navigation, y_m=4.0, x0_m=6.0, x1_m=13.0)
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    centers = _track_centers(result)
    assert centers.size == 2
    assert np.min(np.abs(centers - 2.0)) < 0.20
    assert np.min(np.abs(centers - 4.0)) < 0.20


def test_dense_local_region_does_not_suppress_clean_sparse_region_row():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=2.0, x0_m=0.5, x1_m=6.5, value=0.30)
    for y_m in (4.0, 5.0, 6.0, 7.0):
        _paint_row(evidence, navigation, y_m=y_m, x0_m=7.0, x1_m=13.0, value=1.0)
    result = derive_local_row_tracks(
        navigation,
        _structure(navigation, evidence),
        _config(minimum_peak_support=0.10),
        row_direction_xy=(1.0, 0.0),
    )
    centers = _track_centers(result)
    assert np.min(np.abs(centers - 2.0)) < 0.20
    assert sum(np.min(np.abs(centers - y)) < 0.20 for y in (4.0, 5.0, 6.0, 7.0)) >= 3


def test_unobserved_evidence_cannot_create_local_row_observation_or_track():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=3.0, x0_m=1.0, x1_m=12.0)
    _paint_row(evidence, navigation, y_m=7.0, x0_m=1.0, x1_m=12.0)
    fake_row = int(round(7.0 / navigation.resolution_m - 0.5))
    navigation.point_count[fake_row - 2 : fake_row + 3, :] = 0
    navigation.ground_valid[fake_row - 2 : fake_row + 3, :] = False
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    centers = _track_centers(result)
    assert np.min(np.abs(centers - 3.0)) < 0.20
    assert np.all(np.abs(centers - 7.0) > 0.35)


def test_strong_outer_boundary_is_not_accepted_as_crop_row_track():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=0.15, x0_m=0.0, x1_m=14.0, value=1.0)
    _paint_row(evidence, navigation, y_m=2.0, x0_m=0.5, x1_m=13.0, value=0.8)
    _paint_row(evidence, navigation, y_m=4.0, x0_m=0.5, x1_m=13.0, value=0.8)
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    centers = _track_centers(result)
    assert centers.size == 2
    assert np.all(centers > 0.50)


def test_bounded_one_window_dropout_is_bridged_into_one_track():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=3.0, x0_m=0.5, x1_m=5.0)
    _paint_row(evidence, navigation, y_m=3.0, x0_m=6.0, x1_m=13.0)
    result = derive_local_row_tracks(
        navigation,
        _structure(navigation, evidence),
        _config(maximum_missed_windows=2),
        row_direction_xy=(1.0, 0.0),
    )
    assert len(result.tracks) == 1
    assert abs(result.tracks[0].representative_v_m - 3.0) < 0.20
    assert result.tracks[0].longitudinal_span_m > 9.0


def test_short_isolated_row_observation_fails_track_span_gate():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=3.0, x0_m=2.0, x1_m=2.8)
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    assert len(result.tracks) == 0


def test_track_raster_is_limited_to_active_longitudinal_extent():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=3.0, x0_m=2.0, x1_m=8.0)
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    assert len(result.tracks) == 1
    assert not np.any(result.row_structural_band[:, :5])
    assert not np.any(result.row_structural_band[:, -10:])
    assert np.any(result.row_structural_band[:, 30:70])


def test_three_adjacent_row_tracks_create_two_pair_anchored_aisles():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    for y_m in (2.0, 4.0, 6.0):
        _paint_row(evidence, navigation, y_m=y_m, x0_m=0.5, x1_m=13.0)
    result = derive_local_row_tracks(
        navigation, _structure(navigation, evidence), _config(), row_direction_xy=(1.0, 0.0)
    )
    accepted = [d for d in result.aisle_diagnostics if d.status == "ACCEPTED"]
    assert len(result.tracks) == 3
    assert len(accepted) == 2
    assert {(d.left_row_id, d.right_row_id) for d in accepted} == {(1, 2), (2, 3)}
    assert np.any(result.aisle_candidate)
    assert np.any(result.aisle_centerline)


def test_row_pair_without_enough_longitudinal_overlap_cannot_author_aisle():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=2.0, x0_m=0.5, x1_m=5.0)
    _paint_row(evidence, navigation, y_m=4.0, x0_m=4.7, x1_m=9.0)
    result = derive_local_row_tracks(
        navigation,
        _structure(navigation, evidence),
        _config(minimum_pair_overlap_m=1.5),
        row_direction_xy=(1.0, 0.0),
    )
    assert len(result.tracks) == 2
    assert len(result.aisle_diagnostics) == 1
    assert result.aisle_diagnostics[0].status == "REJECTED_NO_LONGITUDINAL_OVERLAP"
    assert not np.any(result.aisle_candidate)


def test_too_narrow_row_gap_is_rejected_instead_of_becoming_leftover_free_space():
    navigation = _navigation()
    evidence = np.zeros(navigation.occupancy.shape, dtype=np.float64)
    _paint_row(evidence, navigation, y_m=2.0, x0_m=0.5, x1_m=13.0)
    _paint_row(evidence, navigation, y_m=2.7, x0_m=0.5, x1_m=13.0)
    result = derive_local_row_tracks(
        navigation,
        _structure(navigation, evidence),
        _config(
            row_structural_half_width_m=0.20,
            aisle_side_clearance_m=0.10,
            aisle_minimum_width_m=0.45,
            minimum_peak_spacing_m=0.45,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    assert len(result.tracks) == 2
    assert len(result.aisle_diagnostics) == 1
    assert result.aisle_diagnostics[0].status == "REJECTED_TOO_NARROW"
    assert not np.any(result.aisle_candidate)
