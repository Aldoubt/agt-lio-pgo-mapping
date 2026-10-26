import numpy as np

from agt_greenhouse_annotation import (
    UNKNOWN,
    CorridorRefinementConfig,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
    derive_corridor_refinement,
)
from agt_greenhouse_annotation.aisle_centerlines import derive_geometric_aisle_centerlines


def _fixture():
    resolution = 0.10
    height, width = 42, 60
    shape = (height, width)
    point_count = np.full(shape, 8, dtype=np.int32)
    ground_support = np.full(shape, 6, dtype=np.int32)
    obstacle_count = np.zeros(shape, dtype=np.int32)
    row_centers = (1.0, 2.0, 3.0)
    row_regularized = np.zeros(shape, dtype=bool)
    for center in row_centers:
        row = int(center / resolution)
        row_regularized[row - 1 : row + 2, 5:55] = True
        obstacle_count[row - 1 : row + 2, 5:55] = 5

    navigation = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=point_count,
        ground_support_count=ground_support,
        obstacle_count=obstacle_count,
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, UNKNOWN, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            minimum_obstacle_points=2,
            maximum_slope_deg=15.0,
        ),
    )
    structure = NavigationStructureResult(
        ground_confidence=np.ones(shape, dtype=np.float64),
        robust_slope_deg=np.zeros(shape, dtype=np.float64),
        robust_plane_residual_m=np.zeros(shape, dtype=np.float64),
        row_support=row_regularized.astype(np.float64),
        row_regularized_obstacle=row_regularized,
        aisle_candidate=~row_regularized,
        row_model=RowModel(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64),
            angle_deg=0.0,
            centers_v_m=row_centers,
            half_width_m=0.20,
            support_fraction=(0.9, 0.9, 0.9),
        ),
        config=NavigationStructureConfig(row_half_width_m=0.20),
    )
    config = CorridorRefinementConfig(
        boundary_exclusion_m=0.10,
        row_structural_half_width_m=0.15,
        aisle_side_clearance_m=0.05,
        raw_obstacle_clearance_m=0.05,
        aisle_minimum_width_m=0.35,
        minimum_row_longitudinal_span_m=1.50,
    )
    return navigation, structure, config


def test_geometric_centerline_survives_ground_hole_while_safe_centerline_breaks():
    navigation, structure, config = _fixture()
    navigation.ground_valid[14:17, 20:24] = False

    corridor = derive_corridor_refinement(navigation, structure, config)
    geometric = derive_geometric_aisle_centerlines(navigation, structure, corridor)

    assert np.any(corridor.aisle_geometric_envelope[14:17, 20:24])
    assert np.any(geometric.mask[14:17, 20:24])
    assert not np.any(corridor.aisle_centerline[14:17, 20:24])
    assert np.all(~geometric.mask | corridor.aisle_geometric_envelope)
    assert np.all(~corridor.aisle_centerline | corridor.aisle_candidate)


def test_each_geometric_aisle_reports_centerline_independent_of_safe_cells():
    navigation, structure, config = _fixture()
    navigation.ground_valid[14:17, 20:24] = False

    corridor = derive_corridor_refinement(navigation, structure, config)
    geometric = derive_geometric_aisle_centerlines(navigation, structure, corridor)

    interior = [pair for pair in geometric.pairs if pair.pair_kind == "ROW_ROW"]
    assert len(interior) == 2
    assert all(pair.geometric_cell_count > 0 for pair in interior)
    assert all(pair.centerline_cell_count > 0 for pair in interior)
    assert geometric.expected_interior_aisles == 2
    assert geometric.interior_geometric_aisle_count == 2
    assert geometric.geometric_centerline_count >= 2
    assert np.count_nonzero(geometric.mask) > 0
