from pathlib import Path

import numpy as np
import yaml

from agt_greenhouse_annotation import (
    AisleGraphConfig,
    AislePairDiagnostic,
    CorridorRefinementConfig,
    CorridorRefinementResult,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
    derive_agricultural_aisle_graph,
    write_agricultural_aisle_graph,
)


def _fixture():
    shape = (40, 40)
    resolution = 0.10
    rows, cols = np.indices(shape, dtype=np.float64)
    ground = 0.01 * cols
    navigation = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=ground,
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.zeros(shape, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(resolution_m=resolution),
    )
    structure = NavigationStructureResult(
        ground_confidence=np.ones(shape, dtype=np.float64),
        robust_slope_deg=np.zeros(shape, dtype=np.float64),
        robust_plane_residual_m=np.zeros(shape, dtype=np.float64),
        row_support=np.zeros(shape, dtype=np.float64),
        row_regularized_obstacle=np.zeros(shape, dtype=bool),
        aisle_candidate=np.zeros(shape, dtype=bool),
        row_model=RowModel(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64),
            angle_deg=0.0,
            centers_v_m=(1.0, 2.0),
            half_width_m=0.20,
            support_fraction=(1.0, 1.0),
        ),
        config=NavigationStructureConfig(),
    )

    centerline = np.zeros(shape, dtype=bool)
    # map cell centers: y=(row+0.5)*0.1, so rows 14 and 24 are y=1.45/2.45.
    centerline[14, 3:36] = True
    centerline[24, 4:35] = True
    boundary_centerline = np.zeros(shape, dtype=bool)
    boundary_centerline[24, 4:35] = True
    aisle = np.zeros(shape, dtype=bool)
    aisle[12:17, 3:36] = True
    aisle[22:27, 4:35] = True
    boundary_aisle = np.zeros(shape, dtype=bool)
    boundary_aisle[22:27, 4:35] = True

    diagnostics = (
        AislePairDiagnostic(
            pair_index=1,
            left_row_center_v_m=1.0,
            right_row_center_v_m=2.0,
            center_distance_m=1.0,
            structural_reserved_m=0.40,
            side_clearance_reserved_m=0.20,
            geometric_available_width_m=0.40,
            minimum_required_width_m=0.35,
            longitudinal_overlap_m=3.2,
            geometric_cell_count=160,
            safe_cell_count=120,
            centerline_cell_count=33,
            status="ACCEPTED",
            pair_kind="ROW_ROW",
        ),
        AislePairDiagnostic(
            pair_index=2,
            left_row_center_v_m=2.0,
            right_row_center_v_m=3.0,
            center_distance_m=1.0,
            structural_reserved_m=0.30,
            side_clearance_reserved_m=0.20,
            geometric_available_width_m=0.50,
            minimum_required_width_m=0.35,
            longitudinal_overlap_m=3.0,
            geometric_cell_count=150,
            safe_cell_count=110,
            centerline_cell_count=31,
            status="ACCEPTED",
            pair_kind="BOUNDARY_HIGH",
        ),
    )
    corridor = CorridorRefinementResult(
        row_centerline=np.zeros(shape, dtype=bool),
        row_structural_band=np.zeros(shape, dtype=bool),
        vegetation_envelope=np.zeros(shape, dtype=bool),
        boundary_exclusion=np.zeros(shape, dtype=bool),
        aisle_geometric_envelope=aisle.copy(),
        aisle_candidate=aisle,
        aisle_centerline=centerline,
        boundary_aisle_candidate=boundary_aisle,
        boundary_aisle_centerline=boundary_centerline,
        accepted_row_centers_v_m=(1.0, 2.0),
        rejected_row_centers_v_m=(3.0,),
        nominal_row_spacing_m=1.0,
        aisle_pair_diagnostics=diagnostics,
        config=CorridorRefinementConfig(enable_boundary_aisles=True),
    )
    return navigation, structure, corridor


def test_aisle_graph_contains_interior_and_boundary_primitives():
    navigation, structure, corridor = _fixture()
    graph = derive_agricultural_aisle_graph(navigation, structure, corridor)
    assert graph.schema == "agt_agricultural_aisle_graph/v1"
    assert graph.frame_id == "map"
    assert len(graph.aisles) == 2
    assert [aisle.kind for aisle in graph.aisles] == ["interior", "boundary"]
    assert graph.aisles[0].left_structure_ref == "row_01"
    assert graph.aisles[0].right_structure_ref == "row_02"
    assert graph.aisles[1].left_structure_ref == "row_02"
    assert graph.aisles[1].right_structure_ref == "boundary_high"


def test_aisle_centerline_is_ordered_xyz_and_uses_ground_height():
    navigation, structure, corridor = _fixture()
    graph = derive_agricultural_aisle_graph(
        navigation,
        structure,
        corridor,
        AisleGraphConfig(centerline_sample_spacing_m=0.20),
    )
    aisle = graph.aisles[0]
    points = np.asarray(aisle.centerline_xyz)
    assert points.shape[0] >= 10
    assert np.all(np.diff(points[:, 0]) > 0.0)
    assert np.max(points[:, 2]) > np.min(points[:, 2])
    assert aisle.length_m > 2.5
    assert np.isclose(aisle.start_pose[3], 0.0)
    assert np.isclose(aisle.end_pose[3], 0.0)


def test_aisle_graph_export_is_deterministic(tmp_path: Path):
    navigation, structure, corridor = _fixture()
    graph = derive_agricultural_aisle_graph(
        navigation,
        structure,
        corridor,
        source={"map_id": "greenhouse_test"},
    )
    first = write_agricultural_aisle_graph(graph, tmp_path / "first.yaml")
    second = write_agricultural_aisle_graph(graph, tmp_path / "second.yaml")
    assert first.read_bytes() == second.read_bytes()
    payload = yaml.safe_load(first.read_text(encoding="utf-8"))
    assert payload["schema"] == "agt_agricultural_aisle_graph/v1"
    assert payload["aisle_count"] == 2
    assert payload["source"]["map_id"] == "greenhouse_test"
    assert payload["aisles"][1]["kind"] == "boundary"
