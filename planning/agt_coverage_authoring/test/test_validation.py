"""Physical counterexamples and continuous footprint checks for authoring."""
from dataclasses import replace
import math

import numpy as np
import pytest
from shapely.geometry import box

from agt_coverage_authoring.models import GridMap, PathPoint, PlanningContext, Pose, RobotProfile
from agt_coverage_authoring.validation import validate_path


def context(**changes):
    robot = RobotProfile(
        platform_id="mk_mini_fixture", digest="sha256:" + "a" * 64,
        wheelbase=0.6, min_turning_radius=1.5,
        footprint=((-0.4, -0.3), (0.4, -0.3), (0.4, 0.3), (-0.4, 0.3)),
        rear_axle_to_base=(0.3, 0.0),
    )
    data = dict(grid=GridMap(np.zeros((200, 200), dtype=np.int8), 0.1, (-10.0, -10.0, 0.0)),
                robot=robot, boundary=box(-9, -9, 9, 9), rows=[],
                work_area=box(-8, -8, 8, 8), keepouts=[],
                start=Pose(0, 0, 0), end=Pose(1, 0, 0))
    data.update(changes)
    return PlanningContext(**data)


def codes(report):
    return set(report["error_codes"])


def arc_points(radius=2.0, reverse=False, count=11):
    angles = np.linspace(0.0, -0.5 if reverse else 0.5, count)
    return [PathPoint(radius * math.sin(a), radius * (1 - math.cos(a)), a,
                      direction="R" if reverse else "F", v_ref=0.1)
            for a in angles]


def test_free_straight_is_geometric_preview_not_dynamic_approval():
    result = validate_path([PathPoint(0, 0, 0), PathPoint(1, 0, 0)], context())
    assert result["geometric_valid"]
    assert result["kinematic_valid"]
    assert result["dynamic_status"] == "UNKNOWN"
    assert not result["eligible_for_execution"]
    assert "reference_verified" in result["dynamic_missing"]
    assert result["minimum_clearance"] > 0


def test_constant_yaw_lateral_translation_is_rejected():
    result = validate_path([PathPoint(0, 0, 0), PathPoint(0, 1, 0)], context())
    assert not result["geometric_valid"]
    assert "NONHOLONOMIC_MOTION" in codes(result)


@pytest.mark.parametrize("reverse", [False, True])
def test_forward_and_reverse_constant_curvature_arcs(reverse):
    result = validate_path(arc_points(reverse=reverse), context())
    assert result["geometric_valid"], result["issues"]
    assert result["maximum_curvature"] == pytest.approx(0.5)
    # Positive steering curvature remains positive when both yaw rate and
    # signed travel direction reverse.
    assert all(value > 0 for value in result["per_point_curvature"])


def test_reverse_direction_cannot_hide_forward_displacement():
    points = [PathPoint(0, 0, 0, direction="R", v_ref=0.1),
              PathPoint(1, 0, 0, direction="R", v_ref=0.1)]
    result = validate_path(points, context())
    assert "NONHOLONOMIC_MOTION" in codes(result)


def test_task_disallowing_reverse_rejects_reverse_trajectory():
    result = validate_path(arc_points(reverse=True), context(allow_reverse=False))
    assert "REVERSE_DISABLED" in codes(result)


def test_in_place_rotation_and_too_tight_turn_are_rejected():
    turn = validate_path([PathPoint(0, 0, 0), PathPoint(0, 0, 0.2)], context())
    assert "IN_PLACE_ROTATION" in codes(turn)
    tight = validate_path(arc_points(radius=0.5), context())
    assert "TURNING_RADIUS_VIOLATION" in codes(tight)


def test_full_sweep_hits_obstacle_between_free_endpoints():
    ctx = context(keepouts=[box(-0.05, -0.05, 0.05, 0.05)])
    result = validate_path([PathPoint(-2, 0, 0), PathPoint(2, 0, 0)], ctx)
    assert not result["geometric_valid"]
    assert not result["collision_free"]
    assert "FOOTPRINT_COLLISION" in codes(result)
    # Endpoints are clear; the collision is in the swept interior.
    assert all(value > 0 for value in result["per_point_clearance"])
    assert result["minimum_clearance"] == 0


def test_rear_axle_offset_changes_actual_footprint_collision():
    obstacle = box(0.61, -0.1, 0.65, 0.1)
    ctx = context(keepouts=[obstacle])
    points = [PathPoint(0, 0, 0), PathPoint(0.1, 0, 0)]
    actual = validate_path(points, ctx)
    omitted = validate_path(points, replace(ctx, robot=replace(ctx.robot, rear_axle_to_base=(0.0, 0.0))))
    assert not actual["collision_free"]
    assert omitted["collision_free"]


def test_unknown_cell_is_not_free_space():
    ctx = context()
    ctx.grid.data[100, 100] = -1
    result = validate_path([PathPoint(-1, 0, 0), PathPoint(1, 0, 0)], ctx)
    assert "FOOTPRINT_COLLISION" in codes(result)


def test_rotated_map_grid_coordinates_are_respected():
    grid = GridMap(np.zeros((100, 100), dtype=np.int8), 0.1, (0, 0, math.pi / 2))
    ctx = context(grid=grid, boundary=box(-9.5, 0.5, -0.5, 9.5))
    points = [PathPoint(-5, 4, math.pi / 2), PathPoint(-5, 6, math.pi / 2)]
    assert validate_path(points, ctx)["geometric_valid"]
    grid.data[50, 50] = 100
    assert not validate_path(points, ctx)["geometric_valid"]


def test_field_and_map_limits_apply_to_whole_body():
    ctx = context(boundary=box(-1, -1, 1, 1))
    result = validate_path([PathPoint(0, 0, 0), PathPoint(0.5, 0, 0)], ctx)
    assert "FOOTPRINT_OUTSIDE_FIELD" in codes(result)
    outside = validate_path([PathPoint(9, 0, 0), PathPoint(10, 0, 0)], context())
    assert "FOOTPRINT_OUTSIDE_MAP" in codes(outside)


def cusp_points(valid=True):
    stop = "gear_change" if valid else ""
    speed = 0.0 if valid else 0.1
    return [PathPoint(0, 0, 0, v_ref=0.1),
            PathPoint(1, 0, 0, v_ref=speed, event_ref=stop),
            PathPoint(1, 0, 0, direction="R", v_ref=speed, event_ref=stop),
            PathPoint(0, 0, 0, direction="R", v_ref=0.1)]


def test_gear_cusp_requires_explicit_stationary_event():
    assert validate_path(cusp_points(), context())["geometric_valid"]
    invalid = validate_path(cusp_points(False), context())
    assert "GEAR_CHANGE_WITHOUT_STOP" in codes(invalid)


def verified_context(**limits):
    ctx = context()
    values = dict(geometry_verified=True, reference_verified=True, limits_verified=True,
                  max_steering_angle=0.5, max_steering_rate=0.5,
                  max_acceleration=0.3, max_deceleration=0.5)
    values.update(limits)
    return replace(ctx, robot=replace(ctx.robot, **values))


def test_complete_timing_and_measured_limits_enable_dynamic_validity():
    points = [PathPoint(0, 0, 0, time_from_start=0), PathPoint(1, 0, 0, time_from_start=5)]
    assert validate_path(points, verified_context())["dynamic_status"] == "VALID"
    fast = [replace(point, time_from_start=point.time_from_start / 10) for point in points]
    result = validate_path(fast, verified_context())
    assert result["dynamic_status"] == "INVALID"
    assert "TIMED_SPEED_LIMIT_VIOLATION" in codes(result)


def test_steering_rate_violation_is_separate_from_geometric_validity():
    points = [PathPoint(-0.1, 0, 0, v_ref=0.1, time_from_start=0),
              PathPoint(0, 0, 0, v_ref=0.1, time_from_start=1),
              PathPoint(2 * math.sin(0.05), 2 * (1 - math.cos(0.05)), 0.05,
                        v_ref=0.1, time_from_start=2)]
    result = validate_path(points, verified_context(max_steering_rate=0.01))
    assert result["geometric_valid"]
    assert result["dynamic_status"] == "INVALID"
    assert "STEERING_RATE_VIOLATION" in codes(result)


def test_resource_bound_and_cancel_are_enforced():
    result = validate_path([PathPoint(0, 0, 0), PathPoint(1, 0, 0)], context(), max_samples=3)
    assert "SAMPLE_LIMIT_EXCEEDED" in codes(result)
    with pytest.raises(ValueError, match="canceled"):
        validate_path([PathPoint(0, 0, 0), PathPoint(1, 0, 0)], context(cancelled=lambda: True))


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_nonfinite_input_cannot_reach_geometry_engine(bad):
    result = validate_path([PathPoint(0, 0, 0), PathPoint(bad, 0, 0)], context())
    assert "NONFINITE_TRAJECTORY" in codes(result)
