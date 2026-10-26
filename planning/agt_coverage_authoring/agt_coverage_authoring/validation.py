"""Conservative offline footprint and Ackermann trajectory validation.

Adapted from AGT navigation v2's ``path_validator.py`` and
``time_simulation.py`` at 7ad0a6bf3ee39ccceb62d7bdc8c9060005f43f49:
https://github.com/Aldoubt/agt_navigation_v2/tree/7ad0a6bf3ee39ccceb62d7bdc8c9060005f43f49/src/agt_coverage_planning/agt_coverage_planning

This implementation retains full-polygon checking and bounded interpolation,
and adds an explicit rear-axle reference, nonholonomic motion checks and gear
events. It validates an offline geometric interpolant, never controller or
physical execution. Missing timing or measured limits produces UNKNOWN.
"""
from __future__ import annotations

from dataclasses import replace
import math
from typing import Iterable

from shapely.geometry import Polygon

from .grid import forbidden_geometry, footprint_at, map_polygon
from .models import PathPoint, PlanningContext

EPS = 1e-8
MAX_POINTS = 250_000
# Chord heading equals midpoint body heading for a constant-curvature arc.
# This small tolerance accommodates sampled varying-curvature paths, not lateral
# motion. It is deliberately independent of map resolution.
HEADING_TOLERANCE = 0.025


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _arc(start: PathPoint, end: PathPoint):
    dx, dy = end.x - start.x, end.y - start.y
    chord = math.hypot(dx, dy)
    angle = _angle(end.yaw - start.yaw)
    factor = abs(angle) / (2.0 * math.sin(abs(angle) / 2.0)) if abs(angle) > EPS else 1.0
    length = chord * factor
    signed_length = length if start.direction == "F" else -length
    curvature = angle / signed_length if length > EPS else 0.0
    return chord, angle, length, signed_length, curvature


def _interpolate(start: PathPoint, end: PathPoint, ratio: float) -> PathPoint:
    """Constant body-curvature arc with a bounded endpoint residual.

    Using a straight chord for a sparse turn would check the wrong side of an
    obstacle. The nonholonomic check bounds the small residual used to reach
    the exact supplied endpoint; collision sweeping includes its translation.
    """
    _, angle, _, signed_length, _ = _arc(start, end)

    def displacement(t):
        half = angle * t / 2.0
        sinc = math.sin(half) / half if abs(half) > EPS else 1.0
        distance = signed_length * t * sinc
        return distance * math.cos(start.yaw + half), distance * math.sin(start.yaw + half)

    ux, uy = displacement(ratio)
    ex, ey = displacement(1.0)
    return replace(start,
                   x=start.x + ux + ratio * (end.x - start.x - ex),
                   y=start.y + uy + ratio * (end.y - start.y - ey),
                   yaw=_angle(start.yaw + ratio * angle))


def validate_path(points: Iterable[PathPoint], context: PlanningContext, *,
                  check_dynamic: bool = True, max_samples: int = 200_000) -> dict:
    """Validate rear-axle points; ``direction`` applies to the outgoing edge.

    A gear change is represented by two coincident, same-heading points with
    old/new directions, zero speed, and a ``gear_change`` event on both points.
    ``geometric_valid`` includes both collision and nonholonomic constraints.
    UNKNOWN dynamic status permits preview only and must never approve READY.
    """
    points = list(points)
    issues: list[dict] = []
    error_codes: set[str] = set()
    collision_free, kinematic_valid = True, True
    dynamic_invalid = False
    sample_count = 0
    minimum_clearance = math.inf
    curvatures = [0.0] * len(points)
    clearances = [0.0] * len(points)

    def issue(code: str, index: int, detail: str, category: str = "kinematic"):
        nonlocal collision_free, kinematic_valid, dynamic_invalid
        error_codes.add(code)
        if len(issues) < 1000:
            issues.append({"code": code, "index": index, "detail": detail})
        if category == "collision":
            collision_free = False
        elif category == "dynamic":
            dynamic_invalid = True
        else:
            kinematic_valid = False

    def report(dynamic_status="UNKNOWN", missing=None):
        geometric_valid = collision_free and kinematic_valid
        clearance = minimum_clearance if math.isfinite(minimum_clearance) else 0.0
        return {
            "valid": geometric_valid and dynamic_status != "INVALID",
            "geometric_valid": geometric_valid,
            "kinematic_valid": kinematic_valid,
            "collision_free": collision_free,
            "dynamic_status": dynamic_status,
            "dynamic_missing": list(missing or []),
            "eligible_for_execution": False,
            "trajectory_reference": "rear_axle_center",
            "issues": issues,
            "error_codes": sorted(error_codes),
            "minimum_clearance": max(0.0, float(clearance)),
            "maximum_curvature": max((abs(value) for value in curvatures), default=0.0),
            "per_point_curvature": curvatures,
            "per_point_clearance": clearances,
            "sample_count": sample_count,
            "sweep_model": "constant_curvature_endpoint_residual_convex_hulls",
        }

    if len(points) < 2 or len(points) > MAX_POINTS:
        issue("INVALID_POINT_COUNT", -1, f"Expected 2..{MAX_POINTS} trajectory points")
        return report("INVALID")
    if type(max_samples) is not int or max_samples < 2:
        issue("INVALID_SAMPLE_LIMIT", -1, "max_samples must be an integer >= 2")
        return report("INVALID")
    robot = context.robot
    numeric = (robot.wheelbase, robot.min_turning_radius, context.grid.resolution,
               context.sample_step, context.clearance_margin, *robot.rear_axle_to_base)
    if (not all(math.isfinite(value) for value in numeric)
            or min(numeric[:4]) <= 0 or context.clearance_margin < 0):
        issue("INVALID_GEOMETRY", -1, "Finite positive model, grid and sample dimensions are required")
        return report("INVALID")
    if len(robot.footprint) < 3 or not all(
            len(vertex) == 2 and all(math.isfinite(value) for value in vertex)
            for vertex in robot.footprint):
        issue("INVALID_FOOTPRINT", -1, "Canonical footprint requires finite XY vertices")
        return report("INVALID")
    shape = Polygon(robot.footprint)
    if shape.is_empty or not shape.is_valid or not math.isfinite(shape.area) or shape.area <= EPS:
        issue("INVALID_FOOTPRINT", -1, "Canonical footprint must be a valid positive-area polygon")
        return report("INVALID")
    if robot.path_reference != "rear_axle_center":
        issue("REFERENCE_MISMATCH", -1, "Trajectory poses must explicitly refer to rear axle center")
        return report("INVALID")
    for index, point in enumerate(points):
        context.check_cancelled()
        if not all(math.isfinite(value) for value in (point.x, point.y, point.yaw, point.v_ref)):
            issue("NONFINITE_TRAJECTORY", index, "Pose and speed must be finite")
        if point.direction not in ("F", "R") or point.v_ref < 0:
            issue("INVALID_DIRECTION_SPEED", index, "Use explicit F/R and nonnegative speed magnitude")
        if point.direction == "R" and not context.allow_reverse:
            issue("REVERSE_DISABLED", index, "This task disallows reverse travel")
    if issues:
        return report("INVALID")

    arcs = []
    radius_limit = 1.0 / robot.min_turning_radius
    for index, (start, end) in enumerate(zip(points, points[1:])):
        context.check_cancelled()
        chord, angle, length, _, curvature = _arc(start, end)
        if not all(math.isfinite(value) for value in (chord, angle, length, curvature)):
            issue("TRAJECTORY_RANGE_EXCEEDED", index, "Trajectory displacement exceeds finite numeric range")
            return report("INVALID")
        arcs.append((length, angle, curvature))
        curvatures[index] = curvature
        if chord <= EPS:
            if abs(angle) > EPS:
                issue("IN_PLACE_ROTATION", index, "Ackermann trajectories cannot rotate without translation")
        else:
            expected = start.yaw + angle / 2.0 + (math.pi if start.direction == "R" else 0.0)
            bearing = math.atan2(end.y - start.y, end.x - start.x)
            error = abs(_angle(bearing - expected))
            if error > HEADING_TOLERANCE:
                issue("NONHOLONOMIC_MOTION", index,
                      f"Displacement disagrees with vehicle heading/gear by {error:.6g} rad")
            if abs(curvature) > radius_limit + 1e-6:
                issue("TURNING_RADIUS_VIOLATION", index,
                      f"Curvature {curvature:.6g} exceeds 1/Rmin={radius_limit:.6g}")
            steering = math.atan(robot.wheelbase * curvature)
            if robot.max_steering_angle is not None and abs(steering) > robot.max_steering_angle + 1e-6:
                issue("STEERING_ANGLE_VIOLATION", index, "Required equivalent steering angle exceeds profile")
        if start.direction != end.direction:
            if (chord > EPS or abs(angle) > EPS or start.v_ref > EPS or end.v_ref > EPS
                    or start.event_ref != "gear_change" or end.event_ref != "gear_change"):
                issue("GEAR_CHANGE_WITHOUT_STOP", index,
                      "Gear change requires coincident stopped old/new gear points and gear_change events")
    curvatures[-1] = curvatures[-2]
    if not any(length > EPS for length, _, _ in arcs):
        issue("ZERO_MOTION_PATH", -1, "A route must contain nonzero translational motion")

    # Bound work before building any interpolation geometry.
    footprint_radius = max(math.hypot(x + robot.rear_axle_to_base[0],
                                      y + robot.rear_axle_to_base[1])
                           for x, y in robot.footprint)
    linear_step = min(context.sample_step, context.grid.resolution / 2.0)
    angular_step = min(0.05, linear_step / max(footprint_radius, EPS))
    if linear_step <= 0 or angular_step <= 0:
        issue("SAMPLE_LIMIT_EXCEEDED", -1, "Interpolation precision exceeds finite numeric range")
        return report("INVALID")
    counts = []
    required_samples = 1
    for length, angle, _ in arcs:
        required = max(1, length / linear_step, abs(angle) / angular_step)
        if not math.isfinite(required) or required > max_samples:
            issue("SAMPLE_LIMIT_EXCEEDED", -1, "Trajectory requires more than bounded validation samples")
            return report("INVALID")
        counts.append(math.ceil(required))
        required_samples += counts[-1]
        if required_samples > max_samples:
            issue("SAMPLE_LIMIT_EXCEEDED", -1, "Trajectory requires more than bounded validation samples")
            return report("INVALID")

    forbidden = forbidden_geometry(context)
    map_boundary = map_polygon(context.grid)
    field_boundary = context.boundary
    margin = context.clearance_margin

    def check_shape(polygon, index, extra_buffer=0.0):
        nonlocal minimum_clearance
        # Only this explicit task margin is added, once. Canonical footprint is
        # already the selected profile's navigation geometry.
        checked = polygon.buffer(margin + extra_buffer) if margin + extra_buffer > EPS else polygon
        blocked = False
        if not map_boundary.covers(checked):
            issue("FOOTPRINT_OUTSIDE_MAP", index, "Full swept footprint leaves observed map extent", "collision")
            blocked = True
        if not field_boundary.covers(checked):
            issue("FOOTPRINT_OUTSIDE_FIELD", index, "Full swept footprint leaves the confirmed field", "collision")
            blocked = True
        if not forbidden.is_empty and checked.intersects(forbidden):
            issue("FOOTPRINT_COLLISION", index, "Swept footprint intersects occupied, unknown or keepout space", "collision")
            blocked = True
        clearance = min(checked.distance(map_boundary.boundary), checked.distance(field_boundary.boundary))
        if not forbidden.is_empty:
            clearance = min(clearance, checked.distance(forbidden))
        clearance = 0.0 if blocked else clearance
        minimum_clearance = min(minimum_clearance, clearance)
        return clearance

    for index, point in enumerate(points):
        context.check_cancelled()
        clearances[index] = check_shape(footprint_at(point, robot), index)
    sample_count = 1
    for index, (start, end) in enumerate(zip(points, points[1:])):
        count = counts[index]
        previous = footprint_at(start, robot)
        _, angle, curvature = arcs[index]
        for step in range(1, count + 1):
            context.check_cancelled()
            point = _interpolate(start, end, step / count)
            current = footprint_at(point, robot)
            hull = previous.union(current).convex_hull
            half_angle = abs(angle) / (2.0 * count)
            # Hulls miss outward portions of rotating corners and the circular
            # reference-point arc. Add both exact sagitta upper bounds.
            rotational_sagitta = footprint_radius * (1.0 - math.cos(half_angle))
            center_sagitta = ((1.0 - math.cos(half_angle)) / abs(curvature)
                               if abs(curvature) > EPS else 0.0)
            check_shape(hull, index, rotational_sagitta + center_sagitta)
            previous = current
            sample_count += 1

    missing = []
    if not check_dynamic:
        return report("UNKNOWN", ["dynamic_check_not_requested"])
    for name in ("geometry_verified", "reference_verified", "limits_verified"):
        if not getattr(robot, name):
            missing.append(name)
    for name in ("max_steering_angle", "max_steering_rate", "max_acceleration", "max_deceleration"):
        value = getattr(robot, name)
        if value is None:
            missing.append(name)
        elif not math.isfinite(value) or value <= 0:
            issue("INVALID_DYNAMIC_LIMIT", -1, f"{name} must be finite and positive", "dynamic")
    for name in ("max_forward_speed", "max_reverse_speed"):
        value = getattr(robot, name)
        if not math.isfinite(value) or value <= 0:
            issue("INVALID_DYNAMIC_LIMIT", -1, f"{name} must be finite and positive", "dynamic")
    for index, point in enumerate(points):
        speed_limit = robot.max_forward_speed if point.direction == "F" else robot.max_reverse_speed
        if point.v_ref > speed_limit + 1e-6:
            issue("SPEED_LIMIT_VIOLATION", index, "Reference speed exceeds selected gear limit", "dynamic")

    times = [point.time_from_start for point in points]
    if any(value is None for value in times):
        missing.append("complete_time_parameterization")
    elif not all(math.isfinite(value) and value >= 0 for value in times):
        issue("INVALID_TIMING", -1, "Trajectory times must be finite nonnegative seconds", "dynamic")
    else:
        moving_steering = []
        for index, (start, end) in enumerate(zip(points, points[1:])):
            duration = times[index + 1] - times[index]
            length, _, curvature = arcs[index]
            if duration < 0 or (length > EPS and duration <= EPS):
                issue("INVALID_TIMING", index, "Moving segments require strictly increasing time", "dynamic")
                continue
            if duration > EPS:
                speed_limit = robot.max_forward_speed if start.direction == "F" else robot.max_reverse_speed
                if length / duration > speed_limit + 1e-6:
                    issue("TIMED_SPEED_LIMIT_VIOLATION", index, "Arc distance/time exceeds gear speed limit", "dynamic")
                acceleration = (end.v_ref - start.v_ref) / duration
                if robot.max_acceleration is not None and acceleration > robot.max_acceleration + 1e-6:
                    issue("ACCELERATION_LIMIT_VIOLATION", index, "Speed increase exceeds acceleration limit", "dynamic")
                if robot.max_deceleration is not None and -acceleration > robot.max_deceleration + 1e-6:
                    issue("DECELERATION_LIMIT_VIOLATION", index, "Speed decrease exceeds deceleration limit", "dynamic")
            if length > EPS:
                moving_steering.append((index, (times[index] + times[index + 1]) / 2.0,
                                        math.atan(robot.wheelbase * curvature)))
        if robot.max_steering_rate is not None:
            for (_, first_t, first), (index, second_t, second) in zip(moving_steering, moving_steering[1:]):
                duration = second_t - first_t
                if duration <= EPS or abs(second - first) > robot.max_steering_rate * duration + 1e-6:
                    issue("STEERING_RATE_VIOLATION", index, "Required steering change exceeds rate limit", "dynamic")
    return report("INVALID" if dynamic_invalid else "UNKNOWN" if missing else "VALID", missing)


def validate_connection(points: Iterable[PathPoint], context: PlanningContext) -> dict:
    """Geometry-only preview check; no dynamic or execution approval."""
    return validate_path(points, context, check_dynamic=False)
