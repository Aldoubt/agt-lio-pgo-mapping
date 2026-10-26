"""Coverage accounting adapted from AGT v2 variant_comparison.py.

Source: Aldoubt/agt_navigation_v2 @ 7ad0a6bf, Apache-2.0 project code.
This extension preserves the requested denominator and accepts polyline work
segments. Connection segments never contribute to work coverage.
"""
from __future__ import annotations

import math
from typing import Iterable

from shapely.geometry import LineString
from shapely.ops import unary_union

from .models import CoverageError, PathPoint


def coverage_area_metrics(target, swaths: Iterable, operation_width: float) -> dict:
    width = float(operation_width)
    if not math.isfinite(width) or width <= 0:
        raise CoverageError("INVALID_OPERATION_WIDTH", "Operation width must be positive")
    if target is None or target.is_empty or not target.is_valid or target.area <= 0:
        raise CoverageError("INVALID_WORK_AREA", "Requested work area must be a valid polygon")
    strips = []
    for coordinates in swaths:
        line = LineString(coordinates)
        if line.length <= 1e-9:
            raise CoverageError("ZERO_LENGTH_SWATH", "Cannot account for a zero-length work segment")
        strips.append(line.buffer(width / 2, cap_style=2, join_style=2).intersection(target))
    covered = unary_union(strips)
    covered_area = float(covered.area)
    target_area = float(target.area)
    overlap = max(0., sum(float(strip.area) for strip in strips) - covered_area)
    return {
        "target_area_m2": target_area,
        "covered_area_m2": covered_area,
        "missed_area_m2": max(0., target_area - covered_area),
        "overlap_area_m2": overlap,
        "coverage_rate": min(1., covered_area / target_area),
        "overlap_rate": overlap / target_area,
    }


def work_polylines(points: list[PathPoint]) -> list[list[tuple[float, float]]]:
    """Retain segment boundaries, including repeated visits to the same row."""
    output = []
    current = []
    key = None
    for point in points:
        point_key = (point.segment_id, point.semantic_ref) if point.kind == "SWATH" else None
        if point_key != key:
            if len(current) >= 2:
                output.append(current)
            current = []
            key = point_key
        if point_key is not None:
            xy = (point.x, point.y)
            if not current or xy != current[-1]:
                current.append(xy)
    if len(current) >= 2:
        output.append(current)
    return output


def trajectory_metrics(points: list[PathPoint], robot) -> dict:
    total = work = reverse = seconds = 0.
    reversals = 0
    for first, second in zip(points, points[1:]):
        length = math.hypot(second.x - first.x, second.y - first.y)
        total += length
        if first.kind == "SWATH" and second.kind == "SWATH" and first.segment_id == second.segment_id:
            work += length
        if first.direction == "R":
            reverse += length
        if first.direction != second.direction:
            reversals += 1
        speed = robot.max_reverse_speed if first.direction == "R" else robot.max_forward_speed
        seconds += length / speed
    return {
        "total_length_m": total, "work_length_m": work,
        "connection_length_m": max(0., total - work),
        "reverse_length_m": reverse, "gear_changes": reversals,
        "cruise_time_lower_bound_s": seconds,
        "time_estimate_kind": "cruise_lower_bound_excludes_acceleration_and_gear_dwell",
    }
