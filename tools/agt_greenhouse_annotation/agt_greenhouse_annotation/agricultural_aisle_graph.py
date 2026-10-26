"""Deterministic agricultural aisle-graph derivation from V25-12C evidence.

The aisle graph is an offline intermediate asset between Map Workbench structure
analysis and the existing Route Asset pipeline.  It does not mutate occupancy,
choose a coverage order, or generate kinematic connectors.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .navigation_corridor import AislePairDiagnostic, CorridorRefinementResult
from .navigation_map_derivation import NavigationMapResult
from .navigation_structure import NavigationStructureResult


AISLE_GRAPH_SCHEMA = "agt_agricultural_aisle_graph/v1"


@dataclass(frozen=True)
class AisleGraphConfig:
    centerline_sample_spacing_m: float = 0.10
    minimum_centerline_points: int = 2
    round_decimals: int = 6

    def validate(self) -> None:
        if self.centerline_sample_spacing_m <= 0.0:
            raise ValueError("centerline_sample_spacing_m must be > 0")
        if self.minimum_centerline_points < 2:
            raise ValueError("minimum_centerline_points must be >= 2")
        if self.round_decimals < 0:
            raise ValueError("round_decimals must be >= 0")


@dataclass(frozen=True)
class AislePrimitive:
    aisle_id: str
    kind: str
    pair_kind: str
    left_structure_ref: str
    right_structure_ref: str
    centerline_xyz: tuple[tuple[float, float, float], ...]
    start_pose: tuple[float, float, float, float]
    end_pose: tuple[float, float, float, float]
    length_m: float
    geometric_width_m: float
    minimum_required_width_m: float
    center_distance_m: float
    longitudinal_overlap_m: float | None
    safe_cell_count: int
    centerline_cell_count: int
    diagnostic_status: str


@dataclass(frozen=True)
class AgriculturalAisleGraph:
    frame_id: str
    row_direction_xy: tuple[float, float]
    nominal_row_spacing_m: float | None
    aisles: tuple[AislePrimitive, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = AISLE_GRAPH_SCHEMA
    status: str = "DRAFT"


def _normalize_direction(direction_xy: np.ndarray) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        raise ValueError("row direction must be non-zero")
    return direction / norm


def _grid_xyz(navigation: NavigationMapResult) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows, cols = np.indices((navigation.height, navigation.width), dtype=np.float64)
    xx = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
    yy = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
    zz = np.asarray(navigation.ground_height_m, dtype=np.float64)
    return xx, yy, zz


def _row_ref(center_v_m: float, accepted: tuple[float, ...]) -> str:
    if not accepted:
        return "row_unknown"
    centers = np.asarray(accepted, dtype=np.float64)
    index = int(np.argmin(np.abs(centers - float(center_v_m))))
    return f"row_{index + 1:02d}"


def _structure_refs(
    diagnostic: AislePairDiagnostic,
    accepted: tuple[float, ...],
) -> tuple[str, str, str]:
    if diagnostic.pair_kind == "BOUNDARY_LOW":
        return "boundary", "boundary_low", _row_ref(diagnostic.right_row_center_v_m, accepted)
    if diagnostic.pair_kind == "BOUNDARY_HIGH":
        return "boundary", _row_ref(diagnostic.left_row_center_v_m, accepted), "boundary_high"
    return (
        "interior",
        _row_ref(diagnostic.left_row_center_v_m, accepted),
        _row_ref(diagnostic.right_row_center_v_m, accepted),
    )


def _collapse_centerline(
    mask: np.ndarray,
    xx: np.ndarray,
    yy: np.ndarray,
    zz: np.ndarray,
    uu: np.ndarray,
    *,
    spacing_m: float,
) -> tuple[tuple[float, float, float], ...]:
    valid = np.asarray(mask, dtype=bool) & np.isfinite(zz)
    if not np.any(valid):
        return ()

    u_values = uu[valid]
    x_values = xx[valid]
    y_values = yy[valid]
    z_values = zz[valid]
    u_min = float(np.min(u_values))
    bins = np.floor((u_values - u_min) / float(spacing_m) + 1.0e-9).astype(np.int64)

    points: list[tuple[float, float, float]] = []
    for bin_index in np.unique(bins):
        selected = bins == bin_index
        if not np.any(selected):
            continue
        points.append(
            (
                float(np.mean(x_values[selected])),
                float(np.mean(y_values[selected])),
                float(np.mean(z_values[selected])),
            )
        )
    return tuple(points)


def _polyline_length(points: tuple[tuple[float, float, float], ...]) -> float:
    if len(points) < 2:
        return 0.0
    arr = np.asarray(points, dtype=np.float64)
    return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))


def _diagnostic_mask(
    diagnostic: AislePairDiagnostic,
    centerline: np.ndarray,
    vv: np.ndarray,
) -> np.ndarray:
    low = min(diagnostic.left_row_center_v_m, diagnostic.right_row_center_v_m)
    high = max(diagnostic.left_row_center_v_m, diagnostic.right_row_center_v_m)
    # The global centerline contains mutually separated corridor ridges.  The
    # structure-center interval is a deterministic ownership partition for one
    # row-row or boundary-row pair and still permits local lateral avoidance.
    return np.asarray(centerline, dtype=bool) & (vv >= low) & (vv <= high)


def derive_agricultural_aisle_graph(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    corridor: CorridorRefinementResult,
    config: AisleGraphConfig | None = None,
    *,
    frame_id: str = "map",
    source: Mapping[str, Any] | None = None,
) -> AgriculturalAisleGraph:
    """Convert accepted corridor evidence into ordered map-frame aisle primitives."""
    cfg = config or AisleGraphConfig()
    cfg.validate()
    if navigation.occupancy.shape != corridor.aisle_centerline.shape:
        raise ValueError("navigation and corridor grid shapes must match")

    direction = _normalize_direction(structure.row_model.direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    xx, yy, zz = _grid_xyz(navigation)
    uu = xx * direction[0] + yy * direction[1]
    vv = xx * perpendicular[0] + yy * perpendicular[1]
    yaw = math.atan2(float(direction[1]), float(direction[0]))
    accepted = tuple(float(v) for v in corridor.accepted_row_centers_v_m)

    aisles: list[AislePrimitive] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if diagnostic.status not in {"ACCEPTED", "ACCEPTED_NO_CENTERLINE"}:
            continue
        owned = _diagnostic_mask(diagnostic, corridor.aisle_centerline, vv)
        points = _collapse_centerline(
            owned,
            xx,
            yy,
            zz,
            uu,
            spacing_m=cfg.centerline_sample_spacing_m,
        )
        if len(points) < cfg.minimum_centerline_points:
            continue
        kind, left_ref, right_ref = _structure_refs(diagnostic, accepted)
        start = points[0]
        end = points[-1]
        aisles.append(
            AislePrimitive(
                aisle_id=f"aisle_{diagnostic.pair_index:03d}",
                kind=kind,
                pair_kind=str(diagnostic.pair_kind),
                left_structure_ref=left_ref,
                right_structure_ref=right_ref,
                centerline_xyz=points,
                start_pose=(start[0], start[1], start[2], yaw),
                end_pose=(end[0], end[1], end[2], yaw),
                length_m=_polyline_length(points),
                geometric_width_m=float(diagnostic.geometric_available_width_m),
                minimum_required_width_m=float(diagnostic.minimum_required_width_m),
                center_distance_m=float(diagnostic.center_distance_m),
                longitudinal_overlap_m=(
                    None
                    if diagnostic.longitudinal_overlap_m is None
                    else float(diagnostic.longitudinal_overlap_m)
                ),
                safe_cell_count=int(diagnostic.safe_cell_count),
                centerline_cell_count=int(diagnostic.centerline_cell_count),
                diagnostic_status=str(diagnostic.status),
            )
        )

    return AgriculturalAisleGraph(
        frame_id=str(frame_id),
        row_direction_xy=(float(direction[0]), float(direction[1])),
        nominal_row_spacing_m=(
            None
            if corridor.nominal_row_spacing_m is None
            else float(corridor.nominal_row_spacing_m)
        ),
        aisles=tuple(aisles),
        source=dict(source or {}),
    )


def aisle_graph_to_dict(
    graph: AgriculturalAisleGraph,
    *,
    round_decimals: int = 6,
) -> dict[str, Any]:
    def r(value: float) -> float:
        return round(float(value), int(round_decimals))

    payload: dict[str, Any] = {
        "schema": graph.schema,
        "status": graph.status,
        "frame_id": graph.frame_id,
        "source": dict(graph.source),
        "row_direction_xy": [r(graph.row_direction_xy[0]), r(graph.row_direction_xy[1])],
        "nominal_row_spacing_m": (
            None if graph.nominal_row_spacing_m is None else r(graph.nominal_row_spacing_m)
        ),
        "aisle_count": len(graph.aisles),
        "aisles": [],
    }
    for aisle in graph.aisles:
        payload["aisles"].append(
            {
                "aisle_id": aisle.aisle_id,
                "kind": aisle.kind,
                "pair_kind": aisle.pair_kind,
                "adjacent_structure": {
                    "left": aisle.left_structure_ref,
                    "right": aisle.right_structure_ref,
                },
                "centerline_xyz": [[r(x), r(y), r(z)] for x, y, z in aisle.centerline_xyz],
                "start_pose": {
                    "x": r(aisle.start_pose[0]),
                    "y": r(aisle.start_pose[1]),
                    "z": r(aisle.start_pose[2]),
                    "yaw": r(aisle.start_pose[3]),
                },
                "end_pose": {
                    "x": r(aisle.end_pose[0]),
                    "y": r(aisle.end_pose[1]),
                    "z": r(aisle.end_pose[2]),
                    "yaw": r(aisle.end_pose[3]),
                },
                "length_m": r(aisle.length_m),
                "geometric_width_m": r(aisle.geometric_width_m),
                "minimum_required_width_m": r(aisle.minimum_required_width_m),
                "center_distance_m": r(aisle.center_distance_m),
                "longitudinal_overlap_m": (
                    None if aisle.longitudinal_overlap_m is None else r(aisle.longitudinal_overlap_m)
                ),
                "evidence": {
                    "safe_cell_count": aisle.safe_cell_count,
                    "centerline_cell_count": aisle.centerline_cell_count,
                    "diagnostic_status": aisle.diagnostic_status,
                },
            }
        )
    return payload


def write_agricultural_aisle_graph(
    graph: AgriculturalAisleGraph,
    path: str | Path,
    *,
    round_decimals: int = 6,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = aisle_graph_to_dict(graph, round_decimals=round_decimals)
    output.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
