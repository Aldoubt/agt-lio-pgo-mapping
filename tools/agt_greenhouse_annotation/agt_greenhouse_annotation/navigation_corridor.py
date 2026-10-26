"""Refine agricultural row evidence into explicit crop-row and aisle geometry.

This layer keeps five concepts separate:

* row centerline / structural band: nominal crop-row geometry
* vegetation envelope: observed raw obstacle evidence around plants
* aisle geometric envelope: structurally valid corridor before Ground filtering
* interior aisle candidate: safe corridor between adjacent valid crop rows
* boundary aisle candidate: explicit wall/boundary-anchor to nearest crop row

Interior aisles exist only between adjacent valid crop rows. Boundary aisles
require an explicit boundary anchor and are never inferred from leftover free
space near the map edge.  The geometric envelope deliberately survives missing
ground evidence so later V25-12F traversability recovery can reason about short
agricultural occlusion gaps without globally freeing UNKNOWN cells.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .navigation_map_derivation import NavigationMapResult
from .navigation_structure import NavigationStructureResult


@dataclass(frozen=True)
class CorridorRefinementConfig:
    boundary_exclusion_m: float = 0.45
    row_structural_half_width_m: float = 0.20
    aisle_side_clearance_m: float = 0.12
    raw_obstacle_clearance_m: float = 0.12
    aisle_minimum_width_m: float = 0.45
    aisle_centerline_half_width_m: float = 0.06
    spacing_tolerance_ratio: float = 0.38
    # Defensive consolidation of close row hypotheses before spacing and aisle
    # pairing. Zero preserves historical behaviour unless a reviewed site preset
    # explicitly enables same-row peak merging.
    row_hypothesis_merge_distance_m: float = 0.0
    minimum_row_longitudinal_span_m: float = 1.50
    minimum_ground_confidence: float = 0.30

    # Boundary aisles stay opt-in in the offline core. The Workbench enables
    # them explicitly for greenhouse review so existing batch semantics remain
    # conservative unless the caller asks for wall-to-row corridors.
    enable_boundary_aisles: bool = False
    boundary_anchor_max_distance_m: float = 0.80
    boundary_wall_half_width_m: float = 0.10
    boundary_wall_clearance_m: float = 0.12

    def validate(self) -> None:
        if self.boundary_exclusion_m < 0.0:
            raise ValueError("boundary_exclusion_m must be >= 0")
        if self.row_structural_half_width_m <= 0.0:
            raise ValueError("row_structural_half_width_m must be > 0")
        if self.aisle_side_clearance_m < 0.0:
            raise ValueError("aisle_side_clearance_m must be >= 0")
        if self.raw_obstacle_clearance_m < 0.0:
            raise ValueError("raw_obstacle_clearance_m must be >= 0")
        if self.aisle_minimum_width_m <= 0.0:
            raise ValueError("aisle_minimum_width_m must be > 0")
        if self.aisle_centerline_half_width_m <= 0.0:
            raise ValueError("aisle_centerline_half_width_m must be > 0")
        if not 0.0 <= self.spacing_tolerance_ratio < 1.0:
            raise ValueError("spacing_tolerance_ratio must be in [0, 1)")
        if self.row_hypothesis_merge_distance_m < 0.0:
            raise ValueError("row_hypothesis_merge_distance_m must be >= 0")
        if self.minimum_row_longitudinal_span_m <= 0.0:
            raise ValueError("minimum_row_longitudinal_span_m must be > 0")
        if not 0.0 <= self.minimum_ground_confidence <= 1.0:
            raise ValueError("minimum_ground_confidence must be in [0, 1]")
        if self.boundary_anchor_max_distance_m <= 0.0:
            raise ValueError("boundary_anchor_max_distance_m must be > 0")
        if self.boundary_wall_half_width_m <= 0.0:
            raise ValueError("boundary_wall_half_width_m must be > 0")
        if self.boundary_wall_clearance_m < 0.0:
            raise ValueError("boundary_wall_clearance_m must be >= 0")


@dataclass(frozen=True)
class AislePairDiagnostic:
    """Auditable reason why one corridor pair did or did not form an aisle."""

    pair_index: int
    left_row_center_v_m: float
    right_row_center_v_m: float
    center_distance_m: float
    structural_reserved_m: float
    side_clearance_reserved_m: float
    geometric_available_width_m: float
    minimum_required_width_m: float
    longitudinal_overlap_m: float | None
    geometric_cell_count: int
    safe_cell_count: int
    centerline_cell_count: int
    status: str
    pair_kind: str = "ROW_ROW"


@dataclass(frozen=True)
class CorridorRefinementResult:
    row_centerline: np.ndarray
    row_structural_band: np.ndarray
    vegetation_envelope: np.ndarray
    boundary_exclusion: np.ndarray
    aisle_geometric_envelope: np.ndarray
    aisle_candidate: np.ndarray
    aisle_centerline: np.ndarray
    boundary_aisle_candidate: np.ndarray
    boundary_aisle_centerline: np.ndarray
    accepted_row_centers_v_m: tuple[float, ...]
    rejected_row_centers_v_m: tuple[float, ...]
    nominal_row_spacing_m: float | None
    aisle_pair_diagnostics: tuple[AislePairDiagnostic, ...]
    config: CorridorRefinementConfig


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("navigation corridor refinement requires scipy") from exc
    return ndimage


def _grid_xy(result: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    columns = np.arange(result.width, dtype=np.float64)
    rows = np.arange(result.height, dtype=np.float64)
    return np.meshgrid(
        result.origin_x_m + (columns + 0.5) * result.resolution_m,
        result.origin_y_m + (rows + 0.5) * result.resolution_m,
    )


def _normalize_direction(direction_xy: Iterable[float]) -> np.ndarray:
    direction = np.asarray(tuple(direction_xy), dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("row direction must be finite and non-zero")
    return direction / norm


def _merge_close_centers(
    centers: np.ndarray,
    *,
    maximum_distance_m: float,
) -> np.ndarray:
    """Merge same-row center hypotheses without imposing a periodic row model."""
    values = np.sort(np.asarray(centers, dtype=np.float64))
    if values.size < 2 or maximum_distance_m <= 0.0:
        return values
    groups: list[list[float]] = [[float(values[0])]]
    for value in values[1:]:
        current = float(value)
        if current - groups[-1][-1] <= maximum_distance_m:
            groups[-1].append(current)
        else:
            groups.append([current])
    return np.asarray([float(np.mean(group)) for group in groups], dtype=np.float64)


def _boundary_candidate_centers(
    centers: np.ndarray,
    *,
    v_min: float,
    v_max: float,
    boundary_exclusion_m: float,
) -> np.ndarray:
    """Return only row-like peaks that lie inside the geometric boundary band.

    Boundary aisle anchors must come from explicit edge evidence.  In
    particular, members of a merged interior canopy hypothesis are not eligible
    merely because they sit on one side of the merged representative center.
    """
    values = np.sort(np.asarray(centers, dtype=np.float64))
    margin = float(boundary_exclusion_m)
    return values[
        (values <= float(v_min) + margin)
        | (values >= float(v_max) - margin)
    ]


def _filter_row_centers(
    centers: np.ndarray,
    *,
    v_min: float,
    v_max: float,
    boundary_exclusion_m: float,
    spacing_tolerance_ratio: float,
    row_hypothesis_merge_distance_m: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, float | None]:
    if centers.size == 0:
        return centers, centers, None
    centers = np.sort(np.asarray(centers, dtype=np.float64))
    interior = (
        (centers >= v_min + boundary_exclusion_m)
        & (centers <= v_max - boundary_exclusion_m)
    )
    interior_centers = _merge_close_centers(
        centers[interior],
        maximum_distance_m=float(row_hypothesis_merge_distance_m),
    )
    rejected = list(centers[~interior])
    if interior_centers.size < 3:
        return interior_centers, np.asarray(rejected, dtype=np.float64), None

    gaps = np.diff(interior_centers)
    positive = gaps[gaps > 1e-6]
    if positive.size == 0:
        return interior_centers, np.asarray(rejected, dtype=np.float64), None
    nominal = float(np.median(positive))
    low = nominal * (1.0 - spacing_tolerance_ratio)
    high = nominal * (1.0 + spacing_tolerance_ratio)

    keep = np.zeros(interior_centers.size, dtype=bool)
    for index in range(interior_centers.size):
        adjacent = []
        if index > 0:
            adjacent.append(interior_centers[index] - interior_centers[index - 1])
        if index + 1 < interior_centers.size:
            adjacent.append(interior_centers[index + 1] - interior_centers[index])
        for gap in adjacent:
            if low <= gap <= high or 2.0 * low <= gap <= 2.0 * high:
                keep[index] = True
                break
    rejected.extend(interior_centers[~keep].tolist())
    accepted = interior_centers[keep]
    return accepted, np.sort(np.asarray(rejected, dtype=np.float64)), nominal


def _support_u_range(
    support_mask: np.ndarray,
    uu: np.ndarray,
    vv: np.ndarray,
    *,
    center_v_m: float,
    half_width_m: float,
    minimum_span_m: float,
) -> tuple[float, float] | None:
    support = support_mask & (np.abs(vv - float(center_v_m)) <= float(half_width_m))
    if not np.any(support):
        return None
    u_min = float(np.min(uu[support]))
    u_max = float(np.max(uu[support]))
    if u_max - u_min < float(minimum_span_m):
        return None
    return u_min, u_max


def _boundary_anchor(
    centers: np.ndarray,
    *,
    side: str,
    first_row: float,
    last_row: float,
    v_min: float,
    v_max: float,
    maximum_distance_m: float,
) -> float | None:
    values = np.sort(np.asarray(centers, dtype=np.float64))
    if side == "low":
        candidates = values[values < first_row - 1e-6]
        if candidates.size == 0:
            return None
        anchor = float(candidates[np.argmin(np.abs(candidates - v_min))])
        return anchor if abs(anchor - v_min) <= maximum_distance_m else None
    if side == "high":
        candidates = values[values > last_row + 1e-6]
        if candidates.size == 0:
            return None
        anchor = float(candidates[np.argmin(np.abs(candidates - v_max))])
        return anchor if abs(anchor - v_max) <= maximum_distance_m else None
    raise ValueError(f"unknown boundary side: {side}")


def _safest_centerline_within_pair(
    safe: np.ndarray,
    geometry: np.ndarray,
    uu: np.ndarray,
    vv: np.ndarray,
    *,
    corridor_min: float,
    corridor_max: float,
    midpoint: float,
    obstacle_distance: np.ndarray,
    resolution_m: float,
    half_width_m: float,
) -> np.ndarray:
    """Choose a safe cross-section ridge instead of forcing the exact midpoint."""
    output = np.zeros(safe.shape, dtype=bool)
    if not np.any(safe) or not np.any(geometry):
        return output

    u_min = float(np.min(uu[geometry]))
    u_max = float(np.max(uu[geometry]))
    bins = max(1, int(np.ceil((u_max - u_min) / resolution_m)) + 1)
    u_index = np.floor((uu - u_min) / resolution_m).astype(np.int64)
    valid_index = (u_index >= 0) & (u_index < bins)
    previous_v: float | None = None

    for bin_index in range(bins):
        cells = safe & valid_index & (u_index == bin_index)
        if not np.any(cells):
            continue
        candidate_v = vv[cells]
        edge_clearance = np.minimum(
            candidate_v - corridor_min,
            corridor_max - candidate_v,
        )
        raw_clearance = obstacle_distance[cells]
        clearance = np.minimum(edge_clearance, raw_clearance)
        centrality_penalty = 0.02 * np.abs(candidate_v - midpoint)
        if previous_v is not None:
            continuity_penalty = 0.04 * np.abs(candidate_v - previous_v)
        else:
            continuity_penalty = 0.0
        score = clearance - centrality_penalty - continuity_penalty
        selected_v = float(candidate_v[int(np.argmax(score))])
        selected_cells = cells & (np.abs(vv - selected_v) <= half_width_m)
        output |= selected_cells
        previous_v = selected_v

    return output


def derive_corridor_refinement(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    config: CorridorRefinementConfig | None = None,
) -> CorridorRefinementResult:
    cfg = config or CorridorRefinementConfig()
    cfg.validate()
    ndimage = _require_scipy()

    xx, yy = _grid_xy(navigation)
    direction = _normalize_direction(structure.row_model.direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    uu = xx * direction[0] + yy * direction[1]
    vv = xx * perpendicular[0] + yy * perpendicular[1]

    observed = np.asarray(navigation.point_count) > 0
    ground_valid = np.asarray(navigation.ground_valid, dtype=bool)
    if np.any(observed):
        v_min = float(np.min(vv[observed]))
        v_max = float(np.max(vv[observed]))
    else:
        v_min = float(np.min(vv))
        v_max = float(np.max(vv))

    centers = np.asarray(structure.row_model.centers_v_m, dtype=np.float64)
    accepted, rejected, nominal_spacing = _filter_row_centers(
        centers,
        v_min=v_min,
        v_max=v_max,
        boundary_exclusion_m=float(cfg.boundary_exclusion_m),
        spacing_tolerance_ratio=float(cfg.spacing_tolerance_ratio),
        row_hypothesis_merge_distance_m=float(cfg.row_hypothesis_merge_distance_m),
    )

    raw_obstacle = (
        np.asarray(navigation.obstacle_count)
        >= int(navigation.config.minimum_obstacle_points)
    )
    vegetation_envelope = raw_obstacle.copy()

    observed_distance = ndimage.distance_transform_edt(observed) * navigation.resolution_m
    boundary_exclusion = observed & (
        observed_distance <= float(cfg.boundary_exclusion_m)
    )

    row_centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    row_structural_band = np.zeros(navigation.occupancy.shape, dtype=bool)
    row_u_ranges: list[tuple[float, float] | None] = []

    source_rows = np.asarray(structure.row_regularized_obstacle, dtype=bool)
    source_half_width = max(
        float(structure.row_model.half_width_m),
        float(cfg.row_structural_half_width_m),
    )
    for center in accepted:
        support_range = _support_u_range(
            source_rows,
            uu,
            vv,
            center_v_m=float(center),
            half_width_m=source_half_width,
            minimum_span_m=float(cfg.minimum_row_longitudinal_span_m),
        )
        row_u_ranges.append(support_range)
        if support_range is None:
            continue
        u_min, u_max = support_range
        active_u = (uu >= u_min) & (uu <= u_max)
        row_centerline |= active_u & (
            np.abs(vv - center) <= max(navigation.resolution_m * 0.55, 0.03)
        )
        row_structural_band |= active_u & (
            np.abs(vv - center) <= float(cfg.row_structural_half_width_m)
        )

    obstacle_distance = (
        ndimage.distance_transform_edt(~raw_obstacle) * navigation.resolution_m
    )
    clear_of_raw_obstacle = obstacle_distance >= float(cfg.raw_obstacle_clearance_m)
    confident_ground = (
        np.asarray(structure.ground_confidence)
        >= float(cfg.minimum_ground_confidence)
    )
    slope_ok = np.isfinite(structure.robust_slope_deg) & (
        structure.robust_slope_deg <= navigation.config.maximum_slope_deg
    )
    safe_common = (
        ground_valid
        & confident_ground
        & slope_ok
        & clear_of_raw_obstacle
        & ~row_structural_band
    )
    interior_safe_base = safe_common & ~boundary_exclusion
    boundary_safe_base = safe_common

    aisle_geometric_envelope = np.zeros(navigation.occupancy.shape, dtype=bool)
    aisle_candidate = np.zeros(navigation.occupancy.shape, dtype=bool)
    aisle_centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    boundary_aisle_candidate = np.zeros(navigation.occupancy.shape, dtype=bool)
    boundary_aisle_centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    diagnostics: list[AislePairDiagnostic] = []

    def evaluate_corridor(
        *,
        left_v: float,
        right_v: float,
        left_range: tuple[float, float] | None,
        right_range: tuple[float, float] | None,
        structural_reserved_m: float,
        side_reserved_m: float,
        corridor_min: float,
        corridor_max: float,
        safe_base: np.ndarray,
        pair_kind: str,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, AislePairDiagnostic]:
        pair_index = len(diagnostics) + 1
        center_distance = float(right_v - left_v)
        available_width = float(corridor_max - corridor_min)
        base_kwargs = dict(
            pair_index=pair_index,
            left_row_center_v_m=float(left_v),
            right_row_center_v_m=float(right_v),
            center_distance_m=center_distance,
            structural_reserved_m=float(structural_reserved_m),
            side_clearance_reserved_m=float(side_reserved_m),
            geometric_available_width_m=available_width,
            minimum_required_width_m=float(cfg.aisle_minimum_width_m),
            pair_kind=pair_kind,
        )
        empty = np.zeros(navigation.occupancy.shape, dtype=bool)

        if left_range is None or right_range is None:
            return empty, empty, empty, AislePairDiagnostic(
                **base_kwargs,
                longitudinal_overlap_m=None,
                geometric_cell_count=0,
                safe_cell_count=0,
                centerline_cell_count=0,
                status="REJECTED_MISSING_ROW_SUPPORT",
            )

        overlap_min = max(left_range[0], right_range[0])
        overlap_max = min(left_range[1], right_range[1])
        overlap_length = max(0.0, overlap_max - overlap_min)

        if available_width < cfg.aisle_minimum_width_m:
            return empty, empty, empty, AislePairDiagnostic(
                **base_kwargs,
                longitudinal_overlap_m=overlap_length,
                geometric_cell_count=0,
                safe_cell_count=0,
                centerline_cell_count=0,
                status="REJECTED_TOO_NARROW",
            )
        if overlap_length < cfg.minimum_row_longitudinal_span_m:
            return empty, empty, empty, AislePairDiagnostic(
                **base_kwargs,
                longitudinal_overlap_m=overlap_length,
                geometric_cell_count=0,
                safe_cell_count=0,
                centerline_cell_count=0,
                status="REJECTED_NO_LONGITUDINAL_OVERLAP",
            )

        longitudinal = (uu >= overlap_min) & (uu <= overlap_max)
        geometry = longitudinal & (vv >= corridor_min) & (vv <= corridor_max)
        safe = geometry & safe_base
        midpoint = 0.5 * (corridor_min + corridor_max)
        centerline = _safest_centerline_within_pair(
            safe,
            geometry,
            uu,
            vv,
            corridor_min=float(corridor_min),
            corridor_max=float(corridor_max),
            midpoint=float(midpoint),
            obstacle_distance=obstacle_distance,
            resolution_m=navigation.resolution_m,
            half_width_m=float(cfg.aisle_centerline_half_width_m),
        )
        safe_count = int(np.count_nonzero(safe))
        centerline_count = int(np.count_nonzero(centerline))
        if safe_count == 0:
            status = "REJECTED_NO_SAFE_CELLS"
        elif centerline_count == 0:
            status = "ACCEPTED_NO_CENTERLINE"
        else:
            status = "ACCEPTED"
        diagnostic = AislePairDiagnostic(
            **base_kwargs,
            longitudinal_overlap_m=overlap_length,
            geometric_cell_count=int(np.count_nonzero(geometry)),
            safe_cell_count=safe_count,
            centerline_cell_count=centerline_count,
            status=status,
        )
        return geometry, safe, centerline, diagnostic

    for index in range(max(0, accepted.size - 1)):
        left = float(accepted[index])
        right = float(accepted[index + 1])
        corridor_min = left + cfg.row_structural_half_width_m + cfg.aisle_side_clearance_m
        corridor_max = right - cfg.row_structural_half_width_m - cfg.aisle_side_clearance_m
        geometry, safe, centerline, diagnostic = evaluate_corridor(
            left_v=left,
            right_v=right,
            left_range=row_u_ranges[index] if index < len(row_u_ranges) else None,
            right_range=row_u_ranges[index + 1] if index + 1 < len(row_u_ranges) else None,
            structural_reserved_m=2.0 * float(cfg.row_structural_half_width_m),
            side_reserved_m=2.0 * float(cfg.aisle_side_clearance_m),
            corridor_min=float(corridor_min),
            corridor_max=float(corridor_max),
            safe_base=interior_safe_base,
            pair_kind="ROW_ROW",
        )
        diagnostics.append(diagnostic)
        aisle_geometric_envelope |= geometry
        aisle_candidate |= safe
        aisle_centerline |= centerline

    if cfg.enable_boundary_aisles and accepted.size > 0:
        boundary_source = raw_obstacle | source_rows
        boundary_centers = _boundary_candidate_centers(
            centers,
            v_min=v_min,
            v_max=v_max,
            boundary_exclusion_m=float(cfg.boundary_exclusion_m),
        )
        wall_support_half_width = max(
            float(cfg.boundary_wall_half_width_m) + float(cfg.raw_obstacle_clearance_m),
            2.0 * float(navigation.resolution_m),
        )

        low_anchor = _boundary_anchor(
            boundary_centers,
            side="low",
            first_row=float(accepted[0]),
            last_row=float(accepted[-1]),
            v_min=v_min,
            v_max=v_max,
            maximum_distance_m=float(cfg.boundary_anchor_max_distance_m),
        )
        if low_anchor is not None:
            wall_range = _support_u_range(
                boundary_source,
                uu,
                vv,
                center_v_m=low_anchor,
                half_width_m=wall_support_half_width,
                minimum_span_m=float(cfg.minimum_row_longitudinal_span_m),
            )
            corridor_min = (
                low_anchor
                + cfg.boundary_wall_half_width_m
                + cfg.boundary_wall_clearance_m
            )
            corridor_max = (
                float(accepted[0])
                - cfg.row_structural_half_width_m
                - cfg.aisle_side_clearance_m
            )
            geometry, safe, centerline, diagnostic = evaluate_corridor(
                left_v=low_anchor,
                right_v=float(accepted[0]),
                left_range=wall_range,
                right_range=row_u_ranges[0] if row_u_ranges else None,
                structural_reserved_m=(
                    float(cfg.boundary_wall_half_width_m)
                    + float(cfg.row_structural_half_width_m)
                ),
                side_reserved_m=(
                    float(cfg.boundary_wall_clearance_m)
                    + float(cfg.aisle_side_clearance_m)
                ),
                corridor_min=float(corridor_min),
                corridor_max=float(corridor_max),
                safe_base=boundary_safe_base,
                pair_kind="BOUNDARY_LOW",
            )
            diagnostics.append(diagnostic)
            aisle_geometric_envelope |= geometry
            boundary_aisle_candidate |= safe
            boundary_aisle_centerline |= centerline

        high_anchor = _boundary_anchor(
            boundary_centers,
            side="high",
            first_row=float(accepted[0]),
            last_row=float(accepted[-1]),
            v_min=v_min,
            v_max=v_max,
            maximum_distance_m=float(cfg.boundary_anchor_max_distance_m),
        )
        if high_anchor is not None:
            wall_range = _support_u_range(
                boundary_source,
                uu,
                vv,
                center_v_m=high_anchor,
                half_width_m=wall_support_half_width,
                minimum_span_m=float(cfg.minimum_row_longitudinal_span_m),
            )
            corridor_min = (
                float(accepted[-1])
                + cfg.row_structural_half_width_m
                + cfg.aisle_side_clearance_m
            )
            corridor_max = (
                high_anchor
                - cfg.boundary_wall_half_width_m
                - cfg.boundary_wall_clearance_m
            )
            geometry, safe, centerline, diagnostic = evaluate_corridor(
                left_v=float(accepted[-1]),
                right_v=high_anchor,
                left_range=row_u_ranges[-1] if row_u_ranges else None,
                right_range=wall_range,
                structural_reserved_m=(
                    float(cfg.row_structural_half_width_m)
                    + float(cfg.boundary_wall_half_width_m)
                ),
                side_reserved_m=(
                    float(cfg.aisle_side_clearance_m)
                    + float(cfg.boundary_wall_clearance_m)
                ),
                corridor_min=float(corridor_min),
                corridor_max=float(corridor_max),
                safe_base=boundary_safe_base,
                pair_kind="BOUNDARY_HIGH",
            )
            diagnostics.append(diagnostic)
            aisle_geometric_envelope |= geometry
            boundary_aisle_candidate |= safe
            boundary_aisle_centerline |= centerline

    aisle_candidate |= boundary_aisle_candidate
    aisle_centerline |= boundary_aisle_centerline

    return CorridorRefinementResult(
        row_centerline=row_centerline,
        row_structural_band=row_structural_band,
        vegetation_envelope=vegetation_envelope,
        boundary_exclusion=boundary_exclusion,
        aisle_geometric_envelope=aisle_geometric_envelope,
        aisle_candidate=aisle_candidate,
        aisle_centerline=aisle_centerline,
        boundary_aisle_candidate=boundary_aisle_candidate,
        boundary_aisle_centerline=boundary_aisle_centerline,
        accepted_row_centers_v_m=tuple(float(v) for v in accepted),
        rejected_row_centers_v_m=tuple(float(v) for v in rejected),
        nominal_row_spacing_m=nominal_spacing,
        aisle_pair_diagnostics=tuple(diagnostics),
        config=cfg,
    )
