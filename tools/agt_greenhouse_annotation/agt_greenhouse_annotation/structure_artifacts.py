"""Versioned greenhouse structure evidence and proposal artifacts.

Automatic row and aisle proposals remain draft evidence. Physical IDs, travel
orientation, headlands, and frozen topology are assigned by explicit review.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .spatial_regions import points_in_polygon, validate_polygon

from .aisle_centerlines import derive_geometric_aisle_centerlines
from .navigation_corridor import CorridorRefinementConfig, derive_corridor_refinement
from .navigation_map_derivation import (
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    derive_ground_relative_navigation_map,
)
from .navigation_row_tracks import (
    LocalRowObservation,
    LocalRowTrackResult,
    LocalRowTrackingConfig,
    RowTrack,
    derive_local_row_tracks,
)
from .navigation_structure import (
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
    _auto_row_direction,
    _ground_confidence,
    _hybrid_row_evidence,
    _require_scipy as _require_structure_scipy,
    _robust_local_plane_slope,
    _terrain_ridge_evidence,
    derive_navigation_structure,
)
from .terrain_morphology import (
    TerrainMorphologyConfig,
    TerrainMorphologyResult,
    derive_terrain_morphology,
)

ARTIFACT_SCHEMA = "agt_greenhouse_structure_proposal/v1"
ANALYSIS_MODES = ("GLOBAL_PROFILE", "LOCAL_TRACKS", "COMPARE")
LEGACY_SOURCE_BRANCH = "refactor/v25-unified-map-authoring"
LEGACY_SOURCE_COMMIT = "42fff086e41109c74966b79d84cb46c6bbfb37dc"


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if hasattr(value, "__dataclass_fields__"):
        return {key: _plain(item) for key, item in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _canonical_hash(value: Any) -> str:
    data = json.dumps(_plain(value), sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def _code_version() -> str:
    files = ("navigation_map_derivation.py", "navigation_structure.py", "terrain_morphology.py",
             "navigation_row_tracks.py", "navigation_corridor.py", "aisle_centerlines.py",
             "agricultural_aisle_graph.py", "structure_artifacts.py")
    root = Path(__file__).resolve().parent
    return _canonical_hash({name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files})


def canonical_row_direction(direction_xy: Iterable[float]) -> np.ndarray:
    direction = np.asarray(tuple(direction_xy), dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(direction).all() or norm <= 1e-12:
        raise ValueError("row direction must be finite and non-zero")
    direction /= norm
    if direction[0] < -1e-12 or (abs(direction[0]) <= 1e-12 and direction[1] < 0.0):
        direction *= -1.0
    return direction


def _xy(direction: np.ndarray, u: float, v: float) -> list[float]:
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    point = direction * float(u) + perpendicular * float(v)
    return [float(point[0]), float(point[1])]


def _weighted_quantile(values: np.ndarray, weights: np.ndarray, quantile: float) -> float:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    weights = np.asarray(weights, dtype=np.float64).reshape(-1)
    if values.size == 0 or values.size != weights.size:
        raise ValueError("weighted quantile needs equally sized non-empty inputs")
    order = np.argsort(values, kind="mergesort")
    sorted_weights = np.maximum(weights[order], 0.0)
    if float(np.sum(sorted_weights)) <= 0.0:
        return float(np.quantile(values, quantile))
    cumulative = np.cumsum(sorted_weights)
    index = min(int(np.searchsorted(cumulative, float(quantile) * cumulative[-1], side="left")),
                len(order) - 1)
    return float(values[order[index]])


@dataclass(frozen=True)
class GreenhouseStructureConfig:
    navigation: GroundRelativeNavigationConfig = field(default_factory=GroundRelativeNavigationConfig)
    global_profile: NavigationStructureConfig = field(default_factory=NavigationStructureConfig)
    local_tracks: LocalRowTrackingConfig = field(default_factory=LocalRowTrackingConfig)
    corridor: CorridorRefinementConfig = field(default_factory=CorridorRefinementConfig)
    terrain: TerrainMorphologyConfig = field(default_factory=TerrainMorphologyConfig)
    row_exclusion_polygons_xy: tuple = ()

    def validate(self) -> None:
        for config in (self.navigation, self.global_profile, self.local_tracks,
                       self.corridor, self.terrain):
            config.validate()
        for polygon in self.row_exclusion_polygons_xy:
            validate_polygon(polygon)

    def to_dict(self) -> dict[str, Any]:
        return _plain(asdict(self))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "GreenhouseStructureConfig":
        return cls(
            navigation=GroundRelativeNavigationConfig(**dict(value.get("navigation", {}))),
            global_profile=NavigationStructureConfig(**dict(value.get("global_profile", {}))),
            local_tracks=LocalRowTrackingConfig(**dict(value.get("local_tracks", {}))),
            corridor=CorridorRefinementConfig(**dict(value.get("corridor", {}))),
            terrain=TerrainMorphologyConfig(**dict(value.get("terrain", {}))),
            row_exclusion_polygons_xy=tuple(tuple(tuple(point) for point in polygon)
                for polygon in value.get("row_exclusion_polygons_xy", ())),
        )


@dataclass(frozen=True)
class GreenhouseTerrainEvidence:
    ground_height_m: np.ndarray
    ground_valid: np.ndarray
    point_support_count: np.ndarray
    ground_support_count: np.ndarray
    obstacle_count: np.ndarray
    ground_confidence: np.ndarray
    robust_slope_deg: np.ndarray
    robust_plane_residual_m: np.ndarray
    terrain_background_height_m: np.ndarray
    signed_relief_m: np.ndarray
    ridge_evidence: np.ndarray
    depression_evidence: np.ndarray
    step_evidence: np.ndarray


@dataclass(frozen=True)
class GlobalRowProposal:
    auto_id: str
    direction_xy: tuple[float, float]
    lateral_v_m: float
    centerline_xy: tuple[tuple[float, float], ...]
    half_width_m: float
    profile_support: float
    longitudinal_support_fraction: float
    diagnostics: Mapping[str, Any]
    decision: str = "pending"
    physical_row_id: str | None = None
    confirmed_direction: str | None = None
    source_mode: str = "GLOBAL_PROFILE"


@dataclass(frozen=True)
class AisleProposal:
    auto_id: str
    left_row_auto_id: str
    right_row_auto_id: str
    aisle_kind: str
    centerline_xy: tuple[tuple[float, float], ...]
    safe_centerline_xy: tuple[tuple[float, float], ...]
    geometric_width_m: float
    minimum_width_m: float
    diagnostics: Mapping[str, Any]
    decision: str = "pending"
    physical_aisle_id: str | None = None
    boundary_anchor_xy: tuple[float, float] | None = None


@dataclass(frozen=True)
class GreenhouseStructureAnalysis:
    mode: str
    input_map_hash: str
    config: GreenhouseStructureConfig
    navigation: NavigationMapResult
    terrain: GreenhouseTerrainEvidence
    global_result: NavigationStructureResult | None
    local_result: LocalRowTrackResult | None
    geometric_aisle_result: Any | None
    corridor_result: Any | None
    global_rows: tuple[GlobalRowProposal, ...]
    local_rows: tuple[GlobalRowProposal, ...]
    aisle_proposals: tuple[AisleProposal, ...]
    direction_source: str
    diagnostics: Mapping[str, Any]
    code_version: str
    analysis_hash: str


def _polyline_from_mask(mask: np.ndarray, navigation: NavigationMapResult,
                        direction: np.ndarray, *, bin_m: float = 0.25) -> tuple[tuple[float, float], ...]:
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return ()
    x = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
    y = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
    perpendicular = np.array([-direction[1], direction[0]])
    u = x * direction[0] + y * direction[1]
    v = x * perpendicular[0] + y * perpendicular[1]
    origin = float(np.min(u))
    bucket = np.floor((u - origin) / max(bin_m, navigation.resolution_m)).astype(np.int64)
    points = []
    for index in np.unique(bucket):
        selected = bucket == index
        if np.count_nonzero(selected) < 1:
            continue
        points.append(tuple(_xy(direction, float(np.median(u[selected])), float(np.median(v[selected])))))
    return tuple(points)


def _mask_centerline_between(mask: np.ndarray, navigation: NavigationMapResult,
                            direction: np.ndarray, v_min: float, v_max: float,
                            *, bin_m: float = 0.25) -> tuple[tuple[float, float], ...]:
    rows, cols = np.nonzero(np.asarray(mask, dtype=bool))
    if not len(rows):
        return ()
    x = navigation.origin_x_m + (cols + .5) * navigation.resolution_m
    y = navigation.origin_y_m + (rows + .5) * navigation.resolution_m
    perpendicular = np.array([-direction[1], direction[0]])
    u = x * direction[0] + y * direction[1]
    v = x * perpendicular[0] + y * perpendicular[1]
    selected = (v >= min(v_min, v_max)) & (v <= max(v_min, v_max))
    if not np.any(selected):
        return ()
    u, v = u[selected], v[selected]
    start = float(np.min(u))
    bins = np.floor((u - start) / max(bin_m, navigation.resolution_m)).astype(np.int64)
    points = []
    for index in np.unique(bins):
        active = bins == index
        points.append(tuple(_xy(direction, float(np.median(u[active])), float(np.median(v[active])))))
    return tuple(points)


def _attach_aisle_centerlines(aisles: tuple[AisleProposal, ...], rows: tuple[GlobalRowProposal, ...],
                              navigation: NavigationMapResult, direction: np.ndarray,
                              geometric_mask: np.ndarray | None, safe_mask: np.ndarray | None,
                              side_clearance_m: float = 0.0,
                              pair_diagnostics: Iterable[Any] = ()) -> tuple[AisleProposal, ...]:
    by_id = {row.auto_id: row for row in rows}
    pair_diagnostics = tuple(pair_diagnostics)
    output = []
    for aisle in aisles:
        left, right = by_id.get(aisle.left_row_auto_id), by_id.get(aisle.right_row_auto_id)
        if left is None or right is None:
            output.append(aisle)
            continue
        low = left.lateral_v_m + left.half_width_m + float(side_clearance_m)
        high = right.lateral_v_m - right.half_width_m - float(side_clearance_m)
        diagnostics = dict(aisle.diagnostics)
        can_have_centerline = diagnostics.get("status") == "CANDIDATE"
        geometric = (_mask_centerline_between(geometric_mask, navigation, direction, low, high)
                     if can_have_centerline and geometric_mask is not None else ())
        safe = (_mask_centerline_between(safe_mask, navigation, direction, low, high)
                if can_have_centerline and safe_mask is not None else ())
        if pair_diagnostics:
            match = min(pair_diagnostics, key=lambda item:
                        abs(float(item.left_row_center_v_m) - left.lateral_v_m)
                        + abs(float(item.right_row_center_v_m) - right.lateral_v_m))
            mismatch = (abs(float(match.left_row_center_v_m) - left.lateral_v_m)
                        + abs(float(match.right_row_center_v_m) - right.lateral_v_m))
            if mismatch <= 1e-3:
                diagnostics["corridor_check"] = _plain(match)
                diagnostics["safe_status"] = str(match.status)
        if geometric and len(geometric) >= 2:
            updated_line = geometric
        else:
            updated_line = aisle.centerline_xy
        output.append(replace(aisle, centerline_xy=updated_line,
                              safe_centerline_xy=safe, diagnostics=diagnostics))
    return tuple(output)


def _build_global_rows(result: NavigationStructureResult,
                       navigation: NavigationMapResult) -> tuple[GlobalRowProposal, ...]:
    direction = canonical_row_direction(result.row_model.direction_xy)
    perpendicular = np.array([-direction[1], direction[0]])
    rows, cols = np.nonzero(np.asarray(navigation.point_count) > 0)
    if not len(rows):
        return ()
    x = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
    y = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
    u = x * direction[0] + y * direction[1]
    v_cells = x * perpendicular[0] + y * perpendicular[1]
    point_weights = np.asarray(navigation.point_count, dtype=np.float64)[rows, cols]
    regularized = np.asarray(result.row_regularized_obstacle, dtype=bool)[rows, cols]
    centers = sorted(zip(result.row_model.centers_v_m, result.row_model.support_fraction))
    profiles = np.asarray(result.row_support, dtype=np.float64)
    proposals = []
    for ordinal, (v, support_fraction) in enumerate(centers, start=1):
        band_cells = np.abs(v_cells - float(v)) <= result.row_model.half_width_m
        supported_cells = band_cells & regularized
        if np.count_nonzero(supported_cells) < 2:
            supported_cells = band_cells
        if np.count_nonzero(supported_cells) < 2:
            continue
        u_min = _weighted_quantile(u[supported_cells], point_weights[supported_cells], 0.02)
        u_max = _weighted_quantile(u[supported_cells], point_weights[supported_cells], 0.98)
        if u_max - u_min < navigation.resolution_m:
            continue
        center_xy = (_xy(direction, u_min, float(v)), _xy(direction, u_max, float(v)))
        band = np.abs((navigation.origin_x_m + (np.arange(navigation.width)[None, :] + .5) * navigation.resolution_m) * perpendicular[0]
                      + (navigation.origin_y_m + (np.arange(navigation.height)[:, None] + .5) * navigation.resolution_m) * perpendicular[1]
                      - float(v)) <= result.row_model.half_width_m
        profile_support = float(np.mean(profiles[band])) if np.any(band) else 0.0
        proposals.append(GlobalRowProposal(
            auto_id=f"AUTO-G{ordinal:03d}", direction_xy=(float(direction[0]), float(direction[1])),
            lateral_v_m=float(v), centerline_xy=(tuple(center_xy[0]), tuple(center_xy[1])),
            half_width_m=float(result.row_model.half_width_m), profile_support=profile_support,
            longitudinal_support_fraction=float(support_fraction),
            diagnostics={"profile_support": profile_support, "longitudinal_support_fraction": float(support_fraction),
                         "direction_deg": float(result.row_model.angle_deg)},
        ))
    return tuple(proposals)


def _build_local_rows(result: LocalRowTrackResult, direction: np.ndarray,
                      navigation: NavigationMapResult) -> tuple[GlobalRowProposal, ...]:
    columns = np.arange(navigation.width, dtype=np.float64)
    rows = np.arange(navigation.height, dtype=np.float64)
    xx, yy = np.meshgrid(navigation.origin_x_m + (columns + .5) * navigation.resolution_m,
                         navigation.origin_y_m + (rows + .5) * navigation.resolution_m)
    observed = np.asarray(navigation.point_count) > 0
    perpendicular = np.array([-direction[1], direction[0]])
    u_grid = xx * direction[0] + yy * direction[1]
    scene_span = max(1e-9, float(np.ptp(u_grid[observed]))) if np.any(observed) else 1e-9
    proposals = []
    for ordinal, track in enumerate(sorted(result.tracks, key=lambda item: item.representative_v_m), start=1):
        line = []
        for observation in sorted(track.observations, key=lambda item: (item.u_center_m, item.window_index)):
            point = _xy(direction, observation.u_center_m, observation.v_center_m)
            if not line or np.linalg.norm(np.asarray(point) - np.asarray(line[-1])) > 1e-6:
                line.append(tuple(point))
        if len(line) < 2:
            continue
        support = float(track.mean_support)
        proposals.append(GlobalRowProposal(
            auto_id=f"AUTO-L{ordinal:03d}", direction_xy=(float(direction[0]), float(direction[1])),
            lateral_v_m=float(track.representative_v_m), centerline_xy=tuple(line),
            half_width_m=float(result.config.row_structural_half_width_m), profile_support=support,
            longitudinal_support_fraction=min(1.0, track.longitudinal_span_m / scene_span),
            diagnostics={"track_id": int(track.row_id), "observation_count": len(track.observations),
                         "longitudinal_span_m": float(track.longitudinal_span_m),
                         "mean_support": support, "observations": [_plain(item) for item in track.observations]},
            source_mode="LOCAL_TRACKS",
        ))
    return tuple(proposals)


def _build_aisles(rows: tuple[GlobalRowProposal, ...], direction: np.ndarray,
                  minimum_width_m: float, side_clearance_m: float = 0.0,
                  minimum_overlap_m: float = 0.0) -> tuple[AisleProposal, ...]:
    ordered = sorted(rows, key=lambda row: row.lateral_v_m)
    output = []
    for index, (left, right) in enumerate(zip(ordered[:-1], ordered[1:]), start=1):
        separation = right.lateral_v_m - left.lateral_v_m
        structural_reserved = left.half_width_m + right.half_width_m
        side_reserved = 2.0 * float(side_clearance_m)
        available = separation - structural_reserved - side_reserved
        lower_v = left.lateral_v_m + left.half_width_m + float(side_clearance_m)
        upper_v = right.lateral_v_m - right.half_width_m - float(side_clearance_m)
        middle_v = .5 * (lower_v + upper_v)
        left_u = [float(np.dot(np.asarray(p), direction)) for p in left.centerline_xy]
        right_u = [float(np.dot(np.asarray(p), direction)) for p in right.centerline_xy]
        start_u = max(min(left_u), min(right_u))
        end_u = min(max(left_u), max(right_u))
        overlap = max(0.0, end_u - start_u)
        line = ((_xy(direction, start_u, middle_v), _xy(direction, end_u, middle_v))
                if overlap >= float(minimum_overlap_m) else ())
        if available < float(minimum_width_m):
            status = "REJECTED_TOO_NARROW"
        elif overlap < float(minimum_overlap_m):
            status = "REJECTED_NO_LONGITUDINAL_OVERLAP"
        else:
            status = "CANDIDATE"
        output.append(AisleProposal(
            auto_id=f"AUTO-A{index:03d}", left_row_auto_id=left.auto_id, right_row_auto_id=right.auto_id,
            aisle_kind="INTERIOR", centerline_xy=tuple(tuple(p) for p in line), safe_centerline_xy=(),
            geometric_width_m=float(available), minimum_width_m=float(minimum_width_m),
            diagnostics={"adjacent_auto_rows": [left.auto_id, right.auto_id],
                         "row_separation_m": float(separation),
                         "structural_reserved_width_m": float(structural_reserved),
                         "side_clearance_reserved_m": float(side_reserved),
                         "available_width_m": float(available),
                         "longitudinal_overlap_m": float(overlap),
                         "minimum_longitudinal_overlap_m": float(minimum_overlap_m),
                         "minimum_width_m": float(minimum_width_m), "status": status},
        ))
    return tuple(output)


def _local_structure_context(navigation: NavigationMapResult,
                             config: GreenhouseStructureConfig,
                             row_direction_xy: Iterable[float] | None) -> tuple[NavigationStructureResult, np.ndarray]:
    """Build shared ground/frame evidence without running global row peaks."""
    ndimage, _signal = _require_structure_scipy()
    robust_slope, residual = _robust_local_plane_slope(navigation, config.global_profile, ndimage)
    confidence = _ground_confidence(navigation, residual, config.global_profile)
    terrain_ridge = _terrain_ridge_evidence(navigation, confidence, config.global_profile, ndimage)
    hybrid = _hybrid_row_evidence(navigation, terrain_ridge, config.global_profile)
    direction = canonical_row_direction(
        row_direction_xy if row_direction_xy is not None else _auto_row_direction(navigation, hybrid)
    )
    shape = np.asarray(navigation.occupancy).shape
    zeros = np.zeros(shape, dtype=np.float64)
    angle = float(np.degrees(np.arctan2(direction[1], direction[0])))
    row_model = RowModel(direction, angle, (), float(config.global_profile.row_half_width_m), ())
    context = NavigationStructureResult(
        ground_confidence=confidence, robust_slope_deg=robust_slope,
        robust_plane_residual_m=residual, row_support=zeros,
        row_regularized_obstacle=zeros.astype(bool), aisle_candidate=zeros.astype(bool),
        row_model=row_model, config=config.global_profile,
    )
    return context, direction


def row_detection_navigation(navigation, polygons):
    """Mask only the detector input; retain original wall occupancy for corridors."""
    excluded = np.zeros_like(navigation.ground_valid, dtype=bool)
    if not polygons:
        return navigation, excluded
    rows, cols = np.indices(excluded.shape)
    xy = np.column_stack((
        (navigation.origin_x_m + (cols.ravel()+.5)*navigation.resolution_m),
        (navigation.origin_y_m + (rows.ravel()+.5)*navigation.resolution_m)))
    for polygon in polygons:
        excluded |= points_in_polygon(xy, polygon).reshape(excluded.shape)
    return replace(navigation,
        ground_valid=navigation.ground_valid & ~excluded,
        point_count=np.where(excluded, 0, navigation.point_count),
        ground_support_count=np.where(excluded, 0, navigation.ground_support_count),
        obstacle_count=np.where(excluded, 0, navigation.obstacle_count)), excluded


def analyze_greenhouse_structure(
    cloud: Any,
    input_map_hash: str,
    *,
    mode: str = "COMPARE",
    config: GreenhouseStructureConfig | None = None,
    row_direction_xy: Iterable[float] | None = None,
) -> GreenhouseStructureAnalysis:
    """Run global and local evidence chains without fusing their row sets.

    ``COMPARE`` computes both chains for side-by-side review. If no direction is
    supplied, each chain estimates the same canonical row axis from shared map
    evidence; local peaks and track IDs are then generated from windows alone.
    """
    mode = str(mode).upper()
    if mode not in ANALYSIS_MODES:
        raise ValueError(f"mode must be one of {ANALYSIS_MODES}")
    cfg = config or GreenhouseStructureConfig()
    cfg.validate()
    input_point_count = int(len(cloud.points)) if hasattr(cloud, "points") else int(len(cloud))
    stage_timings: dict[str, float] = {}
    stage_start = time.perf_counter()
    navigation = derive_ground_relative_navigation_map(cloud, cfg.navigation)
    stage_timings["navigation_map_derivation_seconds"] = time.perf_counter() - stage_start
    detection_navigation, excluded = row_detection_navigation(navigation, cfg.row_exclusion_polygons_xy)
    global_result = None
    local_result = None
    terrain = None
    geometric = None
    corridor_result = None
    global_rows: tuple[GlobalRowProposal, ...] = ()
    local_rows: tuple[GlobalRowProposal, ...] = ()
    direction_source = "manual" if row_direction_xy is not None else "independent_auto_axis_estimate"

    if mode in ("GLOBAL_PROFILE", "COMPARE"):
        stage_start = time.perf_counter()
        global_result = derive_navigation_structure(
            detection_navigation, cfg.global_profile, row_direction_xy=row_direction_xy,
        )
        global_rows = _build_global_rows(global_result, detection_navigation)
        stage_timings["global_row_model_seconds"] = time.perf_counter() - stage_start
        stage_start = time.perf_counter()
        global_terrain = derive_terrain_morphology(
            navigation, global_result.ground_confidence, cfg.terrain
        )
        terrain = GreenhouseTerrainEvidence(
            ground_height_m=navigation.ground_height_m, ground_valid=navigation.ground_valid,
            point_support_count=navigation.point_count, ground_support_count=navigation.ground_support_count,
            obstacle_count=navigation.obstacle_count, ground_confidence=global_result.ground_confidence,
            robust_slope_deg=global_result.robust_slope_deg,
            robust_plane_residual_m=global_result.robust_plane_residual_m,
            terrain_background_height_m=global_terrain.background_height_m,
            signed_relief_m=global_terrain.signed_relief_m,
            ridge_evidence=global_terrain.ridge_evidence,
            depression_evidence=global_terrain.depression_evidence,
            step_evidence=global_terrain.step_evidence,
        )
        stage_timings["terrain_morphology_seconds"] = time.perf_counter() - stage_start

    local_direction = None
    local_context = None
    if mode in ("LOCAL_TRACKS", "COMPARE"):
        stage_start = time.perf_counter()
        local_context, local_direction = _local_structure_context(detection_navigation, cfg, row_direction_xy)
        if terrain is None:
            terrain_start = time.perf_counter()
            local_terrain = derive_terrain_morphology(
                navigation, local_context.ground_confidence, cfg.terrain
            )
            terrain = GreenhouseTerrainEvidence(
                ground_height_m=navigation.ground_height_m, ground_valid=navigation.ground_valid,
                point_support_count=navigation.point_count, ground_support_count=navigation.ground_support_count,
                obstacle_count=navigation.obstacle_count, ground_confidence=local_context.ground_confidence,
                robust_slope_deg=local_context.robust_slope_deg,
                robust_plane_residual_m=local_context.robust_plane_residual_m,
                terrain_background_height_m=local_terrain.background_height_m,
                signed_relief_m=local_terrain.signed_relief_m,
                ridge_evidence=local_terrain.ridge_evidence,
                depression_evidence=local_terrain.depression_evidence,
                step_evidence=local_terrain.step_evidence,
            )
            stage_timings["terrain_morphology_seconds"] = time.perf_counter() - terrain_start
        stage_timings["local_evidence_context_seconds"] = time.perf_counter() - stage_start
        stage_start = time.perf_counter()
        local_result = derive_local_row_tracks(
            detection_navigation, local_context, cfg.local_tracks, row_direction_xy=local_direction
        )
        local_rows = _build_local_rows(local_result, local_direction, detection_navigation)
        stage_timings["local_row_tracks_seconds"] = time.perf_counter() - stage_start

    stage_start = time.perf_counter()
    if mode == "COMPARE" and global_result is not None:
        corridor_result = derive_corridor_refinement(navigation, global_result, cfg.corridor)
        geometric = derive_geometric_aisle_centerlines(navigation, global_result, corridor_result)

    proposals = local_rows if mode == "LOCAL_TRACKS" else global_rows
    direction = (local_direction if mode == "LOCAL_TRACKS" else
                 canonical_row_direction(global_result.row_model.direction_xy) if global_result is not None else
                 canonical_row_direction(row_direction_xy if row_direction_xy is not None else (1.0, 0.0)))
    aisles = _build_aisles(
        proposals, direction, cfg.corridor.aisle_minimum_width_m,
        cfg.corridor.aisle_side_clearance_m, cfg.corridor.minimum_row_longitudinal_span_m,
    )
    if mode == "COMPARE" and geometric is not None and corridor_result is not None:
        aisles = _attach_aisle_centerlines(
            aisles, proposals, navigation, direction, geometric.mask, corridor_result.aisle_centerline,
            cfg.corridor.aisle_side_clearance_m, corridor_result.aisle_pair_diagnostics,
        )
    elif mode == "LOCAL_TRACKS" and local_result is not None:
        aisles = _attach_aisle_centerlines(
            aisles, proposals, navigation, direction, local_result.aisle_candidate, local_result.aisle_centerline,
            cfg.local_tracks.aisle_side_clearance_m,
        )
    stage_timings["aisle_and_centerline_seconds"] = time.perf_counter() - stage_start
    normalized = {
        "mode": mode, "input_map_hash": input_map_hash, "config": cfg.to_dict(),
        "code_version": _code_version(), "direction_source": direction_source,
        "global_rows": _plain(global_rows), "local_rows": _plain(local_rows), "aisles": _plain(aisles),
        "navigation_counts": navigation.counts(),
    }
    analysis_hash = _canonical_hash(normalized)
    diagnostics = {
        "row_exclusion_polygon_count": len(cfg.row_exclusion_polygons_xy),
        "row_excluded_cell_count": int(np.count_nonzero(excluded)),
        "global_row_count": len(global_rows), "local_track_count": len(local_rows),
        "aisle_proposal_count": len(aisles), "navigation_cells": int(navigation.width * navigation.height),
        "direction_xy": [float(v) for v in direction],
        "global_direction_xy": ([float(v) for v in global_result.row_model.direction_xy]
                                 if global_result is not None else None),
        "local_direction_xy": (list(local_rows[0].direction_xy) if local_rows else
                               ([float(v) for v in local_direction] if local_direction is not None else None)),
        "direction_estimator": "profile contrast over the upper quartile of hybrid evidence, above minimum point support",
        "stage_timings_seconds": stage_timings,
        "input_point_count": input_point_count,
        "global_local_are_independent": True, "compare_fusion": False,
    }
    return GreenhouseStructureAnalysis(
        mode, input_map_hash, cfg, navigation, terrain, global_result, local_result, geometric, corridor_result,
        global_rows, local_rows, aisles, direction_source, diagnostics, _code_version(), analysis_hash,
    )


def decision_for_proposal(proposal: GlobalRowProposal, decision: str, *,
                          physical_row_id: str | None = None,
                          direction: str | None = None) -> GlobalRowProposal:
    decision = decision.lower()
    if decision not in {"accepted", "rejected", "pending"}:
        raise ValueError("proposal decision must be accepted, rejected, or pending")
    if decision == "accepted":
        if not physical_row_id or not physical_row_id.strip():
            raise ValueError("accepted rows require a manually assigned physical row ID")
        if direction not in {"forward", "reverse", "bidirectional"}:
            raise ValueError("accepted rows require a manually confirmed direction")
    elif physical_row_id or direction:
        raise ValueError("physical ID and direction can only be assigned when accepting a row")
    return GlobalRowProposal(**{**asdict(proposal), "decision": decision,
                                "physical_row_id": physical_row_id.strip() if physical_row_id else None,
                                "confirmed_direction": direction})


def merge_row_proposals(rows: Iterable[GlobalRowProposal], auto_id: str) -> GlobalRowProposal:
    items = sorted(tuple(rows), key=lambda item: item.lateral_v_m)
    if len(items) < 2:
        raise ValueError("merge requires at least two row proposals")
    direction = canonical_row_direction(items[0].direction_xy)
    if any(abs(float(np.dot(direction, canonical_row_direction(item.direction_xy)))) < 1.0 - 1e-6 for item in items[1:]):
        raise ValueError("cannot merge rows with different directions")
    lateral = float(np.mean([item.lateral_v_m for item in items]))
    starts = [item.centerline_xy[0] for item in items]
    ends = [item.centerline_xy[-1] for item in items]
    centerline = (tuple(np.mean(starts, axis=0).tolist()), tuple(np.mean(ends, axis=0).tolist()))
    return GlobalRowProposal(
        auto_id=auto_id, direction_xy=tuple(direction), lateral_v_m=lateral, centerline_xy=centerline,
        half_width_m=max(item.half_width_m for item in items),
        profile_support=float(np.mean([item.profile_support for item in items])),
        longitudinal_support_fraction=float(np.mean([item.longitudinal_support_fraction for item in items])),
        diagnostics={"merged_auto_ids": [item.auto_id for item in items]},
    )


def split_row_proposal(row: GlobalRowProposal, *, split_fraction: float = 0.5,
                       first_auto_id: str, second_auto_id: str) -> tuple[GlobalRowProposal, GlobalRowProposal]:
    if not 0.0 < split_fraction < 1.0:
        raise ValueError("split_fraction must be between 0 and 1")
    start, end = (np.asarray(point, dtype=np.float64) for point in row.centerline_xy)
    middle = start + (end - start) * split_fraction
    pieces = ((start, middle), (middle, end))
    output = []
    for auto_id, (a, b) in zip((first_auto_id, second_auto_id), pieces):
        output.append(GlobalRowProposal(
            auto_id=auto_id, direction_xy=row.direction_xy, lateral_v_m=row.lateral_v_m,
            centerline_xy=(tuple(a.tolist()), tuple(b.tolist())), half_width_m=row.half_width_m,
            profile_support=row.profile_support, longitudinal_support_fraction=row.longitudinal_support_fraction,
            diagnostics={"split_from_auto_id": row.auto_id, "split_fraction": split_fraction},
            source_mode=row.source_mode,
        ))
    return tuple(output)


def save_proposal_revision(analysis: GreenhouseStructureAnalysis, site_workspace: str | Path,
                           source_provenance: Mapping[str, Any] | None = None) -> Path:
    root = Path(site_workspace).expanduser().resolve() / "greenhouse_structure" / "drafts"
    root.mkdir(parents=True, exist_ok=True)
    revisions = [int(path.name.split("-")[1]) for path in root.glob("revision-*")
                 if path.is_dir() and path.name.split("-")[1].isdigit()]
    revision = max(revisions, default=0) + 1
    destination = root / f"revision-{revision:04d}-{analysis.analysis_hash[:12]}"
    if destination.exists():
        raise FileExistsError(f"proposal revision already exists: {destination}")
    destination.mkdir()
    payload = {
        "schema": ARTIFACT_SCHEMA, "revision": revision, "analysis_hash": analysis.analysis_hash,
        "created_at": _now(), "input_map_hash": analysis.input_map_hash,
        "code_version": analysis.code_version, "mode": analysis.mode,
        "provenance": {
            **_plain(dict(source_provenance or {})),
            "algorithm_version": analysis.code_version,
            "legacy_source_branch": LEGACY_SOURCE_BRANCH,
            "legacy_source_commit": LEGACY_SOURCE_COMMIT,
            "proposal_hash": analysis.analysis_hash,
            "annotation_revision": revision,
        },
        "config": analysis.config.to_dict(), "direction_source": analysis.direction_source,
        "global_rows": _plain(analysis.global_rows), "local_rows": _plain(analysis.local_rows),
        "aisle_proposals": _plain(analysis.aisle_proposals), "diagnostics": _plain(analysis.diagnostics),
    }
    target = destination / "proposal.yaml"
    target.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    (destination / "revision.json").write_text(json.dumps({
        "revision": revision, "analysis_hash": analysis.analysis_hash,
        "created_at": payload["created_at"], "proposal_path": target.name,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return target


def load_proposal_artifact(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema") != ARTIFACT_SCHEMA:
        raise ValueError(f"unsupported greenhouse proposal artifact: {source}")
    for collection in ("global_rows", "local_rows", "aisle_proposals"):
        if not isinstance(payload.get(collection), list):
            raise ValueError(f"proposal {collection} must be a list")
    return payload


def freeze_accepted_proposals(topology: Mapping[str, Any], rows: Iterable[GlobalRowProposal],
                              aisles: Iterable[AisleProposal], output_path: str | Path,
                              *, package: Any | None = None) -> dict[str, Any]:
    """Return a schema-v1 topology with reviewed proposal geometry; never overwrite."""
    target = Path(output_path).expanduser().resolve()
    if target.exists():
        raise FileExistsError(f"frozen topology is immutable: {target}")
    result = _plain(dict(topology))
    if result.get("schema_version") != 1:
        raise ValueError("frozen topology must use existing schema v1")
    existing = {str(row.get("id")) for row in result.get("rows", [])}
    selected = [row for row in rows if row.decision == "accepted"]
    auto_to_physical: dict[str, str] = {}
    row_centers = sorted(selected, key=lambda row: row.lateral_v_m)
    for row in row_centers:
        physical_id = str(row.physical_row_id or "").strip()
        if not physical_id or not row.confirmed_direction:
            raise ValueError(f"{row.auto_id} needs manually assigned physical ID and direction")
        if physical_id in existing:
            raise ValueError(f"duplicate physical row ID: {physical_id}")
        existing.add(physical_id)
        auto_to_physical[row.auto_id] = physical_id
        result.setdefault("rows", []).append({
            "id": physical_id, "centerline": _plain(row.centerline_xy),
            "nominal_width_m": 2.0 * float(row.half_width_m),
            "direction": row.confirmed_direction, "confidence": "confirmed",
            "proposal_source": {"auto_id": row.auto_id, "analysis_mode": row.source_mode},
            "proposal_diagnostics": _plain(row.diagnostics),
        })
    aisle_ids = {str(item.get("id")) for item in result.get("aisles", [])}
    for aisle in aisles:
        if aisle.decision != "accepted":
            continue
        if aisle.geometric_width_m < aisle.minimum_width_m:
            raise ValueError(f"{aisle.auto_id} is narrower than its required minimum")
        if aisle.aisle_kind == "BOUNDARY":
            if aisle.boundary_anchor_xy is None:
                raise ValueError("boundary aisle requires an explicit manual boundary anchor")
            if aisle.left_row_auto_id not in auto_to_physical:
                raise ValueError("boundary aisle requires its adjacent row to be manually accepted")
        else:
            if aisle.left_row_auto_id not in auto_to_physical or aisle.right_row_auto_id not in auto_to_physical:
                raise ValueError("formal aisle requires both adjacent rows to be manually accepted")
            selected_order = [row.auto_id for row in row_centers]
            pair = (aisle.left_row_auto_id, aisle.right_row_auto_id)
            if pair not in set(zip(selected_order[:-1], selected_order[1:])):
                raise ValueError("formal aisle must join adjacent accepted rows")
        aisle_id = str(aisle.physical_aisle_id or "").strip()
        if not aisle_id or aisle_id in aisle_ids:
            raise ValueError("accepted aisles require a unique manually assigned physical ID")
        aisle_ids.add(aisle_id)
        result.setdefault("aisles", []).append({
            "id": aisle_id, "left_row_id": auto_to_physical[aisle.left_row_auto_id],
            "right_row_id": auto_to_physical.get(aisle.right_row_auto_id),
            "boundary_anchor_xy": _plain(aisle.boundary_anchor_xy),
            "centerline": _plain(aisle.centerline_xy), "confidence": "confirmed",
            "proposal_source": {"auto_id": aisle.auto_id}, "diagnostics": _plain(aisle.diagnostics),
        })
    result.setdefault("aisles", [])
    result.setdefault("headlands", [])
    result.setdefault("scenes", [])
    result.setdefault("annotation", {})
    result["annotation"].update(status="frozen", manual_review_confirmed=True, frozen_at=_now())
    result["annotation"].setdefault("annotator", "manual")
    result["annotation"].setdefault("absolute_ground_truth", False)
    if package is not None:
        # Existing schema-v1 validator and sidecar exporter remain authoritative.
        from .topology import save_topology
        from .validator import validate_topology
        validation = validate_topology(result, package)
        if not validation["valid"]:
            raise ValueError("frozen topology validation failed: " + "; ".join(validation["errors"]))
        save_topology(result, target, package)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return result


def proposal_aisles_for_rows(rows: Iterable[GlobalRowProposal], config: GreenhouseStructureConfig) -> tuple[AisleProposal, ...]:
    """Return draft interior aisles between adjacent row proposals."""
    items = tuple(rows)
    direction = items[0].direction_xy if items else (1.0, 0.0)
    return _build_aisles(
        items, canonical_row_direction(direction), config.corridor.aisle_minimum_width_m,
        config.corridor.aisle_side_clearance_m, config.corridor.minimum_row_longitudinal_span_m,
    )


def make_boundary_aisle_proposal(row: GlobalRowProposal, anchor_xy: Iterable[float],
                                 *, auto_id: str, minimum_width_m: float = 0.45) -> AisleProposal:
    """Create a boundary corridor only from an explicit manually placed anchor."""
    anchor = np.asarray(tuple(anchor_xy), dtype=np.float64).reshape(2)
    if not np.isfinite(anchor).all():
        raise ValueError("boundary anchor must be a finite XY point")
    direction = canonical_row_direction(row.direction_xy)
    perpendicular = np.array([-direction[1], direction[0]])
    anchor_v = float(np.dot(anchor, perpendicular))
    row_v = float(row.lateral_v_m)
    available = abs(anchor_v - row_v) - row.half_width_m
    if available <= 0.0:
        raise ValueError("boundary anchor must be outside the accepted row band")
    middle_v = (anchor_v + row_v) / 2.0
    u_values = [float(np.dot(np.asarray(point), direction)) for point in row.centerline_xy]
    line = (_xy(direction, min(u_values), middle_v), _xy(direction, max(u_values), middle_v))
    return AisleProposal(
        auto_id=auto_id, left_row_auto_id=row.auto_id, right_row_auto_id="BOUNDARY_ANCHOR",
        aisle_kind="BOUNDARY", centerline_xy=tuple(tuple(point) for point in line),
        safe_centerline_xy=(), geometric_width_m=float(available),
        minimum_width_m=float(minimum_width_m),
        diagnostics={"anchor_xy": anchor.tolist(), "anchor_v_m": anchor_v,
                     "adjacent_row_auto_id": row.auto_id,
                     "status": "CANDIDATE" if available >= minimum_width_m else "REJECTED_TOO_NARROW"},
        boundary_anchor_xy=tuple(anchor.tolist()),
    )


def save_review_revision(site_workspace: str | Path, analysis_hash: str,
                         rows: Iterable[GlobalRowProposal],
                         aisles: Iterable[AisleProposal]) -> Path:
    """Persist a reviewed proposal state as a new append-only draft revision."""
    root = Path(site_workspace).expanduser().resolve() / "greenhouse_structure" / "reviews"
    root.mkdir(parents=True, exist_ok=True)
    numbers = [int(path.name.split("-")[1]) for path in root.glob("review-*")
               if path.is_dir() and path.name.split("-")[1].isdigit()]
    revision = max(numbers, default=0) + 1
    directory = root / f"review-{revision:04d}-{analysis_hash[:12]}"
    directory.mkdir()
    payload = {
        "schema": "agt_greenhouse_structure_review/v1", "revision": revision,
        "analysis_hash": analysis_hash, "created_at": _now(),
        "rows": _plain(tuple(rows)), "aisles": _plain(tuple(aisles)),
    }
    target = directory / "review.yaml"
    target.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return target
