"""Local agricultural row tracks and pair-anchored aisle candidates.

This module consumes a verified agricultural row direction.  It deliberately
avoids the single global cross-row profile assumption: row observations are
measured in overlapping longitudinal windows, associated into bounded tracks,
and aisles are derived only from adjacent accepted tracks.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .navigation_map_derivation import NavigationMapResult
from .navigation_structure import NavigationStructureResult
from .terrain_morphology import (
    TerrainMorphologyConfig,
    TerrainMorphologyResult,
    derive_terrain_morphology,
)


@dataclass(frozen=True)
class LocalRowTrackingConfig:
    window_length_m: float = 2.0
    window_stride_m: float = 0.75
    profile_bin_m: float = 0.10
    profile_smoothing_m: float = 0.15
    minimum_peak_spacing_m: float = 0.55
    minimum_peak_prominence_ratio: float = 0.08
    minimum_valid_cells_per_bin: int = 3
    minimum_peak_support: float = 0.12
    boundary_exclusion_m: float = 0.45
    association_distance_m: float = 0.45
    maximum_missed_windows: int = 1
    minimum_track_observations: int = 3
    minimum_track_span_m: float = 3.0
    row_structural_half_width_m: float = 0.20
    aisle_side_clearance_m: float = 0.12
    aisle_minimum_width_m: float = 0.45
    minimum_pair_overlap_m: float = 1.50
    obstacle_evidence_weight: float = 0.55
    terrain_ridge_evidence_weight: float = 0.45
    terrain_background_sigma_m: float = 0.45
    terrain_ridge_scale_m: float = 0.08
    terrain_depression_scale_m: float = 0.08
    terrain_step_scale_m: float = 0.10
    terrain_minimum_ground_confidence: float = 0.20
    aisle_review_minimum_ground_confidence: float = 0.30
    aisle_review_step_threshold: float = 0.60
    aisle_review_depression_threshold: float = 0.60

    def validate(self) -> None:
        if self.window_length_m <= 0.0:
            raise ValueError("window_length_m must be > 0")
        if self.window_stride_m <= 0.0:
            raise ValueError("window_stride_m must be > 0")
        if self.profile_bin_m <= 0.0:
            raise ValueError("profile_bin_m must be > 0")
        if self.profile_smoothing_m < 0.0:
            raise ValueError("profile_smoothing_m must be >= 0")
        if self.minimum_peak_spacing_m <= 0.0:
            raise ValueError("minimum_peak_spacing_m must be > 0")
        if not 0.0 <= self.minimum_peak_prominence_ratio <= 1.0:
            raise ValueError("minimum_peak_prominence_ratio must be in [0, 1]")
        if self.minimum_valid_cells_per_bin < 1:
            raise ValueError("minimum_valid_cells_per_bin must be >= 1")
        if not 0.0 <= self.minimum_peak_support <= 1.0:
            raise ValueError("minimum_peak_support must be in [0, 1]")
        if self.boundary_exclusion_m < 0.0:
            raise ValueError("boundary_exclusion_m must be >= 0")
        if self.association_distance_m <= 0.0:
            raise ValueError("association_distance_m must be > 0")
        if self.maximum_missed_windows < 0:
            raise ValueError("maximum_missed_windows must be >= 0")
        if self.minimum_track_observations < 2:
            raise ValueError("minimum_track_observations must be >= 2")
        if self.minimum_track_span_m <= 0.0:
            raise ValueError("minimum_track_span_m must be > 0")
        if self.row_structural_half_width_m <= 0.0:
            raise ValueError("row_structural_half_width_m must be > 0")
        if self.aisle_side_clearance_m < 0.0:
            raise ValueError("aisle_side_clearance_m must be >= 0")
        if self.aisle_minimum_width_m <= 0.0:
            raise ValueError("aisle_minimum_width_m must be > 0")
        if self.minimum_pair_overlap_m <= 0.0:
            raise ValueError("minimum_pair_overlap_m must be > 0")
        if self.obstacle_evidence_weight < 0.0:
            raise ValueError("obstacle_evidence_weight must be >= 0")
        if self.terrain_ridge_evidence_weight < 0.0:
            raise ValueError("terrain_ridge_evidence_weight must be >= 0")
        if self.obstacle_evidence_weight + self.terrain_ridge_evidence_weight <= 0.0:
            raise ValueError("row evidence weights must have a positive sum")
        for name, value in (
            ("terrain_minimum_ground_confidence", self.terrain_minimum_ground_confidence),
            ("aisle_review_minimum_ground_confidence", self.aisle_review_minimum_ground_confidence),
            ("aisle_review_step_threshold", self.aisle_review_step_threshold),
            ("aisle_review_depression_threshold", self.aisle_review_depression_threshold),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")


@dataclass(frozen=True)
class LocalRowObservation:
    window_index: int
    u_center_m: float
    v_center_m: float
    support: float
    prominence: float
    valid_cell_count: int
    support_u_min_m: float
    support_u_max_m: float


@dataclass(frozen=True)
class RowTrack:
    row_id: int
    observations: tuple[LocalRowObservation, ...]
    u_min_m: float
    u_max_m: float
    representative_v_m: float
    mean_support: float

    @property
    def longitudinal_span_m(self) -> float:
        return float(self.u_max_m - self.u_min_m)


@dataclass(frozen=True)
class AisleSegmentDiagnostic:
    pair_index: int
    left_row_id: int
    right_row_id: int
    longitudinal_overlap_m: float
    minimum_width_m: float
    median_width_m: float
    minimum_required_width_m: float
    terrain_review_fraction: float
    status: str


@dataclass(frozen=True)
class LocalRowTrackResult:
    observations: tuple[LocalRowObservation, ...]
    tracks: tuple[RowTrack, ...]
    row_centerline: np.ndarray
    row_structural_band: np.ndarray
    aisle_candidate: np.ndarray
    aisle_centerline: np.ndarray
    aisle_diagnostics: tuple[AisleSegmentDiagnostic, ...]
    hybrid_row_evidence: np.ndarray
    terrain_morphology: TerrainMorphologyResult
    config: LocalRowTrackingConfig


def _require_scipy():
    try:
        from scipy import ndimage, signal
    except ImportError as exc:
        raise RuntimeError("local row tracking requires scipy") from exc
    return ndimage, signal


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
    direction /= norm
    if direction[0] < 0.0 or (abs(direction[0]) < 1e-9 and direction[1] < 0.0):
        direction = -direction
    return direction


def _hybrid_row_evidence(
    navigation: NavigationMapResult,
    morphology: TerrainMorphologyResult,
    config: LocalRowTrackingConfig,
) -> np.ndarray:
    observed = np.asarray(navigation.point_count) > 0
    obstacle = np.log1p(np.asarray(navigation.obstacle_count, dtype=np.float64))
    obstacle[~observed] = 0.0
    maximum = float(np.max(obstacle)) if obstacle.size else 0.0
    if maximum > 0.0:
        obstacle /= maximum
    obstacle_weight = float(config.obstacle_evidence_weight)
    terrain_weight = float(config.terrain_ridge_evidence_weight)
    total = obstacle_weight + terrain_weight
    evidence = (
        obstacle_weight * obstacle
        + terrain_weight * np.asarray(morphology.ridge_evidence, dtype=np.float64)
    ) / total
    evidence[~observed] = 0.0
    return np.clip(evidence, 0.0, 1.0)


def _profile_edges(v_min: float, v_max: float, bin_m: float) -> np.ndarray:
    count = max(2, int(np.ceil((v_max - v_min) / bin_m)) + 1)
    return v_min + np.arange(count + 1, dtype=np.float64) * bin_m


def _local_observations(
    navigation: NavigationMapResult,
    evidence: np.ndarray,
    uu: np.ndarray,
    vv: np.ndarray,
    observed: np.ndarray,
    config: LocalRowTrackingConfig,
    ndimage,
    signal,
) -> tuple[LocalRowObservation, ...]:
    if not np.any(observed):
        return ()
    u_min = float(np.min(uu[observed]))
    u_max = float(np.max(uu[observed]))
    v_min = float(np.min(vv[observed]))
    v_max = float(np.max(vv[observed]))
    edges = _profile_edges(v_min, v_max, float(config.profile_bin_m))
    centers = 0.5 * (edges[:-1] + edges[1:])
    distance_bins = max(1, int(np.ceil(config.minimum_peak_spacing_m / config.profile_bin_m)))
    sigma_bins = float(config.profile_smoothing_m / config.profile_bin_m)

    starts = np.arange(
        u_min,
        max(u_min + navigation.resolution_m, u_max - config.window_length_m) + config.window_stride_m,
        config.window_stride_m,
        dtype=np.float64,
    )
    observations: list[LocalRowObservation] = []
    for window_index, start in enumerate(starts):
        end = min(float(start + config.window_length_m), u_max + navigation.resolution_m)
        window = observed & (uu >= start) & (uu <= end)
        if not np.any(window):
            continue
        support_count, _ = np.histogram(vv[window], bins=edges)
        weighted, _ = np.histogram(vv[window], bins=edges, weights=evidence[window])
        profile = np.divide(
            weighted,
            np.maximum(support_count, 1),
            out=np.zeros_like(weighted, dtype=np.float64),
            where=support_count > 0,
        )
        eligible = support_count >= int(config.minimum_valid_cells_per_bin)
        if sigma_bins > 0.0:
            profile = ndimage.gaussian_filter1d(profile, sigma=sigma_bins, mode="nearest")
        profile[~eligible] = 0.0
        maximum = float(np.max(profile)) if profile.size else 0.0
        if maximum < float(config.minimum_peak_support):
            continue
        prominence = float(config.minimum_peak_prominence_ratio) * maximum
        peaks, properties = signal.find_peaks(
            profile,
            height=float(config.minimum_peak_support),
            prominence=prominence,
            distance=distance_bins,
        )
        if peaks.size == 0:
            continue
        peak_prominence = properties.get("prominences", np.zeros(peaks.size))
        for index, peak in enumerate(peaks):
            center_v = float(centers[int(peak)])
            if center_v <= v_min + config.boundary_exclusion_m:
                continue
            if center_v >= v_max - config.boundary_exclusion_m:
                continue
            if support_count[int(peak)] < int(config.minimum_valid_cells_per_bin):
                continue
            band_half_width = max(config.profile_bin_m * 1.5, navigation.resolution_m * 1.5)
            support_cells = (
                window
                & (np.abs(vv - center_v) <= band_half_width)
                & (evidence >= max(0.02, 0.25 * float(config.minimum_peak_support)))
            )
            if not np.any(support_cells):
                continue
            observations.append(
                LocalRowObservation(
                    window_index=int(window_index),
                    u_center_m=float(0.5 * (start + end)),
                    v_center_m=center_v,
                    support=float(profile[int(peak)]),
                    prominence=float(peak_prominence[index]),
                    valid_cell_count=int(support_count[int(peak)]),
                    support_u_min_m=float(np.min(uu[support_cells])),
                    support_u_max_m=float(np.max(uu[support_cells])),
                )
            )
    return tuple(observations)


def _associate_tracks(
    observations: tuple[LocalRowObservation, ...],
    config: LocalRowTrackingConfig,
) -> tuple[RowTrack, ...]:
    if not observations:
        return ()
    by_window: dict[int, list[LocalRowObservation]] = {}
    for observation in observations:
        by_window.setdefault(observation.window_index, []).append(observation)

    candidates: list[list[LocalRowObservation]] = []
    last_window: list[int] = []
    for window_index in sorted(by_window):
        current = sorted(by_window[window_index], key=lambda item: item.v_center_m)
        available_tracks = {
            index
            for index, previous_window in enumerate(last_window)
            if window_index - previous_window <= config.maximum_missed_windows + 1
        }
        assigned_tracks: set[int] = set()
        assigned_observations: set[int] = set()
        pair_candidates: list[tuple[float, int, int]] = []
        for obs_index, observation in enumerate(current):
            for track_index in available_tracks:
                previous = candidates[track_index][-1]
                distance = abs(observation.v_center_m - previous.v_center_m)
                if distance <= config.association_distance_m:
                    pair_candidates.append((distance, track_index, obs_index))
        for _, track_index, obs_index in sorted(pair_candidates):
            if track_index in assigned_tracks or obs_index in assigned_observations:
                continue
            candidates[track_index].append(current[obs_index])
            last_window[track_index] = window_index
            assigned_tracks.add(track_index)
            assigned_observations.add(obs_index)
        for obs_index, observation in enumerate(current):
            if obs_index in assigned_observations:
                continue
            candidates.append([observation])
            last_window.append(window_index)

    accepted: list[RowTrack] = []
    for items in candidates:
        if len(items) < int(config.minimum_track_observations):
            continue
        u_min = float(min(item.support_u_min_m for item in items))
        u_max = float(max(item.support_u_max_m for item in items))
        if u_max - u_min < float(config.minimum_track_span_m):
            continue
        weights = np.asarray([max(item.support, 1e-6) for item in items], dtype=np.float64)
        representative_v = float(
            np.average([item.v_center_m for item in items], weights=weights)
        )
        accepted.append(
            RowTrack(
                row_id=0,
                observations=tuple(sorted(items, key=lambda item: item.u_center_m)),
                u_min_m=u_min,
                u_max_m=u_max,
                representative_v_m=representative_v,
                mean_support=float(np.mean(weights)),
            )
        )

    accepted.sort(key=lambda track: track.representative_v_m)
    return tuple(
        RowTrack(
            row_id=index + 1,
            observations=track.observations,
            u_min_m=track.u_min_m,
            u_max_m=track.u_max_m,
            representative_v_m=track.representative_v_m,
            mean_support=track.mean_support,
        )
        for index, track in enumerate(accepted)
    )


def _track_v(track: RowTrack, u_values: np.ndarray) -> np.ndarray:
    obs_u = np.asarray([item.u_center_m for item in track.observations], dtype=np.float64)
    obs_v = np.asarray([item.v_center_m for item in track.observations], dtype=np.float64)
    if obs_u.size == 1:
        return np.full(np.asarray(u_values).shape, obs_v[0], dtype=np.float64)
    return np.interp(np.asarray(u_values, dtype=np.float64), obs_u, obs_v)


def _rasterize_tracks(
    navigation: NavigationMapResult,
    tracks: tuple[RowTrack, ...],
    uu: np.ndarray,
    vv: np.ndarray,
    config: LocalRowTrackingConfig,
) -> tuple[np.ndarray, np.ndarray]:
    centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    band = np.zeros(navigation.occupancy.shape, dtype=bool)
    center_half_width = max(0.03, navigation.resolution_m * 0.55)
    for track in tracks:
        active = (uu >= track.u_min_m) & (uu <= track.u_max_m)
        if not np.any(active):
            continue
        predicted = _track_v(track, uu[active])
        delta = np.abs(vv[active] - predicted)
        local_center = np.zeros(active.shape, dtype=bool)
        local_band = np.zeros(active.shape, dtype=bool)
        local_center[active] = delta <= center_half_width
        local_band[active] = delta <= float(config.row_structural_half_width_m)
        centerline |= local_center
        band |= local_band
    return centerline, band


def _derive_aisles(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    morphology: TerrainMorphologyResult,
    tracks: tuple[RowTrack, ...],
    uu: np.ndarray,
    vv: np.ndarray,
    observed: np.ndarray,
    config: LocalRowTrackingConfig,
) -> tuple[np.ndarray, np.ndarray, tuple[AisleSegmentDiagnostic, ...]]:
    candidate = np.zeros(navigation.occupancy.shape, dtype=bool)
    centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    diagnostics: list[AisleSegmentDiagnostic] = []
    reserve = float(config.row_structural_half_width_m + config.aisle_side_clearance_m)
    center_half_width = max(0.03, navigation.resolution_m * 0.55)

    for pair_index, (left, right) in enumerate(zip(tracks[:-1], tracks[1:]), start=1):
        overlap_min = max(left.u_min_m, right.u_min_m)
        overlap_max = min(left.u_max_m, right.u_max_m)
        overlap = max(0.0, float(overlap_max - overlap_min))
        if overlap < float(config.minimum_pair_overlap_m):
            diagnostics.append(
                AisleSegmentDiagnostic(
                    pair_index=pair_index,
                    left_row_id=left.row_id,
                    right_row_id=right.row_id,
                    longitudinal_overlap_m=overlap,
                    minimum_width_m=0.0,
                    median_width_m=0.0,
                    minimum_required_width_m=float(config.aisle_minimum_width_m),
                    terrain_review_fraction=0.0,
                    status="REJECTED_NO_LONGITUDINAL_OVERLAP",
                )
            )
            continue

        sample_u = np.linspace(overlap_min, overlap_max, max(3, int(np.ceil(overlap / 0.25)) + 1))
        left_v = _track_v(left, sample_u)
        right_v = _track_v(right, sample_u)
        widths = right_v - left_v - 2.0 * reserve
        minimum_width = float(np.min(widths))
        median_width = float(np.median(widths))
        if minimum_width < float(config.aisle_minimum_width_m):
            diagnostics.append(
                AisleSegmentDiagnostic(
                    pair_index=pair_index,
                    left_row_id=left.row_id,
                    right_row_id=right.row_id,
                    longitudinal_overlap_m=overlap,
                    minimum_width_m=minimum_width,
                    median_width_m=median_width,
                    minimum_required_width_m=float(config.aisle_minimum_width_m),
                    terrain_review_fraction=0.0,
                    status="REJECTED_TOO_NARROW",
                )
            )
            continue

        active = observed & (uu >= overlap_min) & (uu <= overlap_max)
        local = np.zeros(active.shape, dtype=bool)
        local_center = np.zeros(active.shape, dtype=bool)
        if np.any(active):
            left_at = _track_v(left, uu[active]) + reserve
            right_at = _track_v(right, uu[active]) - reserve
            v_active = vv[active]
            inside = (v_active >= left_at) & (v_active <= right_at)
            midpoint = 0.5 * (left_at + right_at)
            near_center = inside & (np.abs(v_active - midpoint) <= center_half_width)
            local[active] = inside
            local_center[active] = near_center

        terrain_review = np.zeros(active.shape, dtype=bool)
        terrain_review |= np.asarray(structure.ground_confidence) < float(
            config.aisle_review_minimum_ground_confidence
        )
        terrain_review |= ~np.isfinite(np.asarray(structure.robust_slope_deg))
        terrain_review |= np.asarray(structure.robust_slope_deg) > float(
            navigation.config.maximum_slope_deg
        )
        terrain_review |= morphology.step_evidence >= float(
            config.aisle_review_step_threshold
        )
        terrain_review |= morphology.depression_evidence >= float(
            config.aisle_review_depression_threshold
        )
        review_cells = local & terrain_review
        review_fraction = (
            float(np.count_nonzero(review_cells)) / float(np.count_nonzero(local))
            if np.any(local)
            else 0.0
        )
        status = "ACCEPTED_REVIEW_TERRAIN" if review_fraction > 0.0 else "ACCEPTED"
        candidate |= local
        centerline |= local_center
        diagnostics.append(
            AisleSegmentDiagnostic(
                pair_index=pair_index,
                left_row_id=left.row_id,
                right_row_id=right.row_id,
                longitudinal_overlap_m=overlap,
                minimum_width_m=minimum_width,
                median_width_m=median_width,
                minimum_required_width_m=float(config.aisle_minimum_width_m),
                terrain_review_fraction=review_fraction,
                status=status,
            )
        )

    return candidate, centerline, tuple(diagnostics)


def derive_local_row_tracks(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    config: LocalRowTrackingConfig | None = None,
    *,
    row_direction_xy: Iterable[float] | None = None,
) -> LocalRowTrackResult:
    """Reconstruct local crop-row tracks and pair-anchored aisle candidates."""

    cfg = config or LocalRowTrackingConfig()
    cfg.validate()
    ndimage, signal = _require_scipy()
    if row_direction_xy is None:
        row_direction_xy = structure.row_model.direction_xy
    direction = _normalize_direction(row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    xx, yy = _grid_xy(navigation)
    uu = xx * direction[0] + yy * direction[1]
    vv = xx * perpendicular[0] + yy * perpendicular[1]
    observed = np.asarray(navigation.point_count) > 0

    morphology = derive_terrain_morphology(
        navigation,
        structure.ground_confidence,
        TerrainMorphologyConfig(
            background_sigma_m=float(cfg.terrain_background_sigma_m),
            ridge_scale_m=float(cfg.terrain_ridge_scale_m),
            depression_scale_m=float(cfg.terrain_depression_scale_m),
            step_scale_m=float(cfg.terrain_step_scale_m),
            minimum_ground_confidence=float(cfg.terrain_minimum_ground_confidence),
        ),
    )
    hybrid = _hybrid_row_evidence(navigation, morphology, cfg)
    observations = _local_observations(
        navigation,
        hybrid,
        uu,
        vv,
        observed,
        cfg,
        ndimage,
        signal,
    )
    tracks = _associate_tracks(observations, cfg)
    row_centerline, row_structural_band = _rasterize_tracks(
        navigation, tracks, uu, vv, cfg
    )
    aisle_candidate, aisle_centerline, aisle_diagnostics = _derive_aisles(
        navigation,
        structure,
        morphology,
        tracks,
        uu,
        vv,
        observed,
        cfg,
    )
    return LocalRowTrackResult(
        observations=observations,
        tracks=tracks,
        row_centerline=row_centerline,
        row_structural_band=row_structural_band,
        aisle_candidate=aisle_candidate,
        aisle_centerline=aisle_centerline,
        aisle_diagnostics=aisle_diagnostics,
        hybrid_row_evidence=hybrid,
        terrain_morphology=morphology,
        config=cfg,
    )
