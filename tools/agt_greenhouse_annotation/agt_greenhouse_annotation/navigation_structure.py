"""Agricultural structure analysis over a ground-relative navigation map.

This layer deliberately consumes the generic Ground-relative result instead of
changing point classification semantics. It adds scene-scale terrain evidence
and greenhouse row priors that can be previewed independently before any final
Navigation Map policy uses them.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Iterable

import numpy as np

from .navigation_map_derivation import NavigationMapResult


@dataclass(frozen=True)
class NavigationStructureConfig:
    robust_slope_window_m: float = 0.50
    robust_slope_minimum_cells: int = 9
    robust_slope_prefilter_radius_cells: int = 1
    ground_confidence_support_target: int = 6
    ground_confidence_residual_scale_m: float = 0.05
    row_direction_mode: str = "auto"
    row_profile_bin_m: float = 0.10
    row_profile_smoothing_m: float = 0.20
    row_minimum_spacing_m: float = 0.55
    row_minimum_prominence_ratio: float = 0.12
    # Optional physical consolidation of multiple profile peaks produced by one
    # canopy/ridge. Zero preserves the historical V2.5 behaviour; site presets
    # may enable it after the agricultural frame has been verified.
    row_hypothesis_merge_distance_m: float = 0.0
    row_half_width_m: float = 0.22
    row_max_gap_m: float = 0.60
    row_minimum_segment_length_m: float = 1.50
    row_minimum_support_fraction: float = 0.12
    row_obstacle_evidence_weight: float = 0.55
    row_terrain_evidence_weight: float = 0.45
    row_terrain_background_sigma_m: float = 0.45
    row_terrain_prominence_scale_m: float = 0.08
    row_terrain_minimum_ground_confidence: float = 0.20
    row_minimum_cell_evidence: float = 0.08
    aisle_minimum_ground_confidence: float = 0.30

    def validate(self) -> None:
        if self.robust_slope_window_m <= 0.0:
            raise ValueError("robust_slope_window_m must be > 0")
        if self.robust_slope_minimum_cells < 3:
            raise ValueError("robust_slope_minimum_cells must be >= 3")
        if self.robust_slope_prefilter_radius_cells < 0:
            raise ValueError("robust_slope_prefilter_radius_cells must be >= 0")
        if self.ground_confidence_support_target < 1:
            raise ValueError("ground_confidence_support_target must be >= 1")
        if self.ground_confidence_residual_scale_m <= 0.0:
            raise ValueError("ground_confidence_residual_scale_m must be > 0")
        if self.row_direction_mode not in {"auto", "provided"}:
            raise ValueError("row_direction_mode must be auto or provided")
        if self.row_profile_bin_m <= 0.0:
            raise ValueError("row_profile_bin_m must be > 0")
        if self.row_profile_smoothing_m < 0.0:
            raise ValueError("row_profile_smoothing_m must be >= 0")
        if self.row_minimum_spacing_m <= 0.0:
            raise ValueError("row_minimum_spacing_m must be > 0")
        if not 0.0 <= self.row_minimum_prominence_ratio <= 1.0:
            raise ValueError("row_minimum_prominence_ratio must be in [0, 1]")
        if self.row_hypothesis_merge_distance_m < 0.0:
            raise ValueError("row_hypothesis_merge_distance_m must be >= 0")
        if self.row_half_width_m <= 0.0:
            raise ValueError("row_half_width_m must be > 0")
        if self.row_max_gap_m < 0.0:
            raise ValueError("row_max_gap_m must be >= 0")
        if self.row_minimum_segment_length_m <= 0.0:
            raise ValueError("row_minimum_segment_length_m must be > 0")
        if not 0.0 <= self.row_minimum_support_fraction <= 1.0:
            raise ValueError("row_minimum_support_fraction must be in [0, 1]")
        if self.row_obstacle_evidence_weight < 0.0:
            raise ValueError("row_obstacle_evidence_weight must be >= 0")
        if self.row_terrain_evidence_weight < 0.0:
            raise ValueError("row_terrain_evidence_weight must be >= 0")
        if self.row_obstacle_evidence_weight + self.row_terrain_evidence_weight <= 0.0:
            raise ValueError("row evidence weights must have a positive sum")
        if self.row_terrain_background_sigma_m <= 0.0:
            raise ValueError("row_terrain_background_sigma_m must be > 0")
        if self.row_terrain_prominence_scale_m <= 0.0:
            raise ValueError("row_terrain_prominence_scale_m must be > 0")
        if not 0.0 <= self.row_terrain_minimum_ground_confidence <= 1.0:
            raise ValueError("row_terrain_minimum_ground_confidence must be in [0, 1]")
        if not 0.0 <= self.row_minimum_cell_evidence <= 1.0:
            raise ValueError("row_minimum_cell_evidence must be in [0, 1]")
        if not 0.0 <= self.aisle_minimum_ground_confidence <= 1.0:
            raise ValueError("aisle_minimum_ground_confidence must be in [0, 1]")


@dataclass(frozen=True)
class RowModel:
    direction_xy: np.ndarray
    angle_deg: float
    centers_v_m: tuple[float, ...]
    half_width_m: float
    support_fraction: tuple[float, ...]


@dataclass(frozen=True)
class NavigationStructureResult:
    ground_confidence: np.ndarray
    robust_slope_deg: np.ndarray
    robust_plane_residual_m: np.ndarray
    row_support: np.ndarray
    row_regularized_obstacle: np.ndarray
    aisle_candidate: np.ndarray
    row_model: RowModel
    config: NavigationStructureConfig


def _require_scipy():
    try:
        from scipy import ndimage, signal
    except ImportError as exc:
        raise RuntimeError("navigation structure analysis requires scipy") from exc
    return ndimage, signal


def _grid_xy(result: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    columns = np.arange(result.width, dtype=np.float64)
    rows = np.arange(result.height, dtype=np.float64)
    return np.meshgrid(
        result.origin_x_m + (columns + 0.5) * result.resolution_m,
        result.origin_y_m + (rows + 0.5) * result.resolution_m,
    )


def _robust_local_plane_slope(
    result: NavigationMapResult,
    config: NavigationStructureConfig,
    ndimage,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized local least-squares plane fit after a small median prefilter."""
    z = np.asarray(result.ground_height_m, dtype=np.float64)
    valid = np.isfinite(z) & np.asarray(result.ground_valid, dtype=bool)
    radius_prefilter = int(config.robust_slope_prefilter_radius_cells)
    if radius_prefilter > 0 and np.any(valid):
        _, nearest = ndimage.distance_transform_edt(~valid, return_indices=True)
        nearest_z = z[tuple(nearest)]
        filtered = ndimage.median_filter(
            nearest_z, size=2 * radius_prefilter + 1, mode="nearest"
        )
        z_fit = np.where(valid, filtered, 0.0)
    else:
        z_fit = np.where(valid, z, 0.0)

    xx, yy = _grid_xy(result)
    mask = valid.astype(np.float64)
    window_cells = max(3, int(round(config.robust_slope_window_m / result.resolution_m)))
    if window_cells % 2 == 0:
        window_cells += 1
    kernel = np.ones((window_cells, window_cells), dtype=np.float64)

    def conv(values: np.ndarray) -> np.ndarray:
        return ndimage.convolve(values, kernel, mode="constant", cval=0.0)

    n = conv(mask)
    sx = conv(mask * xx)
    sy = conv(mask * yy)
    sz = conv(z_fit)
    sxx = conv(mask * xx * xx)
    syy = conv(mask * yy * yy)
    sxy = conv(mask * xx * yy)
    sxz = conv(xx * z_fit)
    syz = conv(yy * z_fit)
    szz = conv(z_fit * z_fit)

    eligible = valid & (n >= float(config.robust_slope_minimum_cells))
    indices = np.flatnonzero(eligible.reshape(-1))
    slope = np.full(z.shape, np.nan, dtype=np.float64)
    residual = np.full(z.shape, np.nan, dtype=np.float64)
    if indices.size == 0:
        return slope, residual

    flat = lambda a: a.reshape(-1)[indices]
    matrices = np.empty((indices.size, 3, 3), dtype=np.float64)
    matrices[:, 0, 0] = flat(sxx)
    matrices[:, 0, 1] = flat(sxy)
    matrices[:, 0, 2] = flat(sx)
    matrices[:, 1, 0] = flat(sxy)
    matrices[:, 1, 1] = flat(syy)
    matrices[:, 1, 2] = flat(sy)
    matrices[:, 2, 0] = flat(sx)
    matrices[:, 2, 1] = flat(sy)
    matrices[:, 2, 2] = flat(n)
    vectors = np.column_stack((flat(sxz), flat(syz), flat(sz)))

    determinant = np.abs(np.linalg.det(matrices))
    stable = determinant > 1e-9
    coefficients = np.full((indices.size, 3), np.nan, dtype=np.float64)
    if np.any(stable):
        # NumPy 2 treats a stacked (..., M) RHS as a matrix stack instead of a
        # vector stack. Make the vector axis explicit and squeeze it back out.
        coefficients[stable] = np.linalg.solve(
            matrices[stable], vectors[stable][..., None]
        )[..., 0]
    a = coefficients[:, 0]
    b = coefficients[:, 1]
    slope_values = np.degrees(np.arctan(np.hypot(a, b)))
    explained = np.einsum("ij,ij->i", coefficients, vectors)
    rss = np.maximum(0.0, flat(szz) - explained)
    residual_values = np.sqrt(rss / np.maximum(flat(n), 1.0))
    slope.reshape(-1)[indices] = slope_values
    residual.reshape(-1)[indices] = residual_values
    return slope, residual


def _ground_confidence(
    result: NavigationMapResult,
    residual_m: np.ndarray,
    config: NavigationStructureConfig,
) -> np.ndarray:
    support = np.asarray(result.ground_support_count, dtype=np.float64)
    support_score = np.clip(
        support / float(config.ground_confidence_support_target), 0.0, 1.0
    )
    residual_score = np.exp(
        -np.nan_to_num(residual_m, nan=np.inf)
        / float(config.ground_confidence_residual_scale_m)
    )
    confidence = support_score * residual_score
    confidence[~np.asarray(result.ground_valid, dtype=bool)] = 0.0
    return np.clip(confidence, 0.0, 1.0)


def _terrain_ridge_evidence(
    result: NavigationMapResult,
    ground_confidence: np.ndarray,
    config: NavigationStructureConfig,
    ndimage,
) -> np.ndarray:
    """Extract raised crop-ridge relief after removing slowly varying terrain."""
    z = np.asarray(result.ground_height_m, dtype=np.float64)
    eligible = (
        np.asarray(result.ground_valid, dtype=bool)
        & np.isfinite(z)
        & (
            np.asarray(ground_confidence, dtype=np.float64)
            >= float(config.row_terrain_minimum_ground_confidence)
        )
    )
    evidence = np.zeros(z.shape, dtype=np.float64)
    if not np.any(eligible):
        return evidence

    _, nearest = ndimage.distance_transform_edt(~eligible, return_indices=True)
    filled = z[tuple(nearest)]
    sigma_cells = max(
        1.0, float(config.row_terrain_background_sigma_m) / result.resolution_m
    )
    background = ndimage.gaussian_filter(filled, sigma=sigma_cells, mode="nearest")
    prominence = np.maximum(0.0, z - background)
    evidence = np.clip(
        prominence / float(config.row_terrain_prominence_scale_m), 0.0, 1.0
    )
    evidence *= np.asarray(ground_confidence, dtype=np.float64)
    evidence[~eligible] = 0.0
    return evidence


def _hybrid_row_evidence(
    result: NavigationMapResult,
    terrain_ridge_evidence: np.ndarray,
    config: NavigationStructureConfig,
) -> np.ndarray:
    obstacle = np.log1p(np.asarray(result.obstacle_count, dtype=np.float64))
    observed = np.asarray(result.point_count) > 0
    obstacle[~observed] = 0.0
    obstacle_max = float(np.max(obstacle)) if obstacle.size else 0.0
    if obstacle_max > 0.0:
        obstacle /= obstacle_max
    obstacle_weight = float(config.row_obstacle_evidence_weight)
    terrain_weight = float(config.row_terrain_evidence_weight)
    total = obstacle_weight + terrain_weight
    hybrid = (
        obstacle_weight * obstacle
        + terrain_weight * np.asarray(terrain_ridge_evidence, dtype=np.float64)
    ) / total
    hybrid[~observed] = 0.0
    return np.clip(hybrid, 0.0, 1.0)


def _normalize_direction(direction_xy: Iterable[float]) -> np.ndarray:
    direction = np.asarray(tuple(direction_xy), dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("row direction must be a finite non-zero XY vector")
    direction /= norm
    if direction[0] < 0.0 or (abs(direction[0]) < 1e-9 and direction[1] < 0.0):
        direction = -direction
    return direction


def _auto_row_direction(
    result: NavigationMapResult,
    evidence_weight: np.ndarray,
) -> np.ndarray:
    """Estimate dominant row direction, suppressing sparsely supported outliers.

    Weak hybrid-evidence cells can be spread across a large survey envelope
    and dominate a profile-contrast score despite carrying little structural
    evidence. Estimate the axis from the upper quartile of hybrid evidence,
    after the map's minimum point-support check. This keeps the legacy profile
    score while making its direction estimate robust to sparse map fringes.
    """
    xx, yy = _grid_xy(result)
    weight = np.asarray(evidence_weight, dtype=np.float64)
    valid = weight > 0.05
    if np.count_nonzero(valid) < 20:
        return np.array([1.0, 0.0], dtype=np.float64)
    point_count = np.asarray(result.point_count)
    evidence_floor = float(np.quantile(weight[valid], 0.75))
    supported = valid & (weight >= evidence_floor)
    supported &= point_count >= int(result.config.minimum_cell_points)
    if np.count_nonzero(supported) >= 20:
        valid = supported
    x = xx[valid].copy()
    y = yy[valid].copy()
    w = weight[valid]
    x -= np.average(x, weights=w)
    y -= np.average(y, weights=w)
    best_angle = 0.0
    best_score = -np.inf
    for angle_deg in np.arange(0.0, 180.0, 2.0):
        theta = np.deg2rad(angle_deg)
        v = -x * np.sin(theta) + y * np.cos(theta)
        span = float(np.max(v) - np.min(v))
        if span < 0.5:
            continue
        bins = max(16, int(ceil(span / max(result.resolution_m, 0.10))))
        profile, _ = np.histogram(v, bins=bins, weights=w)
        mean = float(np.mean(profile))
        score = float(np.std(profile) / max(mean, 1e-9))
        if score > best_score:
            best_score = score
            best_angle = angle_deg
    theta = np.deg2rad(best_angle)
    return _normalize_direction((np.cos(theta), np.sin(theta)))


def _fill_short_gaps_1d(mask: np.ndarray, maximum_gap_bins: int) -> np.ndarray:
    output = np.asarray(mask, dtype=bool).copy()
    if maximum_gap_bins <= 0 or output.size == 0:
        return output
    padded = np.r_[False, output, False]
    starts = np.flatnonzero(~padded[:-1] & padded[1:])
    ends = np.flatnonzero(padded[:-1] & ~padded[1:])
    if starts.size < 2:
        return output
    for left_end, right_start in zip(ends[:-1], starts[1:]):
        gap = right_start - left_end
        if 0 < gap <= maximum_gap_bins:
            output[left_end:right_start] = True
    return output


def _remove_short_true_runs(mask: np.ndarray, minimum_bins: int) -> np.ndarray:
    output = np.asarray(mask, dtype=bool).copy()
    if output.size == 0:
        return output
    padded = np.r_[False, output, False]
    starts = np.flatnonzero(~padded[:-1] & padded[1:])
    ends = np.flatnonzero(padded[:-1] & ~padded[1:])
    for start, end in zip(starts, ends):
        if end - start < minimum_bins:
            output[start:end] = False
    return output


def _group_row_peaks(
    peaks: np.ndarray,
    centers: np.ndarray,
    *,
    merge_distance_m: float,
) -> tuple[np.ndarray, ...]:
    """Group nearby profile peaks without assuming a row count or period."""
    ordered = np.asarray(peaks, dtype=np.int64)
    if ordered.size == 0:
        return ()
    ordered = ordered[np.argsort(centers[ordered])]
    if merge_distance_m <= 0.0:
        return tuple(np.asarray([peak], dtype=np.int64) for peak in ordered)

    groups: list[list[int]] = [[int(ordered[0])]]
    for peak in ordered[1:]:
        current = int(peak)
        previous = groups[-1][-1]
        if float(centers[current] - centers[previous]) <= merge_distance_m:
            groups[-1].append(current)
        else:
            groups.append([current])
    return tuple(np.asarray(group, dtype=np.int64) for group in groups)


def _row_structure(
    result: NavigationMapResult,
    config: NavigationStructureConfig,
    direction_xy: np.ndarray,
    ground_confidence: np.ndarray,
    robust_slope_deg: np.ndarray,
    hybrid_evidence: np.ndarray,
    ndimage,
    signal,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, RowModel]:
    xx, yy = _grid_xy(result)
    direction = _normalize_direction(direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    uu = xx * direction[0] + yy * direction[1]
    vv = xx * perpendicular[0] + yy * perpendicular[1]

    weight = np.asarray(hybrid_evidence, dtype=np.float64)
    evidence = weight >= float(config.row_minimum_cell_evidence)
    if not np.any(evidence):
        empty = np.zeros(result.occupancy.shape, dtype=np.float64)
        angle_deg = float(np.degrees(np.arctan2(direction[1], direction[0])))
        model = RowModel(direction, angle_deg, (), config.row_half_width_m, ())
        return empty, empty.astype(bool), empty.astype(bool), model

    v_min = float(np.min(vv))
    v_max = float(np.max(vv))
    bin_m = max(float(config.row_profile_bin_m), result.resolution_m)
    bin_count = max(8, int(ceil((v_max - v_min) / bin_m)))
    profile, edges = np.histogram(
        vv[evidence],
        bins=bin_count,
        range=(v_min, v_max),
        weights=weight[evidence],
    )
    sigma = float(config.row_profile_smoothing_m) / max(bin_m, 1e-9)
    smooth_profile = ndimage.gaussian_filter1d(
        profile.astype(np.float64), sigma=max(0.0, sigma)
    )
    prominence = max(
        1e-9,
        float(np.max(smooth_profile)) * config.row_minimum_prominence_ratio,
    )
    distance_bins = max(1, int(round(config.row_minimum_spacing_m / bin_m)))
    peaks, _ = signal.find_peaks(
        smooth_profile,
        distance=distance_bins,
        prominence=prominence,
    )
    centers = 0.5 * (edges[:-1] + edges[1:])
    peak_groups = _group_row_peaks(
        peaks,
        centers,
        merge_distance_m=float(config.row_hypothesis_merge_distance_m),
    )

    u_min = float(np.min(uu))
    u_max = float(np.max(uu))
    u_bin_m = result.resolution_m
    u_bins = max(1, int(ceil((u_max - u_min) / u_bin_m)))
    max_gap_bins = int(round(config.row_max_gap_m / u_bin_m))
    minimum_segment_bins = max(
        1, int(round(config.row_minimum_segment_length_m / u_bin_m))
    )

    row_regularized = np.zeros(result.occupancy.shape, dtype=bool)
    row_support = np.zeros(result.occupancy.shape, dtype=np.float64)
    accepted_centers: list[float] = []
    support_fractions: list[float] = []
    profile_max = max(float(np.max(smooth_profile)), 1e-9)

    all_u_index = np.floor((uu - u_min) / u_bin_m).astype(np.int64)
    all_u_index = np.clip(all_u_index, 0, u_bins - 1)

    for group in peak_groups:
        peak_weights = np.maximum(smooth_profile[group], 1e-9)
        center_v = float(np.average(centers[group], weights=peak_weights))
        nominal_band = np.abs(vv - center_v) <= float(config.row_half_width_m)

        # When a single canopy forms two parallel profile edges, use both edge
        # neighborhoods only to establish longitudinal support. The produced row
        # remains one nominal structural band at the consolidated center.
        evidence_band = np.zeros(result.occupancy.shape, dtype=bool)
        for peak in group:
            evidence_band |= (
                np.abs(vv - float(centers[int(peak)]))
                <= float(config.row_half_width_m)
            )
        raw = evidence_band & evidence
        if not np.any(raw):
            continue
        u_index = np.floor((uu[raw] - u_min) / u_bin_m).astype(np.int64)
        u_index = np.clip(u_index, 0, u_bins - 1)
        u_support = np.bincount(u_index, minlength=u_bins) > 0
        support_fraction = float(np.count_nonzero(u_support) / max(1, u_bins))
        if support_fraction < config.row_minimum_support_fraction:
            continue
        repaired = _fill_short_gaps_1d(u_support, max_gap_bins)
        repaired = _remove_short_true_runs(repaired, minimum_segment_bins)
        mask = nominal_band & repaired[all_u_index]
        if not np.any(mask):
            continue
        row_regularized |= mask
        peak_score = float(np.max(smooth_profile[group]) / profile_max)
        row_support[nominal_band] = np.maximum(
            row_support[nominal_band], peak_score
        )
        accepted_centers.append(center_v)
        support_fractions.append(support_fraction)

    raw_obstacle = (
        np.asarray(result.obstacle_count)
        >= result.config.minimum_obstacle_points
    )
    acceptable_slope = np.isfinite(robust_slope_deg) & (
        robust_slope_deg <= result.config.maximum_slope_deg
    )
    aisle_candidate = (
        (ground_confidence >= config.aisle_minimum_ground_confidence)
        & acceptable_slope
        & ~row_regularized
        & ~raw_obstacle
    )
    angle_deg = float(np.degrees(np.arctan2(direction[1], direction[0])))
    model = RowModel(
        direction_xy=direction,
        angle_deg=angle_deg,
        centers_v_m=tuple(accepted_centers),
        half_width_m=float(config.row_half_width_m),
        support_fraction=tuple(support_fractions),
    )
    return row_support, row_regularized, aisle_candidate, model


def derive_navigation_structure(
    result: NavigationMapResult,
    config: NavigationStructureConfig | None = None,
    *,
    row_direction_xy: Iterable[float] | None = None,
) -> NavigationStructureResult:
    cfg = config or NavigationStructureConfig()
    cfg.validate()
    ndimage, signal = _require_scipy()
    robust_slope, residual = _robust_local_plane_slope(result, cfg, ndimage)
    confidence = _ground_confidence(result, residual, cfg)
    terrain_ridge = _terrain_ridge_evidence(result, confidence, cfg, ndimage)
    hybrid_evidence = _hybrid_row_evidence(result, terrain_ridge, cfg)
    if row_direction_xy is not None:
        direction = _normalize_direction(row_direction_xy)
    elif cfg.row_direction_mode == "provided":
        raise ValueError("row_direction_mode=provided requires row_direction_xy")
    else:
        direction = _auto_row_direction(result, hybrid_evidence)
    row_support, row_regularized, aisle_candidate, row_model = _row_structure(
        result,
        cfg,
        direction,
        confidence,
        robust_slope,
        hybrid_evidence,
        ndimage,
        signal,
    )
    return NavigationStructureResult(
        ground_confidence=confidence,
        robust_slope_deg=robust_slope,
        robust_plane_residual_m=residual,
        row_support=row_support,
        row_regularized_obstacle=row_regularized,
        aisle_candidate=aisle_candidate,
        row_model=row_model,
        config=cfg,
    )
