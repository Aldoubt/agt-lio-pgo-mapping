"""Deterministic ground-relative 2D navigation-map derivation from a static PCD.

The derivation intentionally avoids one global absolute-Z obstacle slice. It
estimates a locally continuous ground elevation surface, rejects unsupported or
elevated false-ground seeds, classifies point evidence relative to that surface,
and keeps unsupported cells UNKNOWN. The result is suitable for Workbench
preview and for later immutable Navigation Map materialization.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import ceil, floor, tan, radians
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np
import yaml

from .contracts import AssetContractError, sha256_file


NAVIGATION_DERIVATION_SCHEMA = "agt_ground_relative_navigation_map/v1"
FREE = np.uint8(254)
OCCUPIED = np.uint8(0)
UNKNOWN = np.uint8(205)


@dataclass(frozen=True)
class GroundRelativeNavigationConfig:
    resolution_m: float = 0.10
    padding_m: float = 0.30
    ground_quantile: float = 0.10
    minimum_cell_points: int = 3
    ground_seed_support_band_m: float = 0.08
    minimum_ground_seed_support_points: int = 2
    ground_continuity_radius_m: float = 0.80
    ground_reference_percentile: float = 20.0
    ground_seed_max_rise_m: float = 0.12
    ground_seed_max_drop_m: float = 0.20
    maximum_ground_fill_distance_m: float = 0.35
    ground_smoothing_radius_cells: int = 2
    ground_tolerance_m: float = 0.10
    minimum_ground_support_points: int = 2
    obstacle_min_height_m: float = 0.12
    obstacle_max_height_m: float = 1.00
    minimum_obstacle_points: int = 2
    maximum_slope_deg: float = 18.0
    maximum_step_m: float = 0.12
    obstacle_padding_m: float = 0.05

    def validate(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be > 0")
        if self.padding_m < 0.0:
            raise ValueError("padding_m must be >= 0")
        if not 0.0 <= self.ground_quantile <= 1.0:
            raise ValueError("ground_quantile must be in [0, 1]")
        if self.minimum_cell_points < 1:
            raise ValueError("minimum_cell_points must be >= 1")
        if self.ground_seed_support_band_m <= 0.0:
            raise ValueError("ground_seed_support_band_m must be > 0")
        if self.minimum_ground_seed_support_points < 1:
            raise ValueError("minimum_ground_seed_support_points must be >= 1")
        if self.ground_continuity_radius_m <= 0.0:
            raise ValueError("ground_continuity_radius_m must be > 0")
        if not 0.0 <= self.ground_reference_percentile <= 100.0:
            raise ValueError("ground_reference_percentile must be in [0, 100]")
        if self.ground_seed_max_rise_m < 0.0:
            raise ValueError("ground_seed_max_rise_m must be >= 0")
        if self.ground_seed_max_drop_m < 0.0:
            raise ValueError("ground_seed_max_drop_m must be >= 0")
        if self.maximum_ground_fill_distance_m < 0.0:
            raise ValueError("maximum_ground_fill_distance_m must be >= 0")
        if self.ground_smoothing_radius_cells < 0:
            raise ValueError("ground_smoothing_radius_cells must be >= 0")
        if self.ground_tolerance_m <= 0.0:
            raise ValueError("ground_tolerance_m must be > 0")
        if self.minimum_ground_support_points < 1:
            raise ValueError("minimum_ground_support_points must be >= 1")
        if self.obstacle_min_height_m < 0.0:
            raise ValueError("obstacle_min_height_m must be >= 0")
        if self.obstacle_max_height_m <= self.obstacle_min_height_m:
            raise ValueError("obstacle_max_height_m must be > obstacle_min_height_m")
        if self.minimum_obstacle_points < 1:
            raise ValueError("minimum_obstacle_points must be >= 1")
        if not 0.0 <= self.maximum_slope_deg < 90.0:
            raise ValueError("maximum_slope_deg must be in [0, 90)")
        if self.maximum_step_m <= 0.0:
            raise ValueError("maximum_step_m must be > 0")
        if self.obstacle_padding_m < 0.0:
            raise ValueError("obstacle_padding_m must be >= 0")


@dataclass(frozen=True)
class NavigationMapResult:
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int
    ground_height_m: np.ndarray
    ground_valid: np.ndarray
    point_count: np.ndarray
    ground_support_count: np.ndarray
    obstacle_count: np.ndarray
    slope_deg: np.ndarray
    step_m: np.ndarray
    occupancy: np.ndarray
    config: GroundRelativeNavigationConfig
    ground_seed_candidate_count: int = 0
    ground_seed_trusted_count: int = 0
    ground_seed_rejected_count: int = 0

    def pgm_image(self) -> np.ndarray:
        """Return PGM row order: top row is maximum world Y."""
        return np.flipud(self.occupancy).copy()

    def bounds_m(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x_m,
            self.origin_y_m,
            self.origin_x_m + self.width * self.resolution_m,
            self.origin_y_m + self.height * self.resolution_m,
        )

    def counts(self) -> dict[str, int]:
        return {
            "free": int(np.count_nonzero(self.occupancy == FREE)),
            "occupied": int(np.count_nonzero(self.occupancy == OCCUPIED)),
            "unknown": int(np.count_nonzero(self.occupancy == UNKNOWN)),
        }

    def ground_seed_counts(self) -> dict[str, int]:
        return {
            "candidate": int(self.ground_seed_candidate_count),
            "trusted": int(self.ground_seed_trusted_count),
            "rejected": int(self.ground_seed_rejected_count),
        }


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise AssetContractError(
            "navigation_derivation_scipy_missing",
            "ground-relative navigation-map derivation requires python3-scipy",
        ) from exc
    return ndimage


def _aligned_floor(value: float, resolution: float) -> float:
    return floor(value / resolution) * resolution


def _aligned_ceil(value: float, resolution: float) -> float:
    return ceil(value / resolution) * resolution


def _low_quantile_per_cell(
    cell_ids: np.ndarray,
    z: np.ndarray,
    cell_count: int,
    quantile: float,
) -> tuple[np.ndarray, np.ndarray]:
    counts = np.bincount(cell_ids, minlength=cell_count).astype(np.int32)
    seed = np.full(cell_count, np.nan, dtype=np.float64)
    if cell_ids.size == 0:
        return seed, counts
    order = np.lexsort((z, cell_ids))
    sorted_ids = cell_ids[order]
    unique, starts, group_counts = np.unique(
        sorted_ids, return_index=True, return_counts=True
    )
    ranks = np.floor(float(quantile) * (group_counts - 1)).astype(np.int64)
    selected = order[starts + ranks]
    seed[unique] = z[selected]
    return seed, counts


def _trusted_ground_seeds(
    *,
    seed: np.ndarray,
    point_count: np.ndarray,
    cell_ids: np.ndarray,
    z: np.ndarray,
    cfg: GroundRelativeNavigationConfig,
    ndimage,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reject false ground caused by canopy, overhead frames, or isolated noise.

    A seed must have real point support near its selected height and remain
    consistent with a lower-envelope neighborhood reference. The neighborhood
    reference is intentionally broader than the smoothing kernel so a narrow
    overhead frame cannot become a valid terrain island merely because it is the
    lowest return inside one XY cell.
    """
    seed_flat = seed.reshape(-1)
    seed_at_point = seed_flat[cell_ids]
    near_seed = (
        np.isfinite(seed_at_point)
        & (np.abs(z - seed_at_point) <= cfg.ground_seed_support_band_m)
    )
    support_flat = np.bincount(
        cell_ids[near_seed], minlength=seed_flat.size
    ).astype(np.int32)
    support = support_flat.reshape(seed.shape)
    candidate = (
        np.isfinite(seed)
        & (point_count >= cfg.minimum_cell_points)
        & (support >= cfg.minimum_ground_seed_support_points)
    )
    if not np.any(candidate):
        raise ValueError("no locally supported ground seed candidates were found")

    _, nearest_candidate_indices = ndimage.distance_transform_edt(
        ~candidate, return_indices=True
    )
    candidate_surface = seed[tuple(nearest_candidate_indices)]
    radius_cells = max(1, int(ceil(cfg.ground_continuity_radius_m / cfg.resolution_m)))
    size = 2 * radius_cells + 1
    reference = ndimage.percentile_filter(
        candidate_surface,
        percentile=float(cfg.ground_reference_percentile),
        size=size,
        mode="nearest",
    )

    slope_allowance = tan(radians(float(cfg.maximum_slope_deg))) * float(
        cfg.ground_continuity_radius_m
    )
    maximum_rise = float(cfg.ground_seed_max_rise_m) + slope_allowance
    trusted = (
        candidate
        & (seed <= reference + maximum_rise)
        & (seed >= reference - float(cfg.ground_seed_max_drop_m))
    )
    if not np.any(trusted):
        raise ValueError("all ground seed candidates were rejected by continuity checks")
    return trusted, candidate, support


def _polygon_inside(
    x: np.ndarray, y: np.ndarray, polygon_xy: Sequence[Sequence[float]]
) -> np.ndarray:
    polygon = np.asarray(polygon_xy, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
        raise ValueError("override polygon_xy requires at least three [x, y] vertices")
    inside = np.zeros(x.shape[0], dtype=bool)
    xj, yj = polygon[-1]
    for xi, yi in polygon:
        crossing = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-300) + xi
        )
        inside ^= crossing
        xj, yj = xi, yi
    return inside


def apply_navigation_overrides(
    result: NavigationMapResult,
    overrides: Iterable[Mapping] | None,
) -> np.ndarray:
    occupancy = result.occupancy.copy()
    if not overrides:
        return occupancy
    columns = np.arange(result.width, dtype=np.float64)
    rows = np.arange(result.height, dtype=np.float64)
    xx, yy = np.meshgrid(
        result.origin_x_m + (columns + 0.5) * result.resolution_m,
        result.origin_y_m + (rows + 0.5) * result.resolution_m,
    )
    flat_x = xx.reshape(-1)
    flat_y = yy.reshape(-1)
    for index, override in enumerate(overrides):
        mode = str(override.get("mode", "")).strip().lower()
        if mode not in {"force_free", "force_occupied", "unknown", "no_go"}:
            raise ValueError(f"unsupported navigation override mode at {index}: {mode}")
        mask = _polygon_inside(flat_x, flat_y, override.get("polygon_xy", [])).reshape(
            occupancy.shape
        )
        if mode == "force_free":
            occupancy[mask] = FREE
        elif mode in {"force_occupied", "no_go"}:
            occupancy[mask] = OCCUPIED
        else:
            occupancy[mask] = UNKNOWN
    return occupancy


def derive_ground_relative_navigation_map(
    cloud: PcdCloud,
    config: GroundRelativeNavigationConfig | None = None,
    *,
    rotation_map_from_source: np.ndarray | None = None,
    translation_map_from_source_m: np.ndarray | None = None,
    overrides: Iterable[Mapping] | None = None,
) -> NavigationMapResult:
    """Derive conservative FREE/OCCUPIED/UNKNOWN cells from one static PCD."""
    cfg = config or GroundRelativeNavigationConfig()
    cfg.validate()
    xyz = cloud.xyz()
    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    if xyz.shape[0] < 3:
        raise ValueError("navigation-map derivation requires at least three finite XYZ points")

    if rotation_map_from_source is not None or translation_map_from_source_m is not None:
        rotation = (
            np.eye(3, dtype=np.float64)
            if rotation_map_from_source is None
            else np.asarray(rotation_map_from_source, dtype=np.float64).reshape(3, 3)
        )
        translation = (
            np.zeros(3, dtype=np.float64)
            if translation_map_from_source_m is None
            else np.asarray(translation_map_from_source_m, dtype=np.float64).reshape(3)
        )
        xyz = xyz @ rotation.T + translation

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    resolution = float(cfg.resolution_m)
    minimum_x = _aligned_floor(float(np.min(x)) - cfg.padding_m, resolution)
    minimum_y = _aligned_floor(float(np.min(y)) - cfg.padding_m, resolution)
    maximum_x = _aligned_ceil(float(np.max(x)) + cfg.padding_m, resolution)
    maximum_y = _aligned_ceil(float(np.max(y)) + cfg.padding_m, resolution)
    width = max(1, int(round((maximum_x - minimum_x) / resolution)))
    height = max(1, int(round((maximum_y - minimum_y) / resolution)))

    columns = np.floor((x - minimum_x) / resolution).astype(np.int64)
    rows = np.floor((y - minimum_y) / resolution).astype(np.int64)
    columns = np.clip(columns, 0, width - 1)
    rows = np.clip(rows, 0, height - 1)
    cell_ids = rows * width + columns
    cell_count = width * height

    seed_flat, point_count_flat = _low_quantile_per_cell(
        cell_ids, z, cell_count, cfg.ground_quantile
    )
    seed = seed_flat.reshape(height, width)
    point_count = point_count_flat.reshape(height, width)

    ndimage = _require_scipy()
    trusted_seed, candidate_seed, _ = _trusted_ground_seeds(
        seed=seed,
        point_count=point_count,
        cell_ids=cell_ids,
        z=z,
        cfg=cfg,
        ndimage=ndimage,
    )

    distance_cells, nearest_indices = ndimage.distance_transform_edt(
        ~trusted_seed, return_indices=True
    )
    nearest_ground = seed[tuple(nearest_indices)]
    ground_valid = distance_cells * resolution <= cfg.maximum_ground_fill_distance_m
    radius = int(cfg.ground_smoothing_radius_cells)
    filter_size = 2 * radius + 1
    if filter_size > 1:
        smooth_ground = ndimage.median_filter(
            nearest_ground, size=filter_size, mode="nearest"
        )
    else:
        smooth_ground = nearest_ground.copy()
    ground_height = np.where(ground_valid, smooth_ground, np.nan)

    dz_dy, dz_dx = np.gradient(smooth_ground, resolution, resolution)
    slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))
    if filter_size > 1:
        local_max = ndimage.maximum_filter(
            smooth_ground, size=filter_size, mode="nearest"
        )
        local_min = ndimage.minimum_filter(
            smooth_ground, size=filter_size, mode="nearest"
        )
        step_m = local_max - local_min
    else:
        step_m = np.zeros_like(smooth_ground)
    slope_deg = np.where(ground_valid, slope_deg, np.nan)
    step_m = np.where(ground_valid, step_m, np.nan)

    ground_at_point = ground_height[rows, columns]
    classifiable = np.isfinite(ground_at_point)
    relative_height = np.full(z.shape, np.nan, dtype=np.float64)
    relative_height[classifiable] = z[classifiable] - ground_at_point[classifiable]

    ground_evidence = classifiable & (
        np.abs(relative_height) <= cfg.ground_tolerance_m
    )
    obstacle_evidence = (
        classifiable
        & (relative_height >= cfg.obstacle_min_height_m)
        & (relative_height <= cfg.obstacle_max_height_m)
    )
    ground_support_count = np.bincount(
        cell_ids[ground_evidence], minlength=cell_count
    ).reshape(height, width).astype(np.int32)
    obstacle_count = np.bincount(
        cell_ids[obstacle_evidence], minlength=cell_count
    ).reshape(height, width).astype(np.int32)

    obstacle = obstacle_count >= cfg.minimum_obstacle_points
    geometry_bad = (
        ground_valid
        & (point_count > 0)
        & ((slope_deg > cfg.maximum_slope_deg) | (step_m > cfg.maximum_step_m))
    )
    occupied = obstacle | geometry_bad
    padding_cells = int(ceil(cfg.obstacle_padding_m / resolution))
    if padding_cells > 0:
        occupied = ndimage.maximum_filter(
            occupied.astype(np.uint8),
            size=2 * padding_cells + 1,
            mode="constant",
        ).astype(bool)

    free = (
        ground_valid
        & (ground_support_count >= cfg.minimum_ground_support_points)
        & ~occupied
    )
    occupancy = np.full((height, width), UNKNOWN, dtype=np.uint8)
    occupancy[free] = FREE
    occupancy[occupied] = OCCUPIED

    candidate_count = int(np.count_nonzero(candidate_seed))
    trusted_count = int(np.count_nonzero(trusted_seed))
    base = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=minimum_x,
        origin_y_m=minimum_y,
        width=width,
        height=height,
        ground_height_m=ground_height,
        ground_valid=ground_valid,
        point_count=point_count,
        ground_support_count=ground_support_count,
        obstacle_count=obstacle_count,
        slope_deg=slope_deg,
        step_m=step_m,
        occupancy=occupancy,
        config=cfg,
        ground_seed_candidate_count=candidate_count,
        ground_seed_trusted_count=trusted_count,
        ground_seed_rejected_count=candidate_count - trusted_count,
    )
    if overrides:
        occupancy = apply_navigation_overrides(base, overrides)
        return NavigationMapResult(**{**base.__dict__, "occupancy": occupancy})
    return base


def _write_pgm(path: Path, image: np.ndarray) -> None:
    image = np.asarray(image, dtype=np.uint8)
    if image.ndim != 2:
        raise ValueError("PGM image must be 2D")
    header = (
        f"P5\n# AGT ground-relative navigation map\n"
        f"{image.shape[1]} {image.shape[0]}\n255\n"
    )
    path.write_bytes(header.encode("ascii") + image.tobytes(order="C"))


def _write_nav2_map_files(
    result: NavigationMapResult,
    output_dir: Path,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / "navigation_map.pgm"
    yaml_path = output_dir / "navigation_map.yaml"
    _write_pgm(pgm_path, result.pgm_image())
    nav_yaml = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(result.resolution_m),
        "origin": [float(result.origin_x_m), float(result.origin_y_m), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    yaml_path.write_text(
        yaml.safe_dump(nav_yaml, sort_keys=False), encoding="utf-8"
    )
    return {
        "pgm_path": pgm_path,
        "yaml_path": yaml_path,
        "pgm_sha256": sha256_file(pgm_path),
        "yaml_sha256": sha256_file(yaml_path),
    }

write_navigation_map_files = _write_nav2_map_files


def write_navigation_map_derivation(
    result: NavigationMapResult,
    output_dir: str | Path,
    *,
    source_asset: str | None = None,
    frame_id: str = "map",
    overrides: Iterable[Mapping] | None = None,
) -> Path:
    """Write one immutable reviewable derivation directory."""
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"navigation derivation output already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    written = _write_nav2_map_files(result, output_dir)
    pgm_path = written["pgm_path"]
    yaml_path = written["yaml_path"]
    derivation_path = output_dir / "derivation.yaml"
    np.save(output_dir / "ground_height.npy", result.ground_height_m)
    np.save(output_dir / "slope_deg.npy", result.slope_deg)
    np.save(output_dir / "step_m.npy", result.step_m)
    np.save(output_dir / "obstacle_count.npy", result.obstacle_count)
    np.save(output_dir / "ground_support_count.npy", result.ground_support_count)
    record = {
        "schema": NAVIGATION_DERIVATION_SCHEMA,
        "frame_id": str(frame_id),
        "source_asset": source_asset,
        "config": asdict(result.config),
        "grid": {
            "resolution_m": float(result.resolution_m),
            "origin_xy_m": [float(result.origin_x_m), float(result.origin_y_m)],
            "width": int(result.width),
            "height": int(result.height),
            "bounds_m": [float(v) for v in result.bounds_m()],
        },
        "counts": result.counts(),
        "ground_seeds": result.ground_seed_counts(),
        "overrides": list(overrides or []),
        "outputs": {
            "pgm": pgm_path.name,
            "yaml": yaml_path.name,
            "pgm_sha256": written["pgm_sha256"],
            "yaml_sha256": written["yaml_sha256"],
        },
    }
    derivation_path.write_text(
        yaml.safe_dump(record, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return output_dir


def write_navigation_map_freeze_bundle(
    base_result: NavigationMapResult,
    output_dir: str | Path,
    *,
    source_asset: str | None = None,
    frame_id: str = "map",
    overrides: Iterable[Mapping] | None = None,
) -> Path:
    """Write one immutable pre/post-override navigation-map revision.

    The accepted map is always derived from ``base_result`` using the canonical
    ``apply_navigation_overrides`` rasterization rule. Both products therefore
    share one grid geometry while remaining independently hashable for audit.
    """
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"navigation freeze output already exists: {output_dir}")
    output_dir.mkdir(parents=True)

    ordered_overrides = list(overrides or [])
    accepted_occupancy = apply_navigation_overrides(base_result, ordered_overrides)
    accepted_result = NavigationMapResult(
        **{**base_result.__dict__, "occupancy": accepted_occupancy}
    )

    generated = _write_nav2_map_files(base_result, output_dir / "generated")
    accepted = _write_nav2_map_files(accepted_result, output_dir / "accepted")

    np.save(output_dir / "ground_height.npy", base_result.ground_height_m)
    np.save(output_dir / "slope_deg.npy", base_result.slope_deg)
    np.save(output_dir / "step_m.npy", base_result.step_m)
    np.save(output_dir / "obstacle_count.npy", base_result.obstacle_count)
    np.save(output_dir / "ground_support_count.npy", base_result.ground_support_count)

    record = {
        "schema": NAVIGATION_DERIVATION_SCHEMA,
        "revision_kind": "generated_plus_accepted_override_revision",
        "frame_id": str(frame_id),
        "source_asset": source_asset,
        "config": asdict(base_result.config),
        "grid": {
            "resolution_m": float(base_result.resolution_m),
            "origin_xy_m": [
                float(base_result.origin_x_m),
                float(base_result.origin_y_m),
            ],
            "width": int(base_result.width),
            "height": int(base_result.height),
            "bounds_m": [float(v) for v in base_result.bounds_m()],
        },
        "counts": {
            "generated": base_result.counts(),
            "accepted": accepted_result.counts(),
        },
        "ground_seeds": base_result.ground_seed_counts(),
        "overrides": ordered_overrides,
        "outputs": {
            "generated": {
                "pgm": "generated/navigation_map.pgm",
                "yaml": "generated/navigation_map.yaml",
                "pgm_sha256": generated["pgm_sha256"],
                "yaml_sha256": generated["yaml_sha256"],
            },
            "accepted": {
                "pgm": "accepted/navigation_map.pgm",
                "yaml": "accepted/navigation_map.yaml",
                "pgm_sha256": accepted["pgm_sha256"],
                "yaml_sha256": accepted["yaml_sha256"],
            },
        },
    }
    (output_dir / "derivation.yaml").write_text(
        yaml.safe_dump(record, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output_dir
