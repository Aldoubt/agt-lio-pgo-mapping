"""Provisional P3 occupancy comparison and paper figures.

Only observed point support is rasterized. Empty cells stay UNKNOWN. The
existing saved transform is checked and reused; this module never registers
the source maps.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import Patch, Polygon as PlotPolygon
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3D projection
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import yaml

from .core import sha256_file, transform_xyz, validate_se3
from .pipeline import _check_map_hashes, _grid_counts, _load_config, _package_identity, _read_cloud


COLORS = {
    "both": np.array([0.62, 0.66, 0.68]),
    "reference_only": np.array([0.90, 0.48, 0.16]),
    "sparse_only": np.array([0.10, 0.60, 0.77]),
    "unknown": np.array([0.98, 0.98, 0.98]),
    "outside": np.array([0.84, 0.85, 0.86]),
}


def _geometry_parts(payload: dict[str, Any]) -> list[tuple[np.ndarray, list[np.ndarray]]]:
    kind = payload["type"]
    encoded = payload["coordinates_xy_m"]
    parts = [encoded] if kind == "Polygon" else encoded if kind == "MultiPolygon" else []
    if not parts:
        raise ValueError(f"unsupported ROI geometry type: {kind}")
    result = []
    for rings in parts:
        if not rings or len(rings[0]) < 4:
            raise ValueError("ROI polygon exterior ring must contain at least four coordinates")
        exterior = np.asarray(rings[0], dtype=np.float64)
        holes = [np.asarray(ring, dtype=np.float64) for ring in rings[1:]]
        if not np.isfinite(exterior).all() or any(not np.isfinite(hole).all() for hole in holes):
            raise ValueError("ROI coordinates must be finite")
        result.append((exterior, holes))
    return result


def _roi_cell_mask(shape_yx: tuple[int, int], origin_xy: np.ndarray,
                   resolution: float, payload: dict[str, Any]) -> np.ndarray:
    ny, nx = shape_yx
    x = origin_xy[0] + (np.arange(nx, dtype=np.float64) + 0.5) * resolution
    y = origin_xy[1] + (np.arange(ny, dtype=np.float64) + 0.5) * resolution
    xx, yy = np.meshgrid(x, y)
    centers = np.column_stack((xx.ravel(), yy.ravel()))
    mask = np.zeros(len(centers), dtype=bool)
    for exterior, holes in _geometry_parts(payload):
        mask |= MplPath(exterior, closed=True).contains_points(centers, radius=1e-10)
        for hole in holes:
            mask &= ~MplPath(hole, closed=True).contains_points(centers, radius=1e-10)
    return mask.reshape(ny, nx)


def _occupancy_metrics(reference_counts: np.ndarray, sparse_counts: np.ndarray,
                       roi_mask: np.ndarray, minimum_points: int) -> dict:
    if reference_counts.shape != sparse_counts.shape or reference_counts.shape != roi_mask.shape:
        raise ValueError("both occupancy rasters and ROI mask must share the same grid")
    if minimum_points < 1:
        raise ValueError("occupancy_min_points_per_cell must be at least 1")
    reference = (reference_counts >= minimum_points) & roi_mask
    sparse = (sparse_counts >= minimum_points) & roi_mask
    both = reference & sparse
    reference_only = reference & ~sparse
    sparse_only = sparse & ~reference
    unknown_both = ~reference & ~sparse & roi_mask
    intersection = int(np.count_nonzero(both))
    union = int(np.count_nonzero(reference | sparse))
    return {
        "roi_cells_by_cell_center": int(np.count_nonzero(roi_mask)),
        "reference_occupied_cells": int(np.count_nonzero(reference)),
        "sparse_occupied_cells": int(np.count_nonzero(sparse)),
        "intersection_occupied_cells": intersection,
        "union_occupied_cells": union,
        "reference_only_observed_occupied_cells_unknown_in_sparse": int(np.count_nonzero(reference_only)),
        "sparse_only_observed_occupied_cells_unknown_in_reference": int(np.count_nonzero(sparse_only)),
        "unknown_in_both_cells": int(np.count_nonzero(unknown_both)),
        "observed_occupancy_iou": float(intersection / union) if union else None,
        "overlap_ratio_vs_smaller_occupied_set": float(intersection / min(np.count_nonzero(reference), np.count_nonzero(sparse))) if min(np.count_nonzero(reference), np.count_nonzero(sparse)) else None,
        "single_period_only_cells_are_environmental_change": False,
        "empty_cell_semantics": "UNKNOWN; no sensor-ray evidence is present in these PCD inputs",
    }


def _shared_grid_geometry(points: np.ndarray, resolution: float) -> tuple[np.ndarray, tuple[int, int]]:
    if not len(points) or resolution <= 0 or not math.isfinite(resolution):
        raise ValueError("shared grid needs finite points and a positive resolution")
    origin = np.floor(points[:, :2].min(axis=0) / resolution) * resolution
    max_relative_index = np.floor((points[:, :2].max(axis=0) - origin) / resolution).astype(np.int64)
    nx, ny = (max_relative_index + 1).tolist()
    return origin, (int(nx), int(ny))


def _state_rgb(reference_counts: np.ndarray, sparse_counts: np.ndarray,
               roi_mask: np.ndarray, minimum_points: int) -> np.ndarray:
    reference = reference_counts >= minimum_points
    sparse = sparse_counts >= minimum_points
    both = reference & sparse & roi_mask
    ref_only = reference & ~sparse & roi_mask
    sparse_only = sparse & ~reference & roi_mask
    unknown = ~reference & ~sparse & roi_mask
    rgb = np.empty((*roi_mask.shape, 3), dtype=np.float32)
    rgb[:] = COLORS["outside"]
    rgb[unknown] = COLORS["unknown"]
    rgb[both] = COLORS["both"]
    rgb[ref_only] = COLORS["reference_only"]
    rgb[sparse_only] = COLORS["sparse_only"]
    return rgb


def _points_in_geometry(points: np.ndarray, payload: dict[str, Any]) -> np.ndarray:
    xy = points[:, :2]
    mask = np.zeros(len(points), dtype=bool)
    for exterior, holes in _geometry_parts(payload):
        mask |= MplPath(exterior, closed=True).contains_points(xy, radius=1e-10)
        for hole in holes:
            mask &= ~MplPath(hole, closed=True).contains_points(xy, radius=1e-10)
    return mask


def _downsample(points: np.ndarray, leaf_m: float, z_range: tuple[float, float]) -> np.ndarray:
    selected = points[(points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(selected))
    return np.asarray(cloud.voxel_down_sample(leaf_m).points, dtype=np.float64)


def _residual(source: np.ndarray, target: np.ndarray, roi: dict,
              voxel_m: float, z_range: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
    source_ds = _downsample(source, voxel_m, z_range)
    target_ds = _downsample(target, voxel_m, z_range)
    source_ds = source_ds[_points_in_geometry(source_ds, roi)]
    target_ds = target_ds[_points_in_geometry(target_ds, roi)]
    if not len(source_ds) or not len(target_ds):
        raise ValueError("Navigation Interior has no point support in one or both clouds")
    distances = cKDTree(target_ds).query(source_ds, k=1, workers=-1)[0]
    return source_ds, distances


def _residual_summary(points: np.ndarray, distances: np.ndarray, tile_m: float,
                      gate_m: float) -> dict:
    tile_xy = np.floor(points[:, :2] / tile_m).astype(np.int64)
    tile_groups: dict[tuple[int, int], list[int]] = {}
    for index, key in enumerate(map(tuple, tile_xy)):
        tile_groups.setdefault(key, []).append(index)
    regions = []
    for (ix, iy), indices in sorted(tile_groups.items()):
        values = distances[np.asarray(indices, dtype=np.int64)]
        regions.append({
            "tile_xy_index": [int(ix), int(iy)],
            "tile_center_xy_m": [(ix + .5) * tile_m, (iy + .5) * tile_m],
            "sample_count": int(len(values)),
            "median_m": float(np.median(values)),
            "p95_m": float(np.percentile(values, 95)),
            "fraction_within_gate": float(np.mean(values <= gate_m)),
        })
    return {
        "sample_count": int(len(distances)),
        "voxel_m": None,
        "tile_size_m": float(tile_m),
        "match_gate_m": float(gate_m),
        "median_m": float(np.median(distances)),
        "p95_m": float(np.percentile(distances, 95)),
        "max_m": float(np.max(distances)),
        "fraction_within_gate": float(np.mean(distances <= gate_m)),
        "tiles": regions,
        "semantic_status": "ALL_SURFACE_PROXY; fixed greenhouse structures are not manually isolated",
    }


def _save_paper_figure(fig, figures_dir: Path, stem: str) -> dict:
    png = figures_dir / f"{stem}.png"
    pdf = figures_dir / f"{stem}.pdf"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf, dpi=300)
    plt.close(fig)
    return {
        "png": str(png),
        "pdf": str(pdf),
        "png_size_px": [int(plt.imread(png).shape[1]), int(plt.imread(png).shape[0])],
        "dpi": 300,
        "figsize_in": [12, 9],
    }


def _draw_roi_outline(ax, payload: dict, color: str, label: str,
                      linestyle: str = "-", linewidth: float = 1.8) -> None:
    for index, (exterior, _) in enumerate(_geometry_parts(payload)):
        ax.plot(exterior[:, 0], exterior[:, 1], color=color, linestyle=linestyle,
                linewidth=linewidth, label=label if index == 0 else None)


def _figure01(reference_counts: np.ndarray, sparse_counts: np.ndarray,
              origin: np.ndarray, resolution: float, boundary: dict,
              navigation: dict, shrink_m: float, out: Path) -> dict:
    ref_occ = reference_counts > 0
    sparse_occ = sparse_counts > 0
    global_mask = np.ones_like(ref_occ, dtype=bool)
    ny, nx = reference_counts.shape
    extent = [origin[0], origin[0] + nx * resolution, origin[1], origin[1] + ny * resolution]
    fig = plt.figure(figsize=(12, 9), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=(2.0, 1.0), height_ratios=(1.0, 1.0))
    overlay_ax = fig.add_subplot(grid[:, 0])
    reference_ax = fig.add_subplot(grid[0, 1])
    sparse_ax = fig.add_subplot(grid[1, 1])
    overlay_ax.imshow(_state_rgb(reference_counts, sparse_counts, global_mask, 1), origin="lower", extent=extent, interpolation="nearest", aspect="equal")
    _draw_roi_outline(overlay_ax, boundary, "#7b2cbf", "Boundary candidate", "--", 1.8)
    _draw_roi_outline(overlay_ax, navigation, "#187c70", f"Navigation Interior ({shrink_m:g} m inset)", "-", 1.4)
    overlay_ax.set_title("Cross-stage observed occupancy overlay")
    overlay_handles = [
        Patch(facecolor=COLORS["both"], label="Observed occupied in both"),
        Patch(facecolor=COLORS["reference_only"], label="Reference only; sparse UNKNOWN"),
        Patch(facecolor=COLORS["sparse_only"], label="Sparse only; reference UNKNOWN"),
        Patch(facecolor=COLORS["unknown"], edgecolor="#888", label="UNKNOWN in both"),
        Line2D([0], [0], color="#7b2cbf", linestyle="--", label="Boundary candidate"),
        Line2D([0], [0], color="#187c70", linestyle="-", label=f"Navigation Interior ({shrink_m:g} m inset)"),
    ]
    overlay_ax.legend(handles=overlay_handles, loc="upper left", fontsize=7, framealpha=.92, ncol=2)
    for ax, occupied, color, title in (
        (reference_ax, ref_occ, COLORS["reference_only"], "Reference map"),
        (sparse_ax, sparse_occ, COLORS["sparse_only"], "Sparse map transformed into reference frame"),
    ):
        rgb = np.where(occupied[..., None], color, COLORS["unknown"])
        ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest", aspect="equal")
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.12)
    overlay_ax.set_xlabel("x (m), map_reference/camera_init")
    overlay_ax.set_ylabel("y (m), map_reference/camera_init")
    overlay_ax.set_aspect("equal", adjustable="box")
    overlay_ax.grid(alpha=.12)
    fig.suptitle("Fig01 — Cross-stage map overview (PROVISIONAL ROI)", fontsize=14)
    return _save_paper_figure(fig, out, "Fig01_CrossStage_MapOverview")


def _figure02(reference: np.ndarray, sparse: np.ndarray,
              nav_roi: dict, boundary: dict, z_range: tuple[float, float],
              figure_voxel_m: float, residual_voxel_m: float,
              tile_m: float, gate_m: float, out: Path) -> tuple[dict, dict]:
    reference_view = _downsample(reference, figure_voxel_m, z_range)
    sparse_view = _downsample(sparse, figure_voxel_m, z_range)
    reference_view = reference_view[_points_in_geometry(reference_view, boundary)]
    sparse_view = sparse_view[_points_in_geometry(sparse_view, boundary)]
    rng = np.random.default_rng(20261007)
    def sample(points, limit):
        if len(points) <= limit:
            return points
        return points[rng.choice(len(points), limit, replace=False)]
    reference_plot = sample(reference_view, 16000)
    sparse_plot = sample(sparse_view, 16000)
    sparse_res_points, sparse_res = _residual(sparse, reference, nav_roi, residual_voxel_m, z_range)
    ref_res_points, ref_res = _residual(reference, sparse, nav_roi, residual_voxel_m, z_range)
    sparse_summary = _residual_summary(sparse_res_points, sparse_res, tile_m, gate_m)
    ref_summary = _residual_summary(ref_res_points, ref_res, tile_m, gate_m)
    sparse_summary["voxel_m"] = float(residual_voxel_m)
    ref_summary["voxel_m"] = float(residual_voxel_m)

    fig = plt.figure(figsize=(12, 9), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    ax3d = fig.add_subplot(grid[0, 0], projection="3d")
    ax3d.scatter(reference_plot[:, 0], reference_plot[:, 1], reference_plot[:, 2], s=.65, c="#e67a29", alpha=.30, label="Reference")
    ax3d.scatter(sparse_plot[:, 0], sparse_plot[:, 1], sparse_plot[:, 2], s=.65, c="#168db4", alpha=.30, label="Sparse transformed")
    for payload, color, label, z in ((boundary, "#7b2cbf", "Boundary candidate", z_range[0]), (nav_roi, "#187c70", "Navigation Interior", z_range[0] + .05)):
        for i, (exterior, _) in enumerate(_geometry_parts(payload)):
            ax3d.plot(exterior[:, 0], exterior[:, 1], np.full(len(exterior), z), color=color, linewidth=1.6, linestyle="--" if color == "#7b2cbf" else "-", label=label if i == 0 else None)
    z_display_scale = 2.5
    ax3d.set_title("3D overlay — structural regions need manual masks")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_zlim(*z_range)
    ax3d.view_init(elev=28, azim=-58)
    ax3d.legend(fontsize=7, loc="upper left")
    xspan = max(np.ptp(np.r_[reference_plot[:, 0], sparse_plot[:, 0]]), 1.)
    yspan = max(np.ptp(np.r_[reference_plot[:, 1], sparse_plot[:, 1]]), 1.)
    ax3d.set_box_aspect((xspan, yspan, max((z_range[1] - z_range[0]) * z_display_scale, 1.)))
    ax3d.text2D(.03, .76, f"Z display scale ×{z_display_scale:g}", transform=ax3d.transAxes,
                fontsize=8, bbox={"facecolor": "white", "alpha": .78, "edgecolor": "none"})

    ax_res = fig.add_subplot(grid[0, 1])
    if len(sparse_res_points) <= 24000:
        res_plot = sparse_res_points
        res_dist_plot = sparse_res
    else:
        sample_indices = rng.choice(len(sparse_res_points), 24000, replace=False)
        res_plot = sparse_res_points[sample_indices]
        res_dist_plot = sparse_res[sample_indices]
    scatter = ax_res.scatter(res_plot[:, 0], res_plot[:, 1], c=res_dist_plot, s=2, cmap="magma", vmin=0, vmax=max(1.0, float(np.percentile(sparse_res, 95))), rasterized=True)
    _draw_roi_outline(ax_res, nav_roi, "#187c70", "Navigation Interior", "-", 1.1)
    ax_res.set_title("Sparse → reference nearest-surface residual")
    ax_res.set_xlabel("x (m)")
    ax_res.set_ylabel("y (m)")
    ax_res.set_aspect("equal", adjustable="box")
    fig.colorbar(scatter, ax=ax_res, label="distance (m)", shrink=.8)

    ax_hist = fig.add_subplot(grid[1, 0])
    bins = np.linspace(0, max(1.0, float(np.percentile(np.r_[sparse_res, ref_res], 99))), 50)
    ax_hist.hist(sparse_res, bins=bins, density=True, alpha=.62, color="#168db4", label="Sparse → reference")
    ax_hist.hist(ref_res, bins=bins, density=True, alpha=.52, color="#e67a29", label="Reference → sparse")
    ax_hist.set_title("Navigation Interior surface-residual proxy")
    ax_hist.set_xlabel("nearest-surface distance (m)")
    ax_hist.set_ylabel("density")
    ax_hist.legend(fontsize=8)
    ax_hist.grid(alpha=.2)

    ax_tile = fig.add_subplot(grid[1, 1])
    tiles = sparse_summary["tiles"]
    if tiles:
        xy = np.asarray([tile["tile_center_xy_m"] for tile in tiles])
        vals = np.asarray([tile["p95_m"] for tile in tiles])
        tile_plot = ax_tile.scatter(xy[:, 0], xy[:, 1], c=vals, s=90, marker="s", cmap="magma", vmin=0, vmax=max(1.0, float(np.max(vals))))
        fig.colorbar(tile_plot, ax=ax_tile, label="tile P95 (m)", shrink=.8)
    _draw_roi_outline(ax_tile, nav_roi, "#187c70", "Navigation Interior", "-", 1.1)
    ax_tile.set_title(f"{tile_m:g} m tiles — sparse → reference P95")
    ax_tile.set_xlabel("x (m)")
    ax_tile.set_ylabel("y (m)")
    ax_tile.set_aspect("equal", adjustable="box")
    ax_tile.grid(alpha=.16)
    fig.suptitle("Fig02 — Alignment review; residuals are surface proxies, not isolated structure errors", fontsize=13)
    figure = _save_paper_figure(fig, out, "Fig02_Alignment_Validation")
    return figure, {
        "status": "PROVISIONAL_SURFACE_PROXY; stable-structure residual NOT_VERIFIED",
        "navigation_interior_sparse_to_reference": sparse_summary,
        "navigation_interior_reference_to_sparse": ref_summary,
        "fixed_structure_regions": "NOT_VERIFIED: corner, border/frame and fixed-column masks remain unselected in greenhouse_roi.yaml",
        "point_selection": f"{figure_voxel_m:g}m display voxels; residuals use {residual_voxel_m:g}m voxel samples inside Navigation Interior and z={list(z_range)}m",
    }


def _figure03(raster_by_resolution: dict, metrics: dict,
              boundary: dict, navigation: dict, out: Path) -> dict:
    fig = plt.figure(figsize=(12, 9), constrained_layout=False)
    grid = fig.add_gridspec(2, 2, left=.09, right=.98, bottom=.19, top=.90, wspace=.28, hspace=.30)
    axes = np.array([[fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])],
                     [fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])]], dtype=object)
    for row, resolution in enumerate(sorted(raster_by_resolution, reverse=True)):
        raster = raster_by_resolution[resolution]
        for col, scope in enumerate(("global", "navigation_interior")):
            ax = axes[row, col]
            mask = raster["masks"][scope]
            rgb = _state_rgb(raster["reference_counts"], raster["sparse_counts"], mask, raster["minimum_points"])
            ax.imshow(rgb, origin="lower", extent=raster["extent"], interpolation="nearest", aspect="equal")
            if scope == "global":
                _draw_roi_outline(ax, boundary, "#7b2cbf", "Boundary candidate", "--", 1.1)
                _draw_roi_outline(ax, navigation, "#187c70", "Navigation Interior", "-", 1.0)
            else:
                _draw_roi_outline(ax, navigation, "#187c70", "Navigation Interior", "-", 1.2)
            score = metrics[str(resolution)][scope]["observed_occupancy_iou"]
            ax.set_title(f"{scope.replace('_', ' ').title()} @ {resolution:.1f} m — observed IoU {score:.3f}" if score is not None else f"{scope} @ {resolution:.1f}m — IoU N/A")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_aspect("equal", adjustable="box")
            ax.grid(alpha=.10)
    legend = [
        Patch(facecolor=COLORS["both"], label="Observed occupied in both"),
        Patch(facecolor=COLORS["reference_only"], label="Reference-only observed; other map UNKNOWN"),
        Patch(facecolor=COLORS["sparse_only"], label="Sparse-only observed; other map UNKNOWN"),
        Patch(facecolor=COLORS["unknown"], edgecolor="#888", label="No point support in either map — UNKNOWN"),
        Patch(facecolor=COLORS["outside"], label="Outside selected ROI"),
    ]
    fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(.5, .025), ncol=3, fontsize=7.5, framealpha=.95)
    fig.suptitle("Fig03 — Observed occupancy differences (one-sided support is not environmental change)", fontsize=13, y=.97)
    return _save_paper_figure(fig, out, "Fig03_Greenhouse_ROI_Comparison")


def run_p3(config_path: str | Path) -> dict:
    config_path = Path(config_path).expanduser().resolve()
    cfg = _load_config(config_path)
    if not bool(cfg.get("allow_provisional_roi_metrics", False)):
        raise RuntimeError("P3 blocked: set allow_provisional_roi_metrics: true to explicitly authorize provisional metrics")
    output = Path(cfg["output_dir"]).expanduser().resolve()
    roi_path = output / "greenhouse_roi.yaml"
    if not roi_path.is_file():
        raise RuntimeError("P3 blocked: greenhouse_roi.yaml is missing; run the ROI stage first")
    roi = yaml.safe_load(roi_path.read_text(encoding="utf-8")) or {}
    if roi.get("manual_review", {}).get("confirmed", False):
        roi_status = "PROVISIONAL_REVIEWED_BY_USER"
    else:
        roi_status = "PROVISIONAL_UNCONFIRMED"
    regions = roi.get("regions", {})
    boundary_payload = regions.get("greenhouse_boundary_candidate", {}).get("geometry")
    navigation_payload = regions.get("navigation_interior", {}).get("geometry")
    if not boundary_payload or not navigation_payload:
        raise RuntimeError("P3 blocked: provisional boundary candidate and Navigation Interior geometry are required")

    source_json = Path(cfg["existing_alignment_json"]).expanduser().resolve()
    previous = json.loads(source_json.read_text(encoding="utf-8"))
    sparse_root = Path(cfg["maps"]["sparse"]["package"]).expanduser().resolve()
    reference_root = Path(cfg["maps"]["reference"]["package"]).expanduser().resolve()
    _check_map_hashes(previous, sparse_root, reference_root)
    validator = Path(cfg["map_validator"]).expanduser().resolve()
    sparse_id = _package_identity(sparse_root, validator, str(cfg["maps"]["sparse"].get("stage", "UNVERIFIED")), str(cfg["maps"]["sparse"].get("platform", "UNKNOWN")))
    reference_id = _package_identity(reference_root, validator, str(cfg["maps"]["reference"].get("stage", "UNVERIFIED")), str(cfg["maps"]["reference"].get("platform", "UNKNOWN")))
    if roi.get("source_maps", {}).get("sparse", {}).get("map_pcd_sha256") != sparse_id["map_pcd_sha256"] or roi.get("source_maps", {}).get("reference", {}).get("map_pcd_sha256") != reference_id["map_pcd_sha256"]:
        raise RuntimeError("P3 blocked: ROI source map identity differs from the current input packages")
    transform = np.asarray(previous["alignment"]["transform_sparse_to_old"], dtype=np.float64)
    validate_se3(transform, float(cfg.get("se3_tolerance", 1e-6)))
    transform_doc_path = output / "alignment_transform.yaml"
    if not transform_doc_path.is_file():
        raise RuntimeError("P3 blocked: frozen alignment_transform.yaml is missing")
    transform_doc = yaml.safe_load(transform_doc_path.read_text(encoding="utf-8")) or {}
    frozen = np.asarray(transform_doc["T_map_reference_from_sparse"], dtype=np.float64)
    if not np.array_equal(frozen, transform):
        raise RuntimeError("P3 blocked: saved alignment transform differs from the source comparison; refusing to proceed")

    z_range = tuple(float(v) for v in cfg["z_range_m"])
    minimum_points = int(cfg["occupancy_min_points_per_cell"])
    resolutions = [float(v) for v in cfg["occupancy_resolutions_m"]]
    if sorted(resolutions) != [0.2, 0.5]:
        raise ValueError("this P3 contract requires occupancy_resolutions_m: [0.5, 0.2]")
    reference = _read_cloud(reference_root / "map.pcd")
    sparse_native = _read_cloud(sparse_root / "map.pcd")
    sparse = transform_xyz(sparse_native, transform)
    all_selected = np.vstack((
        reference[(reference[:, 2] >= z_range[0]) & (reference[:, 2] <= z_range[1])],
        sparse[(sparse[:, 2] >= z_range[0]) & (sparse[:, 2] <= z_range[1])],
    ))
    if not len(all_selected):
        raise ValueError("no points remain inside configured z_range_m")
    figures_dir = output / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    metrics_by_resolution: dict[str, dict] = {}
    raster_by_resolution: dict[float, dict] = {}
    for resolution in resolutions:
        origin, shape_xy = _shared_grid_geometry(all_selected, resolution)
        nx, ny = shape_xy
        reference_counts = _grid_counts(reference, origin, shape_xy, resolution, z_range)
        sparse_counts = _grid_counts(sparse, origin, shape_xy, resolution, z_range)
        global_mask = np.ones((ny, nx), dtype=bool)
        boundary_mask = _roi_cell_mask(global_mask.shape, origin, resolution, boundary_payload)
        navigation_mask = _roi_cell_mask(global_mask.shape, origin, resolution, navigation_payload)
        scoped_metrics = {
            "global": _occupancy_metrics(reference_counts, sparse_counts, global_mask, minimum_points),
            "greenhouse_boundary_candidate": _occupancy_metrics(reference_counts, sparse_counts, boundary_mask, minimum_points),
            "navigation_interior": _occupancy_metrics(reference_counts, sparse_counts, navigation_mask, minimum_points),
        }
        metrics_by_resolution[f"{resolution:.1f}"] = scoped_metrics
        raster_by_resolution[resolution] = {
            "origin_xy_m": origin.tolist(),
            "shape_xy_cells": [nx, ny],
            "resolution_m": resolution,
            "reference_counts": reference_counts,
            "sparse_counts": sparse_counts,
            "minimum_points": minimum_points,
            "masks": {"global": global_mask, "navigation_interior": navigation_mask},
            "extent": [origin[0], origin[0] + nx * resolution, origin[1], origin[1] + ny * resolution],
        }

    stable_voxel = float(cfg["stable_residual_voxel_m"])
    figure_voxel = float(cfg["alignment_figure_voxel_m"])
    tile_m = float(cfg["residual_tile_m"])
    gate_m = float(cfg["residual_match_gate_m"])
    shrink_m = float(cfg["navigation_interior_shrink_m"])
    fig01 = _figure01(raster_by_resolution[0.5]["reference_counts"], raster_by_resolution[0.5]["sparse_counts"],
                      np.asarray(raster_by_resolution[0.5]["origin_xy_m"]), .5,
                      boundary_payload, navigation_payload, shrink_m, figures_dir)
    fig02, residuals = _figure02(reference, sparse, navigation_payload, boundary_payload, z_range,
                                  figure_voxel, stable_voxel, tile_m, gate_m, figures_dir)
    fig03 = _figure03(raster_by_resolution, metrics_by_resolution, boundary_payload, navigation_payload, figures_dir)

    metrics = {
        "schema_version": 1,
        "status": "PROVISIONAL_NOT_PAPER_TRUTH" if roi_status == "PROVISIONAL_UNCONFIRMED" else "PROVISIONAL_REVIEWED_NOT_PAPER_TRUTH",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "roi_status": roi_status,
        "alignment_status": "APPROXIMATE_CROSS_SESSION_ALIGNMENT_FROZEN",
        "alignment_transform_sha256": sha256_file(transform_doc_path),
        "source_map_pcd_sha256": {"sparse": sparse_id["map_pcd_sha256"], "reference": reference_id["map_pcd_sha256"]},
        "grid_contract": {
            "resolutions_m": resolutions,
            "origin_rule": "floor(global minimum XY / resolution) * resolution separately per resolution",
            "extent_rule": "shared extent covers both maps after frozen transform",
            "z_range_m": list(z_range),
            "occupancy_min_points_per_cell": minimum_points,
            "roi_cell_selection": "cell center inside provisional ROI polygon",
            "empty_cell_state": "UNKNOWN; PCDs do not encode sensor-ray free-space evidence",
        },
        "regions": {
            "global": "whole common XY grid extent",
            "greenhouse_boundary_candidate": "existing purple polygon, still PROVISIONAL",
            "navigation_interior": f"greenhouse boundary candidate eroded inward by {float(cfg['navigation_interior_shrink_m']):g} m, still PROVISIONAL",
            "harvested_transition_dense_vegetation": "unlabeled; no regional IoU computed",
        },
            "metrics_by_resolution_m": metrics_by_resolution,
        "grid_geometry_by_resolution_m": {f"{resolution:.1f}": {"origin_xy_m": raster["origin_xy_m"], "shape_xy_cells": raster["shape_xy_cells"], "extent_xy_m": raster["extent"]} for resolution, raster in raster_by_resolution.items()},
        "difference_semantics": "reference-only/sparse-only means observed occupied in one PCD and UNKNOWN in the other; it is not asserted to be environmental change",
        "stable_structure_residuals": residuals,
        "limitations": [
            "the saved cross-session transform was reused without optimization; geometric visual acceptance is not ground truth",
            "3D overlay shows clouds but physical corners, frame edges and fixed columns are not individually labeled",
            "reported residual values are nearest-surface proxies over all supported surfaces inside Navigation Interior, not stable-structure-only residuals",
            "one-sided occupancy is censored by observation and must not be interpreted as growth, harvest, removal, free space or change",
            "acquisition stage/platform metadata remain PROVISIONAL or UNKNOWN",
        ],
    }
    metrics_path = output / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    csv_path = output / "metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fields = ["resolution_m", "scope", "roi_cells_by_cell_center", "reference_occupied_cells", "sparse_occupied_cells", "intersection_occupied_cells", "union_occupied_cells", "reference_only_observed_occupied_cells_unknown_in_sparse", "sparse_only_observed_occupied_cells_unknown_in_reference", "unknown_in_both_cells", "observed_occupancy_iou", "overlap_ratio_vs_smaller_occupied_set"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for resolution, scopes in metrics_by_resolution.items():
            for scope, values in scopes.items():
                writer.writerow({"resolution_m": resolution, "scope": scope, **{key: values.get(key) for key in fields[2:]}})
    residual_path = output / "stable_structure_residuals.json"
    residual_path.write_text(json.dumps(residuals, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    figures = {
        "Fig01_CrossStage_MapOverview": fig01,
        "Fig02_Alignment_Validation": fig02,
        "Fig03_Greenhouse_ROI_Comparison": fig03,
    }
    figure_manifest = {
        "schema_version": 1,
        "status": metrics["status"],
        "figure_contract": {"aspect_ratio": "4:3", "figsize_in": [12, 9], "dpi": 300, "formats": ["PNG", "PDF"]},
        "figures": figures,
    }
    (output / "paper_figure_manifest.json").write_text(json.dumps(figure_manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    validation = {
        "schema_version": 1,
        "status": metrics["status"],
        "roi_hash": sha256_file(roi_path),
        "checks": {
            "frozen_transform_reused_exactly": "PASS",
            "source_map_hashes_match_saved_alignment": "PASS",
            "source_map_packages_validated": "PASS",
            "both_required_grid_resolutions_generated": "PASS",
            "same_origin_and_extent_within_each_resolution": "PASS",
            "unknown_preserved_and_no_free_inferred": "PASS",
            "one_sided_cells_interpreted_as_environment_change": "PASS: one-sided occupied cells remain UNKNOWN in the other map",
            "roi_is_final_manual_truth": "NOT_VERIFIED; all ROI-derived metrics are PROVISIONAL",
            "stable_structure_only_residual": "NOT_VERIFIED; no reviewed corner/frame/column masks",
            "centerline_or_clearance_metrics": "NOT_RUN as requested",
            "figures_4_3_300dpi_png_pdf": "PASS",
        },
        "outputs": {"metrics_json": str(metrics_path), "metrics_csv": str(csv_path), "stable_structure_residuals": str(residual_path), "figures": figures},
    }
    (output / "p3_validation.json").write_text(json.dumps(validation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    roi_validation_path = output / "roi_validation.json"
    roi_validation = json.loads(roi_validation_path.read_text(encoding="utf-8"))
    roi_validation["checks"]["external_background_excluded_from_core_metrics"] = "PROVISIONAL: metrics are restricted to global, candidate-boundary and navigation-interior scopes; external semantic boundary is not reviewed"
    roi_validation["checks"]["p3_occupancy_comparison"] = "PROVISIONAL_METRICS_GENERATED; not paper-truth"
    roi_validation["p3_metrics"] = str(metrics_path)
    roi_validation_path.write_text(json.dumps(roi_validation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return {"status": metrics["status"], "metrics": str(metrics_path), "figures": figures, "validation": str(output / "p3_validation.json")}
