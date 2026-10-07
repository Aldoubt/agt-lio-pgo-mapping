"""P0/P1/P2 reproducible inputs, existing-transform validation and ROI proposal.

This module never estimates or refines a cross-session transform. It validates
the saved transform, reports geometric residual diagnostics, and produces an
editable ROI draft that must be manually reviewed before P3.
"""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Polygon as PlotPolygon
import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull, cKDTree
from scipy import ndimage
import yaml

from .core import finite_bounds, invert_se3, sha256_file, transform_xyz, validate_se3


def _load_config(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("experiment config must be a YAML mapping")
    return value


def _verify_package(package: Path, validator: Path) -> str:
    completed = subprocess.run(
        ["bash", str(validator), str(package)], capture_output=True, text=True,
        check=False,
    )
    output = (completed.stdout + completed.stderr).strip()
    if completed.returncode:
        raise RuntimeError(f"authoritative map validator rejected {package}: {output}")
    return output


def _package_identity(package: Path, validator: Path, declared_stage: str,
                      declared_platform: str) -> dict:
    metadata_path = package / "metadata.yaml"
    manifest_path = package / "manifest.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    map_path = package / "map.pcd"
    bag = metadata.get("source", {}).get("rosbag")
    bag_metadata = Path(bag) / "metadata.yaml" if bag else None
    result = {
        "map_package": str(package),
        "map_pcd": str(map_path),
        "map_pcd_bytes": map_path.stat().st_size,
        "map_pcd_sha256": sha256_file(map_path),
        "manifest_sha256": sha256_file(manifest_path),
        "metadata_sha256": sha256_file(metadata_path),
        "poses_timed_sha256": sha256_file(package / "poses_timed.txt"),
        "map_frame_local_name": metadata.get("frames", {}).get("map", "UNKNOWN"),
        "map_frame_qualified_name": f"{declared_stage}/{metadata.get('frames', {}).get('map', 'UNKNOWN')}",
        "mapping_backend": metadata.get("mapping_backend", {}).get("id", "UNKNOWN"),
        "mapping_mode": metadata.get("mapping_backend", {}).get("mode", "UNKNOWN"),
        "keyframe_count": int(metadata.get("keyframe_count", 0)),
        "map_point_count": int(metadata.get("map_point_count", 0)),
        "source_bag": bag,
        "declared_stage": declared_stage,
        "capture_platform": declared_platform,
        "validator": _verify_package(package, validator),
        "manifest_declared_files": len(manifest.get("files", {})),
    }
    if bag_metadata and bag_metadata.is_file():
        info = yaml.safe_load(bag_metadata.read_text(encoding="utf-8")) or {}
        info = info.get("rosbag2_bagfile_information", {})
        result["bag_metadata_sha256"] = sha256_file(bag_metadata)
        result["bag_duration_s"] = float(info.get("duration", {}).get("nanoseconds", 0)) * 1e-9
        result["bag_message_count"] = int(info.get("message_count", 0))
        result["bag_lidar_message_count"] = sum(
            int(item.get("message_count", 0))
            for item in info.get("topics_with_message_count", [])
            if "lidar" in item.get("topic_metadata", {}).get("name", "").lower()
        )
        result["recorded_topics"] = [
            item.get("topic_metadata", {}).get("name", "UNKNOWN")
            for item in info.get("topics_with_message_count", [])
        ]
    else:
        result["bag_metadata_sha256"] = None
        result["bag_duration_s"] = None
        result["bag_message_count"] = None
        result["bag_lidar_message_count"] = None
        result["recorded_topics"] = []
    return result


def _check_map_hashes(previous: dict, sparse_root: Path,
                      reference_root: Path) -> None:
    expected_sparse = previous["sparse_map"]["file_hashes"]["map.pcd"]["sha256"]
    expected_reference = previous["old_map"]["file_hashes"]["map.pcd"]["sha256"]
    if sha256_file(sparse_root / "map.pcd") != expected_sparse:
        raise ValueError("sparse map PCD hash does not match the saved alignment provenance")
    if sha256_file(reference_root / "map.pcd") != expected_reference:
        raise ValueError("reference map PCD hash does not match the saved alignment provenance")


def _guard_stage_outputs(output: Path, stage: str) -> None:
    alignment_outputs = {
        "input_manifest.json", "alignment_transform.yaml",
        "alignment_validation.json", "alignment_validation.md",
    }
    roi_outputs = {"greenhouse_roi.yaml", "roi_visualization.png", "roi_manual_review.png", "roi_validation.json"}
    p3_outputs = {"metrics.json", "metrics.csv", "stable_structure_residuals.json", "p3_validation.json", "paper_figure_manifest.json"}
    protected = alignment_outputs | roi_outputs | p3_outputs if stage == "all" else (
        alignment_outputs if stage == "alignment" else roi_outputs if stage == "roi" else p3_outputs if stage == "occupancy" else set()
    )
    existing = sorted(name for name in protected if (output / name).exists())
    if existing:
        raise RuntimeError(
            f"refusing to overwrite existing {stage} outputs in {output}: {', '.join(existing)}; use a fresh output directory"
        )


def _read_cloud(path: Path) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError(f"map PCD is empty or unsupported: {path}")
    if not np.isfinite(points).all():
        raise ValueError(f"map PCD contains nonfinite XYZ: {path}")
    return points


def _downsample(points: np.ndarray, leaf_m: float, z_range: tuple[float, float]) -> np.ndarray:
    selected = points[(points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(selected))
    return np.asarray(cloud.voxel_down_sample(leaf_m).points, dtype=np.float64)


def _regional_residuals(source: np.ndarray, target: np.ndarray, tile_m: float,
                        match_gate_m: float) -> dict:
    if not len(source) or not len(target):
        raise ValueError("regional residual needs nonempty clouds")
    distances = cKDTree(target).query(source, k=1, workers=-1)[0]
    tile_index = np.floor(source[:, :2] / tile_m).astype(np.int64)
    grouped: dict[tuple[int, int], list[int]] = defaultdict(list)
    for index, key in enumerate(map(tuple, tile_index)):
        grouped[key].append(index)
    regions = []
    for (ix, iy), members in sorted(grouped.items()):
        values = distances[np.asarray(members, dtype=np.int64)]
        regions.append({
            "tile_xy_index": [int(ix), int(iy)],
            "tile_bounds_xy_m": [ix * tile_m, iy * tile_m, (ix + 1) * tile_m, (iy + 1) * tile_m],
            "sample_count": int(len(values)),
            "median_m": float(np.median(values)),
            "p95_m": float(np.percentile(values, 95)),
            "max_m": float(np.max(values)),
            "fraction_within_gate": float(np.mean(values <= match_gate_m)),
        })
    return {
        "direction": "source_points_to_nearest_target_surface",
        "sample_count": int(len(distances)),
        "tile_size_m": float(tile_m),
        "match_gate_m": float(match_gate_m),
        "global_median_m": float(np.median(distances)),
        "global_p95_m": float(np.percentile(distances, 95)),
        "fraction_within_gate": float(np.mean(distances <= match_gate_m)),
        "regions": regions,
        "semantic_status": "NOT_IDENTIFIED_AS_STABLE_STRUCTURE; spatial tiles are diagnostics only",
    }


def run_alignment(config_path: str | Path) -> dict:
    config_path = Path(config_path).expanduser().resolve()
    cfg = _load_config(config_path)
    output = Path(cfg["output_dir"]).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    maps = cfg["maps"]
    sparse_root = Path(maps["sparse"]["package"]).expanduser().resolve()
    reference_root = Path(maps["reference"]["package"]).expanduser().resolve()
    validator = Path(cfg["map_validator"]).expanduser().resolve()
    source_json = Path(cfg["existing_alignment_json"]).expanduser().resolve()
    previous = json.loads(source_json.read_text(encoding="utf-8"))
    _check_map_hashes(previous, sparse_root, reference_root)
    sparse_stage = str(maps["sparse"].get("stage", "UNVERIFIED"))
    reference_stage = str(maps["reference"].get("stage", "UNVERIFIED"))
    sparse_platform = str(maps["sparse"].get("platform", "UNKNOWN"))
    reference_platform = str(maps["reference"].get("platform", "UNKNOWN"))
    sparse_id = _package_identity(sparse_root, validator, sparse_stage, sparse_platform)
    reference_id = _package_identity(reference_root, validator, reference_stage, reference_platform)
    T_reference_from_sparse = np.asarray(previous["alignment"]["transform_sparse_to_old"], dtype=np.float64)
    se3 = validate_se3(T_reference_from_sparse, float(cfg.get("se3_tolerance", 1e-6)))
    T_sparse_from_reference = invert_se3(T_reference_from_sparse)
    inverse_error = float(np.max(np.abs(T_sparse_from_reference @ T_reference_from_sparse - np.eye(4))))
    z_range = tuple(float(v) for v in cfg["z_range_m"])
    leaf = float(cfg["residual_voxel_m"])
    sparse_points = _read_cloud(sparse_root / "map.pcd")
    reference_points = _read_cloud(reference_root / "map.pcd")
    sparse_down = _downsample(sparse_points, leaf, z_range)
    reference_down = _downsample(reference_points, leaf, z_range)
    sparse_registered = transform_xyz(sparse_down, T_reference_from_sparse)
    forward = _regional_residuals(sparse_registered, reference_down,
                                  float(cfg["residual_tile_m"]), float(cfg["residual_match_gate_m"]))
    reverse = _regional_residuals(reference_down, sparse_registered,
                                  float(cfg["residual_tile_m"]), float(cfg["residual_match_gate_m"]))
    old_compare = previous["coverage_xy"]
    validation = {
        "schema_version": 1,
        "status": "PARTIAL",
        "alignment_class": "Approximate Cross-Session Alignment",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "transform_source": str(source_json),
        "transform_source_sha256": sha256_file(source_json),
        "transform_method_recorded": previous["alignment"]["method"],
        "transform_semantics": {
            "primary_name": "T_map_reference_from_sparse",
            "equation": "p_map_reference = T_map_reference_from_sparse * p_sparse",
            "source_frame": sparse_id["map_frame_qualified_name"],
            "target_frame": reference_id["map_frame_qualified_name"],
            "reverse_name": "T_sparse_from_map_reference",
            "reverse_equation": "p_sparse = T_sparse_from_map_reference * p_map_reference",
            "stage_alias_dense_to_sparse": "UNRESOLVED until reference capture stage is verified",
        },
        "se3_validation": {**se3, "inverse_compose_identity_max_error": inverse_error},
        "transform": {
            "T_map_reference_from_sparse": T_reference_from_sparse.tolist(),
            "T_sparse_from_map_reference": T_sparse_from_reference.tolist(),
            "yaw_deg": float(previous["alignment"]["yaw_deg"]),
            "translation_m": T_reference_from_sparse[:3, 3].tolist(),
        },
        "source_identity": {"sparse": sparse_id, "reference": reference_id},
        "comparison_reused_from_saved_alignment": {
            "voxel_m": previous["alignment"]["input_voxel_m"],
            "icp_fitness_at_1m_gate": previous["alignment"]["icp_fitness"],
            "icp_rmse_m": previous["alignment"]["icp_rmse_m"],
            "new_to_old_fraction_within_0_5m": previous["alignment"]["scores_by_gate_m"]["0.5"]["new_to_old_fraction"],
            "old_to_new_fraction_within_0_5m": previous["alignment"]["scores_by_gate_m"]["0.5"]["old_to_new_fraction"],
            "global_xy_occupancy_iou_previous_exploratory": old_compare["jaccard"],
        },
        "height_distribution_after_saved_alignment": {
            "z_filter_m": list(z_range),
            "sparse_quantiles_1_10_50_90_99_m": previous["sparse_map"]["z_quantiles_filtered_1_10_50_90_99"],
            "reference_quantiles_1_10_50_90_99_m": previous["old_map"]["z_quantiles_filtered_1_10_50_90_99"],
            "interpretation": "descriptive only; hand-held vs vehicle viewpoint and z-datum are not independently calibrated",
        },
        "local_surface_residuals": {
            "method": f"saved transform only; {leaf:.2f}m voxel samples, no re-registration, z filter {z_range}",
            "sparse_to_reference": forward,
            "reference_to_sparse": reverse,
            "stable_structure_residual_status": "NOT_VERIFIED: fixed greenhouse structures have no reviewed labels/ROI",
        },
        "visual_fixed_boundary_check": "PARTIAL: global occupancy overlay exists; no manually identified greenhouse border or fixed-facility landmarks",
        "limitations": [
            "the two local camera_init frame names are not a shared frame; only the saved approximate transform relates them",
            "recording platform and temporal-stage labels are absent from the map-package metadata",
            "bag metadata does not provide usable shared TF for independent frame validation",
            "nearest-surface residual includes crop and external points; it is not a stable-structure accuracy metric",
            "height differences cannot be interpreted as growth or harvest without calibrated viewpoints and a common vertical datum",
            "same-session frontend references are not absolute ground truth",
        ],
    }
    (output / "input_manifest.json").write_text(json.dumps({
        "schema_version": 1,
        "created_at": validation["created_at"],
        "experiment_config": str(config_path),
        "experiment_config_sha256": sha256_file(config_path),
        "alignment_source_json": str(source_json),
        "alignment_source_sha256": validation["transform_source_sha256"],
        "maps": {"sparse": sparse_id, "reference": reference_id},
        "acquisition_platform_status": {
            "sparse": sparse_platform,
            "reference": reference_platform,
            "evidence": "experiment configuration declarations; not independently verified by map-package metadata",
        },
        "stage_assignment_status": {
            "sparse": sparse_stage,
            "reference": reference_stage,
            "evidence": "experiment configuration declarations; reference stage/order requires user confirmation",
        },
    }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    transform_doc = {
        "schema_version": 1,
        "status": "APPROXIMATE_CROSS_SESSION_ALIGNMENT",
        "source_artifact": str(source_json),
        "source_artifact_sha256": validation["transform_source_sha256"],
        "source_method": previous["alignment"]["method"],
        "source_map_pcd_sha256": sparse_id["map_pcd_sha256"],
        "target_map_pcd_sha256": reference_id["map_pcd_sha256"],
        "source_frame": sparse_id["map_frame_qualified_name"],
        "target_frame": reference_id["map_frame_qualified_name"],
        "convention": "column-vector homogeneous transform; target_point = matrix * source_point",
        "T_map_reference_from_sparse": T_reference_from_sparse.tolist(),
        "T_sparse_from_map_reference": T_sparse_from_reference.tolist(),
        "dense_to_sparse_alias": "not assigned; map stages are not confirmed by source metadata",
    }
    (output / "alignment_transform.yaml").write_text(yaml.safe_dump(transform_doc, sort_keys=False, allow_unicode=True), encoding="utf-8")
    (output / "alignment_validation.json").write_text(json.dumps(validation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    md = f"""# Alignment validation — {validation['alignment_class']}\n\n**Status: PARTIAL.** The transform passes the SE(3) checks, but stable-structure and physical boundary residuals are not verified.\n\n- Source transform: `{source_json}`; method: `{validation['transform_method_recorded']}`. No new registration was run.\n- Direction: `p_reference = T_map_reference_from_sparse * p_sparse`; yaw {validation['transform']['yaw_deg']:.3f}°, translation {validation['transform']['translation_m']} m.\n- SE(3): orthogonality error {se3['orthogonality_error_fro']:.3e}, det(R) {se3['rotation_determinant']:.9f}, inverse closure {inverse_error:.3e}.\n- Existing global occupancy IoU: {old_compare['jaccard']:.3f}; this includes external background and is not greenhouse-interior agreement.\n- Local 0.5m-voxel nearest-surface residual diagnostics are in `alignment_validation.json`. Tiles are spatial diagnostics only; no fixed structures were manually identified.\n- Height quantiles are descriptive; acquisition viewpoints, platform labels and common z datum are unverified.\n\nThe cross-session alignment remains approximate. It is not absolute ground truth and must not be reported as high-precision registration.\n"""
    (output / "alignment_validation.md").write_text(md, encoding="utf-8")
    return validation


def _grid_counts(points: np.ndarray, origin_xy: np.ndarray, shape_xy: tuple[int, int],
                 resolution: float, z_range: tuple[float, float]) -> np.ndarray:
    selected = points[(points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])]
    ij = np.floor((selected[:, :2] - origin_xy) / resolution).astype(np.int64)
    nx, ny = shape_xy
    valid = (ij[:, 0] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] >= 0) & (ij[:, 1] < ny)
    keys = ij[valid, 1] * nx + ij[valid, 0]
    return np.bincount(keys, minlength=nx * ny).reshape(ny, nx)


def _candidate_polygon(old_counts: np.ndarray, sparse_counts: np.ndarray,
                       origin_xy: np.ndarray, resolution: float,
                       minimum_points: int, minimum_component_cells: int) -> tuple[list[list[float]], int]:
    mask = (old_counts >= minimum_points) & (sparse_counts >= minimum_points)
    labels, _ = ndimage.label(mask, structure=np.ones((3, 3), dtype=np.uint8))
    sizes = np.bincount(labels.ravel())
    if len(sizes) <= 1:
        raise ValueError("no common high-support component found for ROI proposal")
    sizes[0] = 0
    label = int(np.argmax(sizes))
    if int(sizes[label]) < minimum_component_cells:
        raise ValueError("common high-support component is below configured minimum")
    y, x = np.where(labels == label)
    centers = np.column_stack((origin_xy[0] + (x + .5) * resolution,
                               origin_xy[1] + (y + .5) * resolution))
    hull = ConvexHull(centers)
    polygon = centers[hull.vertices]
    return polygon.tolist(), int(len(x))


def _geometry_payload(geometry) -> dict:
    polygons = [geometry] if geometry.geom_type == "Polygon" else list(geometry.geoms)
    encoded = []
    for part in polygons:
        rings = [part.exterior, *part.interiors]
        encoded.append([[[float(x), float(y)] for x, y in ring.coords] for ring in rings])
    coordinates = encoded[0] if geometry.geom_type == "Polygon" else encoded
    return {"type": geometry.geom_type, "coordinates_xy_m": coordinates}


def _inward_buffer(polygon_geometry, shrink_m: float):
    if not math.isfinite(shrink_m) or shrink_m <= 0:
        raise ValueError("navigation_interior_shrink_m must be finite and greater than zero")
    # Shapely 1.x (ROS Humble system package) accepts GEOS integer enums here.
    result = polygon_geometry.buffer(-shrink_m, join_style=2)
    if result.is_empty or result.geom_type not in {"Polygon", "MultiPolygon"} or result.area <= 0:
        raise ValueError("configured Navigation Interior shrink removes the entire candidate polygon")
    return result


def run_roi(config_path: str | Path) -> dict:
    config_path = Path(config_path).expanduser().resolve()
    cfg = _load_config(config_path)
    output = Path(cfg["output_dir"]).expanduser().resolve()
    previous = json.loads(Path(cfg["existing_alignment_json"]).expanduser().resolve().read_text(encoding="utf-8"))
    maps = cfg["maps"]
    sparse_stage = str(maps["sparse"].get("stage", "UNVERIFIED"))
    reference_stage = str(maps["reference"].get("stage", "UNVERIFIED"))
    sparse_platform = str(maps["sparse"].get("platform", "UNKNOWN"))
    reference_platform = str(maps["reference"].get("platform", "UNKNOWN"))
    sparse_root = Path(cfg["maps"]["sparse"]["package"]).expanduser().resolve()
    reference_root = Path(cfg["maps"]["reference"]["package"]).expanduser().resolve()
    _check_map_hashes(previous, sparse_root, reference_root)
    validator = Path(cfg["map_validator"]).expanduser().resolve()
    sparse_id = _package_identity(sparse_root, validator, sparse_stage, sparse_platform)
    reference_id = _package_identity(reference_root, validator, reference_stage, reference_platform)
    T = np.asarray(previous["alignment"]["transform_sparse_to_old"], dtype=np.float64)
    validate_se3(T)
    sparse = _read_cloud(sparse_root / "map.pcd")
    reference = _read_cloud(reference_root / "map.pcd")
    sparse_registered = transform_xyz(sparse, T)
    resolution = float(cfg["roi_candidate_resolution_m"])
    z_range = tuple(float(v) for v in cfg["z_range_m"])
    included = np.vstack((reference[(reference[:, 2] >= z_range[0]) & (reference[:, 2] <= z_range[1])],
                          sparse_registered[(sparse_registered[:, 2] >= z_range[0]) & (sparse_registered[:, 2] <= z_range[1])]))
    origin = np.floor(included[:, :2].min(axis=0) / resolution) * resolution
    high = np.ceil(included[:, :2].max(axis=0) / resolution) * resolution
    nx, ny = np.rint((high - origin) / resolution).astype(int)
    old_counts = _grid_counts(reference, origin, (int(nx), int(ny)), resolution, z_range)
    sparse_counts = _grid_counts(sparse_registered, origin, (int(nx), int(ny)), resolution, z_range)
    polygon, component_cells = _candidate_polygon(
        old_counts, sparse_counts, origin, resolution,
        int(cfg["roi_min_points_per_cell_each_map"]),
        int(cfg["roi_min_component_cells"]))
    from shapely.geometry import Polygon
    polygon_geometry = Polygon(polygon)
    polygon_valid = bool(polygon_geometry.is_valid and polygon_geometry.area > 0)
    if not polygon_valid:
        raise ValueError("auto-suggested common support polygon is invalid")
    shrink_m = float(cfg["navigation_interior_shrink_m"])
    navigation_geometry = _inward_buffer(polygon_geometry, shrink_m)
    roi = {
        "schema_version": 1,
        "asset_type": "agt.cross_stage_greenhouse_roi_candidate/v1",
        "version": "candidate-001",
        "status": "PROVISIONAL",
        "coordinate_frame": "map_reference/camera_init",
        "coordinate_units": "m",
        "source_maps": {
            "sparse": {"map_package": str(sparse_root), "map_pcd_sha256": previous["sparse_map"]["file_hashes"]["map.pcd"]["sha256"], "stage": sparse_stage, "platform": sparse_platform},
            "reference": {"map_package": str(reference_root), "map_pcd_sha256": previous["old_map"]["file_hashes"]["map.pcd"]["sha256"], "stage": reference_stage, "platform": reference_platform},
        },
        "grid_contract": {"resolution_m": resolution, "origin_xy_m": origin.tolist(), "shape_xy_cells": [int(nx), int(ny)], "z_range_m": list(z_range), "classification": "observed occupied cells only; absence is UNKNOWN, never FREE"},
        "regions": {
            "greenhouse_boundary_candidate": {
                "status": "PROVISIONAL",
                "geometry": _geometry_payload(polygon_geometry),
                "reason": "existing purple support-envelope proposal; not a manually verified greenhouse wall boundary",
            },
            "greenhouse_boundary": {"status": "PROVISIONAL_CANDIDATE_ONLY", "geometry": _geometry_payload(polygon_geometry), "reason": "physical outer wall boundary needs manual confirmation"},
            "greenhouse_interior": {
                "status": "PROVISIONAL_NOT_FINAL",
                "geometry": None,
                "reason": "derive the final research interior after reviewing the greenhouse boundary candidate",
            },
            "navigation_interior": {
                "status": "PROVISIONAL",
                "geometry": _geometry_payload(navigation_geometry),
                "shrink_distance_m": shrink_m,
                "derived_from": "greenhouse_boundary_candidate",
                "reason": "inward-buffered candidate to exclude perimeter structures; not an accepted research ROI",
            },
            "support_envelope_diagnostic": {
                "status": "PROVISIONAL",
                "proposal_method": f"convex hull of largest 8-connected component of {resolution:g}m cells with at least the configured point count in both aligned maps and z-filtered points",
                "proposal_parameters": {"resolution_m": resolution, "minimum_points_per_cell_each_map": int(cfg["roi_min_points_per_cell_each_map"]), "minimum_component_cells": int(cfg["roi_min_component_cells"]), "component_cells": component_cells, "area_m2": float(polygon_geometry.area)},
                "semantic_confidence": "UNKNOWN",
            },
            "harvested_region": {"status": "PROVISIONAL_UNLABELED", "geometry": None, "reason": "awaiting manual polygon; acquisition metadata does not establish this region"},
            "transition_region": {"status": "PROVISIONAL_UNLABELED", "geometry": None, "reason": "awaiting manually selected boundary between cultivation states"},
            "dense_vegetation_region": {"status": "PROVISIONAL_UNLABELED", "geometry": None, "reason": "point counts are observation-weighted and do not establish vegetation density"},
            "external_background": {"status": "PROVISIONAL_UNLABELED", "geometry": None, "reason": "awaiting reviewed exterior polygon or polygons"},
        },
        "stable_structure_review": {
            "status": "PROVISIONAL_UNLABELED",
            "coordinate_frame": "map_reference/camera_init",
            "selections": [
                {"id": f"greenhouse_corner_{index:02d}", "kind": "corner", "geometry_xyz_m": None, "status": "NEEDS_MANUAL_SELECTION"}
                for index in range(1, 5)
            ] + [
                {"id": "greenhouse_frame_edges", "kind": "fixed_frame_edge", "geometry_xyz_m": None, "status": "NEEDS_MANUAL_SELECTION"},
                {"id": "greenhouse_fixed_columns", "kind": "fixed_columns", "geometry_xyz_m": None, "status": "NEEDS_MANUAL_SELECTION"},
            ],
        },
        "manual_review": {"required": True, "confirmed": False, "reviewer": None, "reviewed_at": None, "status": "PROVISIONAL", "notes": "Edit metric-coordinate polygons; fill semantic zones and fixed-structure selections from field knowledge. All derived metrics remain provisional until review."},
    }
    output.mkdir(parents=True, exist_ok=True)
    alignment_source = Path(cfg["existing_alignment_json"]).expanduser().resolve()
    manifest_path = output / "input_manifest.json"
    manifest_content = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "experiment_config": str(config_path),
        "experiment_config_sha256": sha256_file(config_path),
        "alignment_source_json": str(alignment_source),
        "alignment_source_sha256": sha256_file(alignment_source),
        "maps": {"sparse": sparse_id, "reference": reference_id},
        "acquisition_platform_status": {
            "sparse": sparse_platform,
            "reference": reference_platform,
            "evidence": "experiment configuration declarations; not independently verified by map-package metadata",
        },
        "stage_assignment_status": {
            "sparse": sparse_stage,
            "reference": reference_stage,
            "evidence": "experiment configuration declarations; reference stage/order requires user confirmation",
        },
    }
    if manifest_path.exists():
        existing_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if (existing_manifest.get("experiment_config_sha256") != manifest_content["experiment_config_sha256"]
                or existing_manifest.get("maps", {}).get("sparse", {}).get("map_pcd_sha256") != sparse_id["map_pcd_sha256"]
                or existing_manifest.get("maps", {}).get("reference", {}).get("map_pcd_sha256") != reference_id["map_pcd_sha256"]):
            raise RuntimeError("existing input_manifest.json belongs to different inputs or config; use a fresh output directory")
    else:
        manifest_path.write_text(json.dumps(manifest_content, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    roi_path = output / "greenhouse_roi.yaml"
    roi_path.write_text(yaml.safe_dump(roi, sort_keys=False, allow_unicode=True), encoding="utf-8")
    # Draw a reproducible 12:9 meter-coordinate occupancy comparison, not a GUI screenshot.
    shared = (old_counts > 0) & (sparse_counts > 0)
    old_only = (old_counts > 0) & ~(sparse_counts > 0)
    sparse_only = (sparse_counts > 0) & ~(old_counts > 0)
    rgb = np.ones((ny, nx, 3), dtype=np.float32)
    rgb[old_only] = [0.90, 0.48, 0.16]
    rgb[sparse_only] = [0.10, 0.60, 0.77]
    rgb[shared] = [0.62, 0.66, 0.68]
    fig, ax = plt.subplots(figsize=(12, 9), constrained_layout=True)
    extent = [origin[0], origin[0] + nx * resolution, origin[1], origin[1] + ny * resolution]
    ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest", aspect="equal")
    ax.add_patch(PlotPolygon(np.asarray(polygon), closed=True, fill=False, edgecolor="#7b2cbf", linewidth=2.2, linestyle="--"))
    nav_parts = [navigation_geometry] if navigation_geometry.geom_type == "Polygon" else list(navigation_geometry.geoms)
    for part in nav_parts:
        ax.add_patch(PlotPolygon(np.asarray(part.exterior.coords), closed=True, fill=False, edgecolor="#187c70", linewidth=1.8))
    ax.legend(handles=[Patch(facecolor="#9ea8ad", label="Observed occupied in both"),
                       Patch(facecolor="#e67a29", label="Reference only"),
                       Patch(facecolor="#1a99c4", label="Sparse only"),
                       PlotPolygon([[0, 0]], closed=True, fill=False, edgecolor="#7b2cbf", linestyle="--", label="Boundary candidate — PROVISIONAL"),
                       PlotPolygon([[0, 0]], closed=True, fill=False, edgecolor="#187c70", label=f"Navigation Interior — shrink {shrink_m:g} m")],
              loc="upper left", framealpha=.95)
    ax.set_xlabel("x (m) in map_reference/camera_init")
    ax.set_ylabel("y (m) in map_reference/camera_init")
    ax.set_title("Cross-stage observed occupancy and ROI candidate\n0.5m cells; z ∈ [-2, 6]m; manual review required")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=.15)
    image_path = output / "roi_visualization.png"
    fig.savefig(image_path, dpi=300)
    plt.close(fig)
    review_fig = plt.figure(figsize=(12, 9), constrained_layout=True)
    review_grid = review_fig.add_gridspec(1, 2, width_ratios=[3.4, 1.0])
    map_ax = review_fig.add_subplot(review_grid[0, 0])
    info_ax = review_fig.add_subplot(review_grid[0, 1])
    map_ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest", aspect="equal")
    map_ax.add_patch(PlotPolygon(np.asarray(polygon), closed=True, fill=False, edgecolor="#7b2cbf", linewidth=2.3, linestyle="--", label="Greenhouse boundary candidate"))
    for index, part in enumerate(nav_parts):
        map_ax.add_patch(PlotPolygon(np.asarray(part.exterior.coords), closed=True, facecolor="#2a9d8f", alpha=.10, edgecolor="#187c70", linewidth=2.0, label="Navigation Interior" if index == 0 else None))
    map_ax.set_xlabel("x (m) in map_reference/camera_init")
    map_ax.set_ylabel("y (m) in map_reference/camera_init")
    map_ax.set_title("ROI review — both outlines are PROVISIONAL")
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.grid(alpha=.15)
    map_ax.legend(loc="upper left", framealpha=.95)
    info_ax.axis("off")
    info_ax.set_title("Manual annotation slots", loc="left", fontsize=13, fontweight="bold")
    labels = [
        ("Harvested", "PROVISIONAL — polygon: null", "#e76f51"),
        ("Transition", "PROVISIONAL — polygon: null", "#e9c46a"),
        ("Dense Vegetation", "PROVISIONAL — polygon: null", "#457b9d"),
        ("Stable structures", "corners / frame / columns: unselected", "#6c757d"),
    ]
    for i, (name, state, color) in enumerate(labels):
        y = .84 - i * .18
        info_ax.add_patch(plt.Rectangle((.03, y - .025), .08, .06, transform=info_ax.transAxes, color=color, clip_on=False))
        info_ax.text(.15, y, name, transform=info_ax.transAxes, fontsize=11, fontweight="bold", va="center")
        info_ax.text(.15, y - .055, state, transform=info_ax.transAxes, fontsize=9, va="center", wrap=True)
    info_ax.text(.03, .08, f"Navigation shrink: {shrink_m:.2f} m\nMap labels and acquisition stages remain PROVISIONAL.", transform=info_ax.transAxes, fontsize=10, va="bottom")
    review_image_path = output / "roi_manual_review.png"
    review_fig.savefig(review_image_path, dpi=300)
    plt.close(review_fig)
    validation = {
        "schema_version": 1,
        "status": "PROVISIONAL_NEEDS_MANUAL_REVIEW",
        "roi_file": str(roi_path),
        "roi_file_sha256": sha256_file(roi_path),
        "visualization": str(image_path),
        "visualization_size_px": list(plt.imread(image_path).shape[1::-1]),
        "checks": {
            "coordinate_frame_declared": "PASS",
            "metric_units_declared": "PASS",
            "candidate_polygon_finite_and_valid": "PASS",
            "candidate_is_derived_from_map_coordinates_not_screenshot_pixels": "PASS",
            "navigation_interior_buffer_valid": "PASS",
            "navigation_interior_shrink_m_configured": shrink_m,
            "greenhouse_boundary_confirmed": "NOT_VERIFIED",
            "interior_candidate_manually_reviewed": "NOT_VERIFIED",
            "harvested_transition_dense_regions_reviewed": "NOT_VERIFIED",
            "stable_structure_regions_selected": "NOT_VERIFIED",
            "external_background_excluded_from_core_metrics": "PROVISIONAL; semantic exterior boundary remains unreviewed",
            "p3_occupancy_comparison": "PROVISIONAL_PENDING_CONFIGURED_METRICS_STAGE",
        },
        "candidate": {"component_cells": component_cells, "polygon_area_m2": float(polygon_geometry.area), "polygon_xy_m": polygon, "navigation_interior_area_m2": float(navigation_geometry.area), "navigation_interior_shrink_m": shrink_m, "grid_origin_xy_m": origin.tolist(), "grid_shape_xy": [int(nx), int(ny)], "resolution_m": resolution, "z_range_m": list(z_range)},
        "manual_review_image": str(review_image_path),
        "manual_review_image_size_px": list(plt.imread(review_image_path).shape[1::-1]),
    }
    (output / "roi_validation.json").write_text(json.dumps(validation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return validation


def run_stage(config_path: str | Path, stage: str) -> dict:
    if stage not in {"alignment", "roi", "all", "occupancy"}:
        raise ValueError(f"unsupported stage: {stage}")
    cfg = _load_config(Path(config_path).expanduser().resolve())
    _guard_stage_outputs(Path(cfg["output_dir"]).expanduser().resolve(), stage)
    if stage == "alignment":
        return {"alignment": run_alignment(config_path)}
    if stage == "roi":
        return {"roi": run_roi(config_path)}
    if stage == "occupancy":
        from .p3 import run_p3
        return run_p3(config_path)
    if stage == "all":
        alignment = run_alignment(config_path)
        roi = run_roi(config_path)
        p3 = None
        if bool(cfg.get("allow_provisional_roi_metrics", False)):
            from .p3 import run_p3
            p3 = run_p3(config_path)
        return {"alignment": alignment, "roi": roi, "p3": p3,
                "status": "PROVISIONAL_P3_COMPLETE; ROI and stage metadata remain unconfirmed" if p3 else "PARTIAL_NEEDS_MANUAL_REVIEW; P3 was not run"}
    raise AssertionError("unreachable")
