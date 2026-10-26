#!/usr/bin/env python3
"""Create a durable Research Asset V1 project referencing immutable run008."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

import yaml

from contract import sha256_file, validate_bundle


DEFAULT_RUN = Path("/home/yangxuan/ros2_ws/experiments/artifacts/output/cross_stage_greenhouse_20261007_run008")
DEFAULT_OUTPUT = Path("/home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1")
TOMATO_PACKAGE = Path("/home/yangxuan/ros2_ws/experiments/artifacts/output/white_tomato_20261006_collect_080252_mapstudio_20261007/map_package")
GREENHOUSE_PACKAGE = Path("/home/yangxuan/ros2_ws/experiments/fastlivo_lio_green-house_20261006_retry01/map_package")


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=False) + "\n")


def _geometry_from_roi(region: dict) -> dict | None:
    geometry = region.get("geometry")
    if not geometry:
        return None
    coordinates = geometry.get("coordinates_xy_m")
    if not coordinates or not coordinates[0]:
        return None
    ring = [[float(point[0]), float(point[1])] for point in coordinates[0]]
    if len(ring) > 3 and ring[0] == ring[-1]:
        ring.pop()
    return {"kind": "polygon_xy", "coordinates_xy_m": ring}


def migrate(run_dir: Path, output_dir: Path) -> dict:
    run_dir = run_dir.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing research assets: {output_dir}")
    for name in ("input_manifest.json", "alignment_transform.yaml", "experiment.yaml", "greenhouse_roi.yaml"):
        if not (run_dir / name).is_file():
            raise FileNotFoundError(run_dir / name)

    input_manifest = json.loads((run_dir / "input_manifest.json").read_text())
    transform_doc = yaml.safe_load((run_dir / "alignment_transform.yaml").read_text())
    source_run_config = yaml.safe_load((run_dir / "experiment.yaml").read_text())
    roi = yaml.safe_load((run_dir / "greenhouse_roi.yaml").read_text())

    by_role = input_manifest["maps"]
    session_ids = {"sparse": "session_white_tomato_collect_20261006_080252", "reference": "session_green_house"}
    package_paths = {"sparse": TOMATO_PACKAGE, "reference": GREENHOUSE_PACKAGE}
    source_bag_paths = {
        "sparse": Path("/home/yangxuan/rosbags/白云番茄2026-10-6/数据集/collect_20261006_080252"),
        "reference": Path("/home/yangxuan/rosbags/green-house"),
    }
    dataset_id = "dataset_cross_stage_greenhouse_20261007"
    target_frame = "UNVERIFIED/camera_init"
    alignment_id = "alignment_run008_approximate_001"
    source_id, target_id = session_ids["sparse"], session_ids["reference"]
    source_hash = by_role["sparse"]["map_pcd_sha256"]
    target_hash = by_role["reference"]["map_pcd_sha256"]
    matrix = transform_doc["T_map_reference_from_sparse"]
    config_path = run_dir / "experiment.yaml"

    staging_parent = output_dir.parent
    staging_parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.staging-", dir=staging_parent))
    try:
        sessions = []
        for role in ("sparse", "reference"):
            record = by_role[role]
            package_path = package_paths[role].resolve()
            bag_path = source_bag_paths[role].resolve()
            bag_meta = bag_path / "metadata.yaml"
            metadata = yaml.safe_load((package_path / "metadata.yaml").read_text())
            session_id = session_ids[role]
            if role == "reference":
                platform, platform_status = "HANDHELD", "USER_DECLARED"
                platform_evidence = "User explicitly identified green-house as handheld acquisition in the task."
            else:
                platform, platform_status = "VEHICLE", "USER_DECLARED"
                platform_evidence = "User explicitly identified the comparison session as vehicle-mounted LiDAR acquisition."
            if role == "sparse":
                stage, stage_status = "SPARSE", "USER_DECLARED"
                stage_evidence = "User previously described this bag as sparse-period data; stage is not inferred from its filename."
            else:
                stage, stage_status = "UNKNOWN", "UNKNOWN"
                stage_evidence = "Neither the map package nor rosbag metadata identifies this acquisition's growth stage."
            session = {
                "schema_id": "agt.research_session_manifest", "schema_version": 1,
                "session_id": session_id, "map_package": str(package_path),
                "map_pcd": str(package_path / "map.pcd"), "source_hash": record["map_pcd_sha256"],
                "package_manifest_hash": record["manifest_sha256"], "bag_metadata_hash": record["bag_metadata_sha256"],
                "source_bag": str(bag_path), "reference_frame": metadata["frames"]["map"],
                "acquisition_platform": platform, "acquisition_platform_status": platform_status,
                "growth_stage": stage, "growth_stage_status": stage_status,
                "mapping_backend": metadata.get("mapping_backend", {}).get("id", "UNKNOWN"),
                "evidence": [
                    {"field": "acquisition_platform", "source": "user declaration" if platform_status == "USER_DECLARED" else "map/rosbag metadata", "status": platform_status, "note": platform_evidence},
                    {"field": "growth_stage", "source": "user declaration" if stage_status == "USER_DECLARED" else "map/rosbag metadata", "status": stage_status, "note": stage_evidence},
                ],
            }
            rel = f"sessions/{role}/session_manifest.json"
            _write_json(staging / rel, session)
            sessions.append({"session_id": session_id, "manifest": rel})

        alignment = {
            "schema_id": "agt.research_alignment_contract", "schema_version": 1,
            "alignment_id": alignment_id, "source_session_id": source_id, "target_session_id": target_id,
            "source_frame": transform_doc["source_frame"], "target_frame": transform_doc["target_frame"],
            "reference_frame": target_frame, "transform_direction": "T_target_from_source",
            "matrix_4x4": matrix, "source_hash": source_hash, "target_hash": target_hash,
            "status": "APPROXIMATE", "review_status": "DRAFT", "method": transform_doc["source_method"],
            "provenance_path": str(run_dir / "alignment_transform.yaml"),
            "provenance_hash": sha256_file(run_dir / "alignment_transform.yaml"),
            "note": "Reused exactly from run008; no global GICP or transform update was performed.",
        }
        _write_json(staging / "alignment_contract.json", alignment)

        dataset = {
            "schema_id": "agt.research_dataset_manifest", "schema_version": 1,
            "dataset_id": dataset_id, "reference_frame": target_frame, "sessions": sessions,
            "alignment_contract": "alignment_contract.json", "annotation_schema": "agt.research_annotations/v1",
            "evidence_status": "PROVISIONAL",
        }
        _write_json(staging / "dataset_manifest.json", dataset)

        annotations = []
        for source_candidate_id, annotation_type, key in (
            ("greenhouse_boundary_candidate", "greenhouse_boundary", "greenhouse_boundary_candidate"),
            ("navigation_interior", "navigation_interior", "navigation_interior"),
        ):
            geometry = _geometry_from_roi(roi["regions"].get(key, {}))
            if geometry is None:
                continue
            annotations.append({
                "annotation_id": f"{source_candidate_id}-run008-import",
                "annotation_type": annotation_type,
                "geometry": geometry,
                "review_status": "DRAFT",
                "source_session_ids": [source_id, target_id],
                "source_candidate_id": source_candidate_id,
                "candidate_status": "PROVISIONAL",
                "notes": "Imported from run008 candidate geometry. Human review required; this is not final research truth.",
            })
        annotation = {
            "schema_id": "agt.research_annotations", "schema_version": 1,
            "dataset_id": dataset_id, "annotation_version": "research-annotation-v1-draft-001",
            "review_status": "DRAFT", "reference_frame": target_frame,
            "source_hashes": {source_id: source_hash, target_id: target_hash},
            "reviewer": None, "reviewed_at": None,
            "imported_from": {"path": str(run_dir / "greenhouse_roi.yaml"), "sha256": sha256_file(run_dir / "greenhouse_roi.yaml"), "source_version": roi.get("version", "unknown")},
            "annotations": annotations,
        }
        _write_json(staging / "annotations/annotation_v1.json", annotation)

        analysis = {
            "schema_id": "agt.research_analysis_configuration", "schema_version": 1,
            "dataset_id": dataset_id, "annotation_version": annotation["annotation_version"],
            "alignment_id": alignment_id, "source_run_configuration_path": str(config_path),
            "source_run_configuration_sha256": sha256_file(config_path),
            "analysis": {key: value for key, value in source_run_config.items() if key not in ("output_dir", "maps", "existing_alignment_json")},
            "input_hashes": {
                "run008_input_manifest": sha256_file(run_dir / "input_manifest.json"),
                "run008_roi_candidate": sha256_file(run_dir / "greenhouse_roi.yaml"),
                "saved_alignment": sha256_file(run_dir / "alignment_transform.yaml"),
                "sparse_map_pcd": source_hash, "reference_map_pcd": target_hash,
            },
            "source_map_roles": {source_id: "source", target_id: "target"},
        }
        analysis_path = staging / "analysis_configuration.json"
        _write_json(analysis_path, analysis)
        analysis_configuration_hash = sha256_file(analysis_path)

        result_hashes = {str(path.relative_to(run_dir)): sha256_file(path) for path in sorted(run_dir.rglob("*")) if path.is_file()}
        result = {
            "schema_id": "agt.research_experiment_result", "schema_version": 1,
            "result_id": "cross-stage-greenhouse-run008", "dataset_id": dataset_id,
            "annotation_version": "run008-greenhouse_roi.yaml:candidate-001",
            "alignment_id": alignment_id, "analysis_configuration_hash": analysis_configuration_hash,
            "result_path": str(run_dir), "result_hashes": result_hashes,
            "review_status": "DRAFT", "paper_statistics_eligible": False,
            "eligibility_blockers": ["run008 ROI was provisional", "reference growth stage is unknown", "alignment remains approximate and unreviewed"],
        }
        _write_json(staging / "experiment_result_run008.json", result)

        project = {
            "schema_id": "agt.mapstudio_research_project", "schema_version": 1,
            "dataset_manifest": "dataset_manifest.json", "alignment_contract": "alignment_contract.json",
            "annotation_file": "annotations/annotation_v1.json", "analysis_configuration": "analysis_configuration.json",
            "experiment_result": "experiment_result_run008.json",
            "candidate_roi_file": str(run_dir / "greenhouse_roi.yaml"),
            "primary_session_id": target_id, "comparison_session_id": source_id,
        }
        _write_json(staging / "project.json", project)
        (staging / "README.md").write_text(
            "# Cross-stage greenhouse research assets\n\n"
            "Persistent annotation source for MapStudio. All annotations begin in DRAFT. "
            "Source bags, source map packages, and run008 remain external read-only references.\n\n"
            "The green-house platform is HANDHELD by user declaration. Its growth stage is UNKNOWN. "
            "The white-tomato platform is UNKNOWN because map/rosbag files do not identify the mounting; "
            "SPARSE is a prior user declaration, not a filename inference.\n"
        )
        result = validate_bundle(staging / "project.json", verify_source_files=True)
        os.replace(staging, output_dir)
        return result
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run008", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = migrate(args.run008, args.output)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
