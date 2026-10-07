import copy
import hashlib
import json
import shutil
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from contract import SCHEMA, paper_statistics_gate, validate_bundle
from migrate_run008 import migrate


RUN008 = Path("/home/yangxuan/ros2_ws/experiments/artifacts/output/cross_stage_greenhouse_20261007_run008")


@pytest.fixture
def migrated(tmp_path):
    output = tmp_path / "assets"
    summary = migrate(RUN008, output)
    return output, summary


def test_run008_migration_is_draft_and_binds_sessions(migrated):
    root, summary = migrated
    assert summary["status"] == "PASS"
    assert summary["session_ids"] == ["session_green_house", "session_white_tomato_collect_20261006_080252"]
    assert summary["annotation_review_status"] == "DRAFT"
    assert summary["paper_statistics_eligible"] is False
    annotation = json.loads((root / "annotations/annotation_v1.json").read_text())
    assert len(annotation["annotations"]) == 2
    assert all(item["review_status"] == "DRAFT" for item in annotation["annotations"])


def test_platform_and_growth_stage_are_evidence_labeled_not_name_inferred(migrated):
    root, _ = migrated
    data = json.loads((root / "dataset_manifest.json").read_text())
    sessions = {}
    for entry in data["sessions"]:
        sessions[entry["session_id"]] = json.loads((root / entry["manifest"]).read_text())
    handheld = sessions["session_green_house"]
    tomato = sessions["session_white_tomato_collect_20261006_080252"]
    assert handheld["acquisition_platform"] == "HANDHELD"
    assert handheld["acquisition_platform_status"] == "USER_DECLARED"
    assert handheld["growth_stage"] == "UNKNOWN"
    assert tomato["acquisition_platform"] == "VEHICLE"
    assert tomato["acquisition_platform_status"] == "USER_DECLARED"
    assert tomato["evidence"][0]["source"] == "user declaration"
    assert "vehicle-mounted" in tomato["evidence"][0]["note"]
    assert tomato["growth_stage"] == "SPARSE"
    assert tomato["growth_stage_status"] == "USER_DECLARED"


def test_wrong_map_hash_is_rejected(migrated, tmp_path):
    root, _ = migrated
    annotation_path = root / "annotations/annotation_v1.json"
    annotation = json.loads(annotation_path.read_text())
    annotation["source_hashes"]["session_green_house"] = "0" * 64
    annotation_path.write_text(json.dumps(annotation))
    with pytest.raises(ValueError, match="source_hashes"):
        validate_bundle(root / "project.json", verify_source_files=False)


def test_cross_session_annotation_identity_is_rejected(migrated):
    root, _ = migrated
    annotation_path = root / "annotations/annotation_v1.json"
    annotation = json.loads(annotation_path.read_text())
    annotation["annotations"][0]["source_session_ids"] = ["some-other-session"]
    annotation_path.write_text(json.dumps(annotation))
    with pytest.raises(ValueError, match="unknown sessions"):
        validate_bundle(root / "project.json", verify_source_files=False)


def test_direction_and_transform_hashes_are_checked(migrated):
    root, _ = migrated
    alignment_path = root / "alignment_contract.json"
    alignment = json.loads(alignment_path.read_text())
    alignment["transform_direction"] = "T_source_from_target"
    alignment_path.write_text(json.dumps(alignment))
    with pytest.raises(ValueError, match="transform_direction"):
        validate_bundle(root / "project.json", verify_source_files=False)


def test_unreviewed_annotation_cannot_enter_formal_statistics(migrated):
    root, _ = migrated
    annotation = json.loads((root / "annotations/annotation_v1.json").read_text())
    alignment = json.loads((root / "alignment_contract.json").read_text())
    result = json.loads((root / "experiment_result_run008.json").read_text())
    dataset = json.loads((root / "dataset_manifest.json").read_text())
    sessions = {entry["session_id"]: json.loads((root / entry["manifest"]).read_text()) for entry in dataset["sessions"]}
    eligible, reasons = paper_statistics_gate(annotation, alignment, sessions, result)
    assert not eligible
    assert any("not FROZEN" in reason for reason in reasons)
    assert any("different annotation_version" in reason for reason in reasons)


def test_schema_is_gui_independent(migrated):
    root, _ = migrated
    # Contract validation only consumes JSON/schema and source bytes; no Qt/MapStudio process is involved.
    assert validate_bundle(root / "project.json")["status"] == "PASS"


def test_experiment_result_binds_exact_analysis_configuration_bytes(migrated):
    root, _ = migrated
    project = json.loads((root / "project.json").read_text())
    analysis_path = root / project["analysis_configuration"]
    result_path = root / project["experiment_result"]
    result = json.loads(result_path.read_text())
    assert result["analysis_configuration_hash"] == hashlib.sha256(analysis_path.read_bytes()).hexdigest()
    analysis = json.loads(analysis_path.read_text())
    analysis["analysis"]["test_tamper"] = True
    analysis_path.write_text(json.dumps(analysis))
    with pytest.raises(ValueError, match="analysis_configuration.json bytes"):
        validate_bundle(root / "project.json", verify_source_files=False)


def test_3d_selection_schema_requires_geometry_for_selected_shape():
    validator = Draft202012Validator({**SCHEMA, "$ref": "#/$defs/geometry"})
    valid_aabb = {
        "kind": "selection_3d", "shape": "aabb",
        "min_xyz_m": [0, 1, 2], "max_xyz_m": [3, 4, 5],
    }
    assert validator.is_valid(valid_aabb)
    assert not validator.is_valid({"kind": "selection_3d", "shape": "aabb"})
    assert not validator.is_valid({
        "kind": "selection_3d", "shape": "sphere", "center_xyz_m": [0, 0, 0]
    })
