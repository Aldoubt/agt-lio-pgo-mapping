"""Research asset V1 validation and review gates, independent of MapStudio."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from jsonschema import Draft202012Validator


REPOSITORY = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPOSITORY / "docs/contracts/research_asset_contract_v1.schema.json"
SCHEMA = json.loads(SCHEMA_PATH.read_text())


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_sha256(value: dict) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def _validate_schema(value: dict, definition: str, label: str) -> None:
    validator = Draft202012Validator({**SCHEMA, "$ref": f"#/$defs/{definition}"})
    errors = sorted(validator.iter_errors(value), key=lambda e: list(map(str, e.path)))
    if errors:
        error = errors[0]
        location = "/".join(map(str, error.path)) or "<root>"
        raise ValueError(f"{label} schema error at {location}: {error.message}")


def _read_json(path: Path) -> dict:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _resolve(base: Path, reference: str) -> Path:
    candidate = Path(reference).expanduser()
    return candidate.resolve() if candidate.is_absolute() else (base / candidate).resolve()


def _validate_se3(matrix: list[list[float]], label: str) -> None:
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError(f"{label}: expected a 4x4 matrix")
    if not all(math.isfinite(float(v)) for row in matrix for v in row):
        raise ValueError(f"{label}: matrix contains a non-finite value")
    if any(abs(float(matrix[3][i]) - expected) > 1e-8 for i, expected in enumerate((0, 0, 0, 1))):
        raise ValueError(f"{label}: invalid homogeneous row")
    rotation = [[float(matrix[i][j]) for j in range(3)] for i in range(3)]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-5:
                raise ValueError(f"{label}: rotation is not orthonormal")
    det = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(det - 1.0) > 1e-5:
        raise ValueError(f"{label}: rotation determinant is not +1")


def _assert_finite_numbers(value: object, label: str) -> None:
    if isinstance(value, bool) or value is None:
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise ValueError(f"{label}: contains a non-finite number")
    elif isinstance(value, dict):
        for key, child in value.items():
            _assert_finite_numbers(child, f"{label}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _assert_finite_numbers(child, f"{label}/{index}")


def validate_bundle(project_path: str | Path, verify_source_files: bool = True) -> dict:
    """Validate the project, component schemas, exact session binding and hashes."""
    project_file = Path(project_path).expanduser().resolve()
    base = project_file.parent
    project = _read_json(project_file)
    _validate_schema(project, "projectManifest", "project")

    dataset_path = _resolve(base, project["dataset_manifest"])
    dataset = _read_json(dataset_path)
    _validate_schema(dataset, "datasetManifest", "dataset manifest")
    dataset_base = dataset_path.parent

    session_manifests = {}
    for row in dataset["sessions"]:
        session_id = row["session_id"]
        if session_id in session_manifests:
            raise ValueError(f"duplicate session_id: {session_id}")
        path = _resolve(dataset_base, row["manifest"])
        value = _read_json(path)
        _validate_schema(value, "sessionManifest", f"session {session_id}")
        if value["session_id"] != session_id:
            raise ValueError(f"session manifest identity mismatch: {session_id} vs {value['session_id']}")
        if verify_source_files:
            pcd = Path(value["map_pcd"]).expanduser()
            if not pcd.is_file():
                raise ValueError(f"missing source PCD for {session_id}: {pcd}")
            if sha256_file(pcd) != value["source_hash"]:
                raise ValueError(f"source PCD hash mismatch for {session_id}")
        session_manifests[session_id] = value

    alignment = _read_json(_resolve(base, project["alignment_contract"]))
    _validate_schema(alignment, "alignmentContract", "alignment contract")
    _validate_se3(alignment["matrix_4x4"], "alignment contract")
    source_id, target_id = alignment["source_session_id"], alignment["target_session_id"]
    if source_id not in session_manifests or target_id not in session_manifests or source_id == target_id:
        raise ValueError("alignment must bind two distinct sessions in this dataset")
    if alignment["source_hash"] != session_manifests[source_id]["source_hash"]:
        raise ValueError("alignment source_hash does not match source session")
    if alignment["target_hash"] != session_manifests[target_id]["source_hash"]:
        raise ValueError("alignment target_hash does not match target session")

    annotation = _read_json(_resolve(base, project["annotation_file"]))
    _validate_schema(annotation, "annotation", "annotation")
    if annotation["dataset_id"] != dataset["dataset_id"]:
        raise ValueError("annotation dataset_id does not match dataset manifest")
    if annotation["reference_frame"] != alignment["reference_frame"]:
        raise ValueError("annotation reference_frame does not match alignment contract")
    if annotation["source_hashes"] != {k: v["source_hash"] for k, v in session_manifests.items()}:
        raise ValueError("annotation source_hashes do not exactly match dataset sessions")
    seen = set()
    for item in annotation["annotations"]:
        if item["annotation_id"] in seen:
            raise ValueError(f"duplicate annotation_id: {item['annotation_id']}")
        seen.add(item["annotation_id"])
        unknown_sessions = set(item["source_session_ids"]) - set(session_manifests)
        if unknown_sessions:
            raise ValueError(f"annotation {item['annotation_id']} has unknown sessions: {sorted(unknown_sessions)}")
        geometry = item["geometry"]
        _assert_finite_numbers(geometry, f"annotation {item['annotation_id']} geometry")
        if geometry["kind"] == "selection_3d":
            if geometry["shape"] == "aabb" and any(
                float(lo) > float(hi)
                for lo, hi in zip(geometry["min_xyz_m"], geometry["max_xyz_m"])
            ):
                raise ValueError(f"annotation {item['annotation_id']} has inverted AABB bounds")
            if geometry["shape"] == "polygon_prism" and geometry["z_range_m"][0] > geometry["z_range_m"][1]:
                raise ValueError(f"annotation {item['annotation_id']} has an inverted Z range")

    analysis_path = _resolve(base, project["analysis_configuration"])
    analysis = _read_json(analysis_path)
    _validate_schema(analysis, "analysisConfiguration", "analysis configuration")
    if analysis["dataset_id"] != dataset["dataset_id"] or analysis["alignment_id"] != alignment["alignment_id"]:
        raise ValueError("analysis configuration is bound to a different dataset or alignment")

    result = _read_json(_resolve(base, project["experiment_result"]))
    _validate_schema(result, "experimentResult", "experiment result")
    if result["dataset_id"] != dataset["dataset_id"] or result["alignment_id"] != alignment["alignment_id"]:
        raise ValueError("experiment result is bound to a different dataset or alignment")
    if result["analysis_configuration_hash"] != sha256_file(analysis_path):
        raise ValueError("experiment result analysis_configuration_hash does not match analysis_configuration.json bytes")

    project_sessions = {project["primary_session_id"], project["comparison_session_id"]}
    if len(project_sessions) != 2 or not project_sessions.issubset(session_manifests):
        raise ValueError("project primary/comparison session IDs must be distinct dataset sessions")
    if project_sessions != {source_id, target_id}:
        raise ValueError("project sessions do not exactly match the alignment source/target pair")

    eligible, reasons = paper_statistics_gate(annotation, alignment, session_manifests, result)
    return {
        "status": "PASS",
        "dataset_id": dataset["dataset_id"],
        "session_ids": sorted(session_manifests),
        "annotation_version": annotation["annotation_version"],
        "annotation_review_status": annotation["review_status"],
        "paper_statistics_eligible": eligible,
        "paper_statistics_blockers": reasons,
    }


def paper_statistics_gate(annotation: dict, alignment: dict, sessions: dict, result: dict) -> tuple[bool, list[str]]:
    reasons = []
    if annotation.get("review_status") != "FROZEN":
        reasons.append("annotation review_status is not FROZEN")
    if any(item.get("review_status") != "FROZEN" for item in annotation.get("annotations", [])):
        reasons.append("one or more annotation features are not FROZEN")
    if alignment.get("status") != "FROZEN" or alignment.get("review_status") != "FROZEN":
        reasons.append("alignment is not FROZEN")
    if result.get("annotation_version") != annotation.get("annotation_version"):
        reasons.append("result was generated from a different annotation_version")
    if result.get("review_status") != "FROZEN":
        reasons.append("experiment result is not FROZEN")
    unknown_stage = [key for key, value in sessions.items() if value.get("growth_stage") == "UNKNOWN"]
    if unknown_stage:
        reasons.append("growth_stage UNKNOWN for session(s): " + ", ".join(sorted(unknown_stage)))
    return not reasons, reasons
