"""A0 contract fixtures exercised against the existing read-only validators."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).resolve().parent
RUNTIME_ROOT = Path(
    os.environ.get("AGT_NAVIGATION_RUNTIME_ROOT", "/home/yangxuan/agt_navigation_runtime")
).resolve()

# These imports use the current runtime checkout read-only. The fixture tests
# write only under pytest's temporary directory; run with bytecode/cache off.
for package_parent in (
    REPO_ROOT / "artifacts/agt_mapping_artifacts",
    RUNTIME_ROOT / "src/agt_runtime_contracts",
    RUNTIME_ROOT / "src/agt_site_runtime",
    RUNTIME_ROOT / "src/agt_navigation",
):
    sys.path.insert(0, str(package_parent))

from agt_mapping_artifacts.frontend_package import (  # noqa: E402
    verify_frontend_map_package,
    write_frontend_map_package,
)
from agt_mapping_artifacts.map_package_exporter import MapPackageExporter  # noqa: E402
from agt_mapping_artifacts.validation import (  # noqa: E402
    ArtifactValidationError,
    verify_artifact,
)
from agt_navigation.route_runtime import RouteRuntimeError, load_route_asset  # noqa: E402
from agt_runtime_contracts.validator import validate_runtime_contracts  # noqa: E402
from agt_site_runtime.models import SiteCandidate, SiteKey  # noqa: E402
from agt_site_runtime.summary_builder import build_site_summary  # noqa: E402
from agt_site_runtime.validator import SiteValidator  # noqa: E402


SITE_SCHEMA = RUNTIME_ROOT / "schemas/site_package.schema.json"
VEHICLE_SCHEMA = RUNTIME_ROOT / "schemas/vehicle_profile.schema.json"
VEHICLE_PROFILE = RUNTIME_ROOT / "profiles/platforms/mk_mini.yaml"
SITE_HASH = "af55d856bab8fcd0b28973cb21cfbd317cd4ced76bf0d4033051e95ae477a565"
PROFILE_HASH = "sha256:" + "b" * 64  # fixture sentinel, not a robot-profile digest


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _site_validation(site_root: Path):
    manifest_path = site_root / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    candidate = SiteCandidate(
        SiteKey(manifest["site"]["id"], manifest["site"]["revision"]),
        site_root,
        manifest_path,
    )
    validation = SiteValidator(VEHICLE_PROFILE, VEHICLE_SCHEMA, SITE_SCHEMA).validate(candidate)
    return candidate, validation


def _write_pgo_input(root: Path) -> Path:
    source = root / "pgo_input"
    (source / "patches").mkdir(parents=True)
    (source / "map.pcd").write_text(
        "# .PCD v0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )
    (source / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n", encoding="ascii")
    (source / "poses_timed.txt").write_text("0 0 0 0 1 0 0 0\n", encoding="ascii")
    (source / "calibration.yaml").write_text("schema_version: 1\n", encoding="ascii")
    (source / "metadata.yaml").write_text(
        "backend: PGO\nbackend_status:\n  optimized: true\n", encoding="ascii"
    )
    (source / "patches/0.pcd").write_text("fixture patch bytes\n", encoding="ascii")
    return source


def _frontend_records():
    cloud = np.asarray([[1.0, 2.0, 3.0, 10.0], [2.0, 1.0, 0.0, 20.0]], dtype="<f4")
    return [
        {
            "stamp_sec": 10 + index,
            "stamp_nanosec": 0,
            "position": np.asarray([float(index), 0.0, 0.0]),
            "quaternion_xyzw": np.asarray([0.0, 0.0, 0.0, 1.0]),
            "points_xyzi": cloud.copy(),
        }
        for index in range(7)
    ]


def _frontend_provenance():
    return {
        "mapping_backend": {
            "id": "lio_sam_noloop",
            "project": "A0 fixture",
            "mode": "no_loop",
            "source_commit": "a" * 40,
            "config_sha256": "b" * 64,
            "loop_closure": False,
            "gps_factor": False,
            "external_global_correction": False,
        },
        "source": {"rosbag": "fixture-bag", "lidar_topic": "/input/lidar", "imu_topic": "/input/imu"},
        "frames": {"map": "map", "body": "body", "lidar": "lidar", "imu": "imu"},
        "reference": {
            "same_session": True,
            "absolute_ground_truth": False,
            "source": "mapping_frontend_odometry",
            "pgo_applied": False,
            "optimized": False,
        },
    }


def test_scenario_matrix_keeps_unknown_separate_from_operation_status():
    matrix = yaml.safe_load((FIXTURES / "cases.yaml").read_text(encoding="utf-8"))
    cases = {case["id"]: case for case in matrix["cases"]}
    assert {
        "normal_site_route",
        "invalid_route_draft",
        "stale_source_digest",
        "wrong_map_identity",
        "wrong_profile_digest",
        "tampered_site_asset",
        "tampered_route_csv",
        "unsupported_optional_layer",
    } <= set(cases)
    assert cases["unknown_annotation_value"]["expected"] == "UNKNOWN"
    assert cases["unknown_annotation_value"]["is_operation_status"] is False
    assert cases["unsupported_optional_layer"]["facade_status_projection"] == "PROPOSED"


def test_normal_site_and_ready_route_fixture_share_exact_map_reference():
    site_root = FIXTURES / "site_valid"
    candidate, validation = _site_validation(site_root)
    assert validation.valid, validation.blocker_messages
    summary = build_site_summary(candidate, validation, active=False)
    assert summary.map_id == "greenhouse_test"
    assert summary.map_version_id == "r01"
    assert summary.map_hash == SITE_HASH

    matrix = yaml.safe_load((FIXTURES / "cases.yaml").read_text(encoding="utf-8"))
    normal_case = next(case for case in matrix["cases"] if case["id"] == "normal_site_route")
    route_manifest = FIXTURES / "route_ready/route.yaml"
    route_csv = FIXTURES / "route_ready/route.csv"
    assert normal_case["route_manifest_sha256"] == "sha256:" + _sha256(route_manifest)
    assert normal_case["route_csv_sha256"] == "sha256:" + _sha256(route_csv)

    route = load_route_asset(
        FIXTURES / "route_ready",
        expected_map_content_sha256="sha256:" + summary.map_hash,
        expected_vehicle_profile_sha256=PROFILE_HASH,
    )
    assert (route.map_id, route.map_version_id) == (summary.map_id, summary.map_version_id)
    assert route.map_content_sha256 == "sha256:" + summary.map_hash
    assert (route.route_id, route.revision) == ("inspection", 1)


def test_site_validator_rejects_tampered_asset(tmp_path):
    site = tmp_path / "site"
    shutil.copytree(FIXTURES / "site_valid", site)
    with (site / "map/navigation.pgm").open("ab") as stream:
        stream.write(b"tampered\n")
    report = validate_runtime_contracts(VEHICLE_PROFILE, site, VEHICLE_SCHEMA, SITE_SCHEMA)
    assert not report.ok
    assert any(issue.code == "HASH_MISMATCH" for issue in report.issues)


def test_route_loader_rejects_draft_status(tmp_path):
    route_dir = tmp_path / "route"
    shutil.copytree(FIXTURES / "route_ready", route_dir)
    manifest_path = route_dir / "route.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["status"] = "DRAFT"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    with pytest.raises(RouteRuntimeError, match="READY Route") as exc:
        load_route_asset(route_dir)
    assert exc.value.code == "route_not_ready"


def test_route_loader_rejects_stale_map_digest(tmp_path):
    route_dir = tmp_path / "route"
    shutil.copytree(FIXTURES / "route_ready", route_dir)
    with pytest.raises(RouteRuntimeError) as exc:
        load_route_asset(
            route_dir,
            expected_map_content_sha256="sha256:" + "0" * 64,
            expected_vehicle_profile_sha256=PROFILE_HASH,
        )
    assert exc.value.code == "route_map_binding_mismatch"


def test_route_loader_rejects_wrong_profile_digest():
    with pytest.raises(RouteRuntimeError) as exc:
        load_route_asset(
            FIXTURES / "route_ready",
            expected_map_content_sha256="sha256:" + SITE_HASH,
            expected_vehicle_profile_sha256="sha256:" + "c" * 64,
        )
    assert exc.value.code == "route_vehicle_binding_mismatch"


def test_route_loader_rejects_tampered_csv(tmp_path):
    route_dir = tmp_path / "route"
    shutil.copytree(FIXTURES / "route_ready", route_dir)
    with (route_dir / "route.csv").open("ab") as stream:
        stream.write(b"tampered\n")
    with pytest.raises(RouteRuntimeError) as exc:
        load_route_asset(route_dir)
    assert exc.value.code == "route_csv_hash_mismatch"


def test_frontend_source_publisher_and_validator_accept_normal_fixture(tmp_path):
    output = tmp_path / "frontend_source"
    result = write_frontend_map_package(output, _frontend_records(), _frontend_provenance())
    assert result["status"] == "PASS"
    assert verify_frontend_map_package(output) == result
    assert verify_artifact(output) == output


def test_current_pgo_exporter_output_is_rejected_for_manifest_coverage(tmp_path):
    source = _write_pgo_input(tmp_path)
    output = MapPackageExporter().export(source, tmp_path / "published", "greenhouse_test", "a0")
    with pytest.raises(ArtifactValidationError, match="Incomplete checksum coverage") as exc:
        verify_artifact(output)
    assert "manifest.yaml" in str(exc.value)
    index = (output / "checksums.sha256").read_text(encoding="utf-8")
    assert "manifest.yaml" not in index


def test_route_map_id_mutation_is_visible_to_exact_fixture_comparison(tmp_path):
    """Expose the current loader gap: its digest gate alone does not check map IDs."""
    route_dir = tmp_path / "route"
    shutil.copytree(FIXTURES / "route_ready", route_dir)
    manifest_path = route_dir / "route.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["map_binding"]["map_id"] = "other_site"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    route = load_route_asset(
        route_dir,
        expected_map_content_sha256="sha256:" + SITE_HASH,
        expected_vehicle_profile_sha256=PROFILE_HASH,
    )
    assert route.map_id != "greenhouse_test"
