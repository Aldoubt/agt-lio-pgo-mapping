"""Writer invariants plus optional roundtrip through the real pinned V4 owners.

Set AGT_COVERAGE_AUTHORITY_ROOT to a read-only checkout/install of the existing
authority packages. Tests never substitute a local Route validator. Synthetic
Profile snapshots below are test fixtures, not platform calibration evidence.
"""
from __future__ import annotations

from dataclasses import replace
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import box
import yaml

from agt_coverage_authoring import assets
from agt_coverage_authoring.models import (
    CoverageError, GridMap, PathPoint, PlanResult, PlanningContext, Pose, RobotProfile, WorkRow,
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@pytest.fixture
def context(tmp_path):
    profile = tmp_path / "robot.yaml"
    profile.write_text("robot_id: fixture_mkmini\n", encoding="utf-8")
    map_yaml = tmp_path / "map.yaml"
    map_image = tmp_path / "map.pgm"
    Image.fromarray(np.full((80, 100), 254, dtype=np.uint8)).save(map_image)
    map_yaml.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    robot = RobotProfile(
        "fixture_mkmini", digest(profile), .4, 1.0,
        ((.2, .15), (.2, -.15), (-.2, -.15), (-.2, .15)),
        geometry_verified=True, reference_verified=True, limits_verified=True,
        max_forward_speed=.3, max_reverse_speed=.2, max_steering_angle=.5,
        max_steering_rate=.5, max_acceleration=.3, max_deceleration=.6,
    )
    inputs = {str(path): digest(path) for path in (profile, map_yaml, map_image)}
    return PlanningContext(
        GridMap(np.zeros((80, 100), dtype=np.int16), .05, (0., 0., 0.), str(map_yaml), str(map_image)),
        robot, box(0, 0, 5, 4), [WorkRow("row_1", ((1., 1.), (2., 1.)))],
        box(0, 0, 5, 4), [], Pose(1, 1, 0), Pose(2, 1, 0),
        profile_path=str(profile), input_files=inputs,
        map_binding={"map_id": "fixture_site", "map_version_id": "r1", "map_content_sha256": "sha256:" + "a" * 64},
    )


def straight(context):
    points = [PathPoint(1 + index * .1, 1, 0, v_ref=.2, segment_id="row_1",
                        semantic_ref="row_1", kind="SWATH", time_from_start=index * .5)
              for index in range(11)]
    return PlanResult(points, True, {"test_fixture": True}, provenance={"input_files": dict(context.input_files)})


def test_draft_preserves_reverse_and_semantic_annotations(context, tmp_path):
    plan = straight(context)
    plan.points[4].direction = "R"
    plan.points[4].event_ref = "gear_change"
    plan.complete = False
    plan.missing = [{"row_id": "row_2", "reason": "NO_CONNECTION"}]
    result = assets.export_draft(context, plan, tmp_path / "draft")
    with (tmp_path / "draft/route.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[4]["direction"] == "R"
    assert rows[4]["event_ref"] == "gear_change"
    report = json.loads((tmp_path / "draft/report.json").read_text())
    assert report["complete"] is False and report["missing"] == plan.missing
    assert report["point_annotations"][4]["kind"] == "SWATH"
    assert report["output_reference_point"] == "rear_axle_center"
    manifest = yaml.safe_load((tmp_path / "draft/route.yaml").read_text())
    assert "platform_profile_sha256" not in manifest["vehicle_binding"]
    assert result["status"] == "DRAFT" and result["eligible_for_execution"] is False
    assert result["route_ref"]["path_sha256"] == "sha256:" + digest(tmp_path / "draft/route.csv")


def test_existing_revision_is_never_replaced(context, tmp_path):
    destination = tmp_path / "draft"
    assets.export_draft(context, straight(context), destination)
    original = (destination / "route.csv").read_bytes()
    with pytest.raises(CoverageError, match="Refusing to replace"):
        assets.export_draft(context, straight(context), destination)
    assert (destination / "route.csv").read_bytes() == original
    assert not list(tmp_path.glob("draft-stage-*"))


def test_stale_source_and_stale_plan_cannot_export(context, tmp_path):
    plan = straight(context)
    Path(context.profile_path).write_text("changed\n")
    with pytest.raises(CoverageError) as error:
        assets.export_draft(context, plan, tmp_path / "draft")
    assert error.value.code == "STALE_INPUT"
    assert not (tmp_path / "draft").exists()
    context.input_files[context.profile_path] = digest(context.profile_path)
    with pytest.raises(CoverageError) as error:
        assets.export_draft(context, plan, tmp_path / "draft")
    assert error.value.code == "STALE_PLAN"


def test_cancellation_does_not_publish(context, tmp_path):
    context.cancelled = lambda: True
    with pytest.raises(CoverageError) as error:
        assets.export_draft(context, straight(context), tmp_path / "draft")
    assert error.value.code == "CANCELED"
    assert not (tmp_path / "draft").exists()


def test_stale_during_write_removes_staging(context, tmp_path, monkeypatch):
    original = assets._write
    def change_input(path, payload):
        original(path, payload)
        if path.name == "report.json":
            Path(context.profile_path).write_text("changed after planning\n")
    monkeypatch.setattr(assets, "_write", change_input)
    with pytest.raises(CoverageError) as error:
        assets.export_draft(context, straight(context), tmp_path / "draft")
    assert error.value.code == "STALE_INPUT"
    assert not (tmp_path / "draft").exists()
    assert not list(tmp_path.glob("draft-stage-*"))


@pytest.mark.parametrize("mutate, code", [
    (lambda plan, ctx: setattr(plan, "complete", False), "INCOMPLETE_COVERAGE"),
    (lambda plan, ctx: setattr(plan.points[3], "direction", "R"), "REVERSE_UNSUPPORTED"),
    (lambda plan, ctx: setattr(plan.points[3], "v_ref", .1), "VARIABLE_SPEED_UNSUPPORTED"),
    (lambda plan, ctx: setattr(ctx, "robot", replace(ctx.robot, geometry_verified=False)), "PROFILE_UNVERIFIED"),
    (lambda plan, ctx: setattr(ctx, "robot", replace(ctx.robot, rear_axle_to_base=(.3, 0))), "REFERENCE_POINT_UNSUPPORTED"),
])
def test_incompatible_plans_are_rejected_before_owner_import(context, tmp_path, mutate, code):
    plan = straight(context)
    mutate(plan, context)
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, plan, tmp_path / "routes", "coverage", 1,
                             review_confirmed=True, route_reference_point="base_link")
    assert error.value.code == code
    assert not (tmp_path / "routes").exists()


def test_publication_requires_current_review(context, tmp_path):
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, straight(context), tmp_path / "routes", "coverage", 1)
    assert error.value.code == "REVIEW_REQUIRED"


def test_atomic_noreplace_refuses_racing_empty_directory(tmp_path):
    source, destination = tmp_path / "stage", tmp_path / "published"
    source.mkdir()
    (source / "artifact").write_text("new")
    destination.mkdir()
    with pytest.raises(CoverageError) as error:
        assets._rename_noreplace(source, destination)
    assert error.value.code == "REVISION_EXISTS"
    assert destination.is_dir() and not list(destination.iterdir())
    assert (source / "artifact").read_text() == "new"


@pytest.fixture
def authority_context(context, tmp_path, monkeypatch):
    root_value = os.environ.get("AGT_COVERAGE_AUTHORITY_ROOT")
    if not root_value:
        pytest.skip("Existing V4 owner packages not supplied; no mirrored validator is used")
    authority_root = Path(root_value)
    monkeypatch.syspath_prepend(str(authority_root))
    pytest.importorskip("agt_navigation_route_runtime.route_runtime")
    schema = authority_root / "schemas/site_package.schema.json"
    if not schema.is_file():
        pytest.skip("Existing Site schema not supplied alongside owner packages")
    from agt_navigation_map_runtime.models import SiteCandidate, SiteKey
    from agt_navigation_map_runtime.validator import SiteValidator
    from agt_navigation_map_runtime.summary_builder import build_site_summary
    fixture = Path(__file__).resolve().parents[3] / "tests/contract_fixtures/a0/site_valid"
    site = tmp_path / "sites/fixture_site/r1"
    shutil.copytree(fixture, site)
    manifest = yaml.safe_load((site / "manifest.yaml").read_text())
    manifest["site"] = {"id": "fixture_site", "revision": "r1"}
    manifest["compatibility"]["vehicle_profiles"] = ["fixture_compatibility"]
    (site / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    shutil.copyfile(context.grid.image_path, site / "map/navigation.pgm")
    (site / "map/navigation.yaml").write_text("image: navigation.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    hashes = yaml.safe_load((site / "hashes.yaml").read_text())
    for relative in hashes["hashes"]:
        hashes["hashes"][relative] = digest(site / relative)
    (site / "hashes.yaml").write_text(yaml.safe_dump(hashes))
    context.grid.yaml_path = str(site / "map/navigation.yaml")
    context.grid.image_path = str(site / "map/navigation.pgm")
    candidate = SiteCandidate(SiteKey("fixture_site", "r1"), site, site / "manifest.yaml")
    checked = SiteValidator("fixture_compatibility", schema).validate(candidate)
    assert checked.valid, checked.blocker_messages
    summary = build_site_summary(candidate, checked, active=False)
    context.site_root = str(site)
    context.map_binding = {"map_id": summary.map_id, "map_version_id": summary.map_version_id,
                           "map_content_sha256": "sha256:" + summary.map_hash}
    closure = {"format": "agt_robot_profile_content_closure_v1", "profile_id": context.robot.platform_id,
               "artifacts": {"profile_yaml": digest(context.profile_path)}}
    closure_digest = hashlib.sha256(json.dumps(closure, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()
    snapshot = tmp_path / "profile_snapshot.json"
    snapshot.write_text(json.dumps({"snapshot_format": "agt_robot_profile_snapshot_v1",
                                   "robot_profile_id": context.robot.platform_id,
                                   "robot_profile_sha256": closure_digest,
                                   "profile": {"robot_id": context.robot.platform_id},
                                   "digest_manifest": closure}))
    for path in (schema, snapshot, Path(context.grid.yaml_path), Path(context.grid.image_path)):
        context.input_files[str(path.resolve())] = digest(path)
    kwargs = {"review_confirmed": True, "route_reference_point": "base_link", "site_schema": schema,
              "profile_snapshot": snapshot, "profile_digest": closure_digest,
              "profile_snapshot_sha256": digest(snapshot), "site_compatibility_key": "fixture_compatibility"}
    return context, kwargs


def test_ready_roundtrip_through_real_v4_reader(authority_context, tmp_path):
    context, kwargs = authority_context
    plan = straight(context)
    result = assets.publish_ready(context, plan, tmp_path / "companion", "coverage", 1, **kwargs)
    from agt_navigation_route_runtime.route_runtime import load_route_asset, MapBinding, RouteReference
    ref = result["route_ref"]
    loaded = load_route_asset(
        tmp_path / "companion", relative_route_dir="routes/coverage/1",
        expected_route=RouteReference(ref["route_id"], ref["revision"], ref["route_manifest_sha256"], ref["path_sha256"]),
        expected_map=MapBinding("fixture_site", "r1", context.map_binding["map_content_sha256"].removeprefix("sha256:")),
        expected_robot_profile_sha256=kwargs["profile_digest"],
    )
    assert loaded.route_id == "coverage"
    assert len(loaded.segments[0].points) == 11
    assert result["status"] == "READY" and result["eligible_for_execution"] is False
    assert loaded.robot_profile_sha256 != context.robot.digest  # closure != source YAML bytes
    assert not list((tmp_path / "companion").glob("route-stage-*"))


def test_draft_rejected_by_real_ready_reader(authority_context, tmp_path):
    context, kwargs = authority_context
    result = assets.export_draft(context, straight(context), tmp_path / "draft")
    from agt_navigation_route_runtime.route_runtime import load_route_asset, MapBinding, RouteReference, RouteRuntimeError
    ref = result["route_ref"]
    with pytest.raises(RouteRuntimeError) as error:
        load_route_asset(tmp_path, relative_route_dir="draft",
                         expected_route=RouteReference(ref["route_id"], ref["revision"], ref["route_manifest_sha256"], ref["path_sha256"]),
                         expected_map=MapBinding("fixture_site", "r1", context.map_binding["map_content_sha256"].removeprefix("sha256:")),
                         expected_robot_profile_sha256=kwargs["profile_digest"])
    assert error.value.code == "route_not_ready"


def test_site_hash_mismatch_and_profile_closure_mismatch_fail_closed(authority_context, tmp_path):
    context, kwargs = authority_context
    plan = straight(context)
    wrong = dict(kwargs, profile_digest="b" * 64)
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, plan, tmp_path / "companion", "coverage", 1, **wrong)
    assert error.value.code == "STALE_PROFILE"
    image = Path(context.site_root) / "map/localization_map.pcd"
    image.write_bytes(image.read_bytes() + b"\ntampered\n")
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, plan, tmp_path / "companion", "coverage", 1, **kwargs)
    assert error.value.code == "SITE_INVALID"
    assert not (tmp_path / "companion/routes/coverage/1").exists()


def test_reader_failure_leaves_no_published_revision(authority_context, tmp_path, monkeypatch):
    context, kwargs = authority_context
    from agt_navigation_route_runtime import route_runtime
    def refuse(*args, **kwargs):
        raise route_runtime.RouteRuntimeError("fixture_reader_rejection", "deliberate owner rejection")
    monkeypatch.setattr(route_runtime, "load_route_asset", refuse)
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, straight(context), tmp_path / "companion", "coverage", 1, **kwargs)
    assert error.value.code == "fixture_reader_rejection"
    assert not (tmp_path / "companion/routes/coverage/1").exists()
    assert not list((tmp_path / "companion").glob("route-stage-*"))


def test_route_store_cannot_overlap_site(authority_context):
    context, kwargs = authority_context
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, straight(context), context.site_root, "coverage", 1, **kwargs)
    assert error.value.code == "ROUTE_STORE_OVERLAPS_SITE"


def test_nested_output_does_not_mutate_site(authority_context):
    context, kwargs = authority_context
    nested = Path(context.site_root) / "new_drafts"
    with pytest.raises(CoverageError) as error:
        assets.export_draft(context, straight(context), nested)
    assert error.value.code == "ROUTE_STORE_OVERLAPS_SITE"
    assert not nested.exists()
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, straight(context), nested, "coverage", 1, **kwargs)
    assert error.value.code == "ROUTE_STORE_OVERLAPS_SITE"
    assert not nested.exists()


def test_ready_recomputes_coverage_and_clearance(authority_context, tmp_path):
    context, kwargs = authority_context
    plan = straight(context)
    plan.points[0].curvature = 999
    plan.points[0].clearance = 999
    result = assets.publish_ready(context, plan, tmp_path / "companion", "coverage", 1, **kwargs)
    with (Path(result["directory"]) / "route.csv").open(newline="") as stream:
        first = next(csv.DictReader(stream))
    assert float(first["curvature"]) == 0
    assert float(first["clearance"]) < 999
    missing = straight(context)
    missing.points = [replace(point, kind="CONNECTION") for point in missing.points]
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, missing, tmp_path / "companion", "coverage", 2, **kwargs)
    assert error.value.code == "INCOMPLETE_COVERAGE"


def test_ready_rechecks_one_way_row_constraints(context, tmp_path):
    context.rows = [replace(context.rows[0], allowed_directions=("along",))]
    context.start = Pose(2, 1, math.pi)
    context.end = Pose(1, 1, math.pi)
    plan = straight(context)
    plan.points = [replace(point, yaw=math.pi, time_from_start=index * .5)
                   for index, point in enumerate(reversed(plan.points))]
    with pytest.raises(CoverageError) as error:
        assets.publish_ready(context, plan, tmp_path / "companion", "coverage", 1,
                             review_confirmed=True, route_reference_point="base_link")
    assert error.value.code == "ROW_DIRECTION_MISMATCH"
