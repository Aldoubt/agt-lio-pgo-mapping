"""Immutable authoring output and adapters to the existing V4 Site/Route owners.

This module is a writer, not a second Site, Robot Profile or Route validator.
READY publication calls the installed V4 validators and exact-byte reader.
The reference implementation inspected for these adapters is
``Aldoubt/agt_navigation_v4@434d6623305ed271a6b1b0978ae1701a41b61fd0``.
No function activates a map, dispatches a task or publishes motion commands.
"""
from __future__ import annotations

import csv
import ctypes
from dataclasses import replace
import errno
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any

import yaml

from .models import CoverageError, PathPoint, PlanResult, PlanningContext


# Existing Route v1 wire columns, deliberately not a new authoring Route schema.
ROUTE_COLUMNS = (
    "seq", "segment_id", "x", "y", "yaw", "direction", "v_ref",
    "curvature", "clearance", "semantic_ref", "event_ref",
)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _file_hash(path: str | Path) -> str:
    try:
        digest = hashlib.sha256()
        with Path(path).open("rb") as stream:
            before = os.fstat(stream.fileno())
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
            after = os.fstat(stream.fileno())
        if (before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
            after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns
        ):
            raise CoverageError("STALE_INPUT", f"Input changed while hashing: {path}")
        return digest.hexdigest()
    except OSError as exc:
        raise CoverageError("STALE_INPUT", f"Cannot read bound input {path}: {exc}") from exc


def assert_inputs_current(context: PlanningContext) -> None:
    """Check the exact source snapshots used to compute a draft, without rehashing assets."""
    context.check_cancelled()
    for filename, expected in context.input_files.items():
        if _file_hash(filename) != str(expected).removeprefix("sha256:"):
            raise CoverageError("STALE_INPUT", f"Input changed since planning: {filename}")


def _assert_plan_binding(context: PlanningContext, result: PlanResult) -> None:
    if result.provenance.get("input_files") != context.input_files:
        raise CoverageError("STALE_PLAN", "Plan input snapshots do not match the currently loaded request")


def _module(name: str):
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise CoverageError(
            "AUTHORITATIVE_VALIDATOR_UNAVAILABLE",
            f"Install/source the existing owner package providing {name}; "
            "READY export is disabled without it.",
        ) from exc


def _owner_call(function, *args, **kwargs):
    try:
        return function(*args, **kwargs)
    except CoverageError:
        raise
    except Exception as exc:
        code = getattr(exc, "code", getattr(exc, "blocker_code", "AUTHORITATIVE_VALIDATION_FAILED"))
        raise CoverageError(str(code), str(exc)) from exc


def _identifier(value: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", value) is None:
        raise CoverageError("INVALID_ROUTE_ID", "Route ID must be a safe, nonempty store component")
    return value


def _revision(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise CoverageError("INVALID_REVISION", "Route revision must be a positive integer")
    return value


def _absolute_directory(path: str | Path, *, create: bool = False) -> Path:
    directory = Path(os.path.abspath(Path(path).expanduser()))
    if directory.resolve() != directory:
        raise CoverageError("UNSAFE_OUTPUT_PATH", "Output directory may not contain symlinks")
    if create:
        directory.mkdir(parents=True, exist_ok=True)
    if not directory.is_dir():
        raise CoverageError("INVALID_OUTPUT_DIRECTORY", f"Directory does not exist: {directory}")
    return directory


def _disjoint(first: Path, second: Path) -> bool:
    return not (first == second or first in second.parents or second in first.parents)


def _rename_noreplace(source: Path, destination: Path) -> None:
    """Publish a complete directory atomically without replacing an existing revision.

    Linux renameat2 provides the required no-replace guarantee. Refusing an
    unsupported filesystem is preferable to a check-then-replace race.
    """
    source_descriptor = os.open(source, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(source_descriptor)
    finally:
        os.close(source_descriptor)
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, "renameat2", None)
    if rename is None:
        raise CoverageError("ATOMIC_PUBLISH_UNSUPPORTED", "renameat2(RENAME_NOREPLACE) is required")
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        error = ctypes.get_errno()
        if error in (errno.EEXIST, errno.ENOTEMPTY):
            raise CoverageError("REVISION_EXISTS", f"Refusing to replace {destination}")
        raise CoverageError("ATOMIC_PUBLISH_FAILED", os.strerror(error))
    descriptor = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write(path: Path, payload: bytes) -> None:
    with path.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _json(payload: Any) -> bytes:
    return (json.dumps(payload, sort_keys=True, ensure_ascii=False, indent=2,
                       allow_nan=False) + "\n").encode("utf-8")


def _route_bytes(points: list[PathPoint], *, offset: tuple[float, float] = (0.0, 0.0)) -> bytes:
    text = io.StringIO(newline="")
    writer = csv.DictWriter(text, fieldnames=ROUTE_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for seq, point in enumerate(points):
        row = {name: getattr(point, name) for name in ROUTE_COLUMNS if name != "seq"}
        if not all(math.isfinite(float(row[key])) for key in (
            "x", "y", "yaw", "v_ref", "curvature", "clearance"
        )):
            raise CoverageError("NON_FINITE_ROUTE", f"Route sample {seq} contains a non-finite value")
        row["seq"] = seq
        cosine, sine = math.cos(point.yaw), math.sin(point.yaw)
        row["x"] = point.x + cosine * offset[0] - sine * offset[1]
        row["y"] = point.y + sine * offset[0] + cosine * offset[1]
        writer.writerow(row)
    return text.getvalue().encode("utf-8")


def _report(context: PlanningContext, result: PlanResult, *, output_reference: str) -> dict[str, Any]:
    return {
        "complete": result.complete,
        "missing": result.missing,
        "report": result.report,
        "provenance": result.provenance,
        "source_inputs": context.input_files,
        "request_path": context.request_path,
        "map_binding": context.map_binding,
        "planning_profile_file_sha256": context.robot.digest,
        "planning_reference_point": "rear_axle_center",
        "output_reference_point": output_reference,
        "rear_axle_to_base": list(context.robot.rear_axle_to_base),
        "curvature_reference_point": "rear_axle_center",
        "eligible_for_execution": False,
        "point_annotations": [
            {"seq": index, "kind": point.kind, "time_from_start": point.time_from_start}
            for index, point in enumerate(result.points)
        ],
    }


def _manifest(context: PlanningContext, csv_bytes: bytes, route_id: str, revision: int,
              *, status: str, map_binding: dict[str, Any], profile_digest: str | None) -> dict[str, Any]:
    vehicle_binding = {"platform_id": context.robot.platform_id}
    if profile_digest is not None:
        vehicle_binding["platform_profile_sha256"] = "sha256:" + profile_digest.removeprefix("sha256:")
    return {
        "schema_version": 1,
        "route_id": route_id,
        "revision": revision,
        "frame_id": "map",
        "map_binding": map_binding,
        "vehicle_binding": vehicle_binding,
        "route_csv_sha256": "sha256:" + _sha256(csv_bytes),
        "status": status,
    }


def _reference(route_id: str, revision: int, manifest_bytes: bytes, csv_bytes: bytes) -> dict[str, Any]:
    return {
        "schema": "agt_route_reference_v1",
        "route_id": route_id,
        "revision": revision,
        "route_manifest_sha256": "sha256:" + _sha256(manifest_bytes),
        "path_sha256": "sha256:" + _sha256(csv_bytes),
    }


def _assert_task_coverage(context: PlanningContext, result: PlanResult) -> None:
    """Recompute authoring completion instead of trusting a serialized report flag."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    from .metrics import coverage_area_metrics, work_polylines
    tolerance = max(1e-6, min(context.sample_step, context.grid.resolution) * .1)
    for point, target in ((result.points[0], context.start), (result.points[-1], context.end)):
        heading = math.atan2(math.sin(point.yaw - target.yaw), math.cos(point.yaw - target.yaw))
        if math.hypot(point.x - target.x, point.y - target.y) > tolerance or abs(heading) > 1e-3:
            raise CoverageError("INCOMPLETE_COVERAGE", "Route does not preserve the requested entrance and exit poses")
    if context.mode == "rows":
        if not context.rows:
            raise CoverageError("INCOMPLETE_COVERAGE", "No requested work rows are bound to this plan")
        for row in context.rows:
            target_line = LineString(row.centerline)
            relevant = [point if point.semantic_ref == row.id else replace(point, kind="CONNECTION")
                        for point in result.points]
            lines = [LineString(polyline) for polyline in work_polylines(relevant)]
            for line in lines:
                positions = [target_line.project(Point(xy)) for xy in line.coords]
                deltas = [last - first for first, last in zip(positions, positions[1:])]
                if (("along" not in row.allowed_directions and any(value > tolerance for value in deltas))
                        or ("against" not in row.allowed_directions and any(value < -tolerance for value in deltas))):
                    raise CoverageError("ROW_DIRECTION_MISMATCH", f"Work segment violates allowed direction for {row.id}")
            swept = unary_union(lines).buffer(tolerance)
            if target_line.difference(swept).length > tolerance:
                raise CoverageError("INCOMPLETE_COVERAGE", f"Requested row {row.id} is not fully covered by work segments")
    elif context.mode == "area":
        if context.operation_width is None:
            raise CoverageError("INCOMPLETE_COVERAGE", "Area task requires a bound operation width")
        area = coverage_area_metrics(context.work_area, work_polylines(result.points), context.operation_width)
        if area["missed_area_m2"] > max(1e-8, area["target_area_m2"] * 1e-8):
            raise CoverageError("INCOMPLETE_COVERAGE", "Requested work area is not fully covered")
    else:
        raise CoverageError("INCOMPLETE_COVERAGE", "Unsupported coverage mode")


def export_draft(context: PlanningContext, result: PlanResult, output_dir: str | Path,
                 *, route_id: str = "coverage_draft", revision: int = 1) -> dict[str, Any]:
    """Save a complete or partial draft, preserving all reverse samples and semantics.

    DRAFT is deliberately rejected by the existing READY reader. Its Profile
    closure binding remains absent; the raw planning YAML digest belongs only
    in the advisory report. publish_ready obtains the authoritative closure.
    """
    route_id, revision = _identifier(route_id), _revision(revision)
    assert_inputs_current(context)
    _assert_plan_binding(context, result)
    target = Path(os.path.abspath(Path(output_dir).expanduser()))
    if context.site_root and not _disjoint(target, Path(context.site_root).resolve()):
        raise CoverageError("ROUTE_STORE_OVERLAPS_SITE", "Draft output must be disjoint from the immutable Site")
    parent = _absolute_directory(target.parent, create=True)
    if target.exists() or target.is_symlink():
        raise CoverageError("REVISION_EXISTS", f"Refusing to replace {target}")
    stage = Path(tempfile.mkdtemp(prefix="draft-stage-", dir=parent))
    try:
        csv_bytes = _route_bytes(result.points)
        manifest = _manifest(context, csv_bytes, route_id, revision, status="DRAFT",
                             map_binding=context.map_binding, profile_digest=None)
        manifest_bytes = yaml.safe_dump(manifest, sort_keys=False).encode("utf-8")
        _write(stage / "route.csv", csv_bytes)
        _write(stage / "route.yaml", manifest_bytes)
        _write(stage / "report.json", _json(_report(context, result, output_reference="rear_axle_center")))
        assert_inputs_current(context)
        _rename_noreplace(stage, target)
        return {"directory": str(target), "status": "DRAFT", "eligible_for_execution": False,
                "route_ref": _reference(route_id, revision, manifest_bytes, csv_bytes)}
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def resolve_site_binding(context: PlanningContext, *, site_schema: str | Path,
                         profile_snapshot: str | Path, profile_digest: str,
                         profile_snapshot_sha256: str, site_compatibility_key: str) -> dict[str, Any]:
    """Use the current V4 owners to bind Site bytes and Profile closure to the input map."""
    if not context.site_root:
        raise CoverageError("SITE_REQUIRED", "READY export requires an immutable Site 1.0 root")
    site_root = _absolute_directory(context.site_root)
    compatibility = _module("agt_navigation_map_runtime.profile_site_compatibility")
    provider = compatibility.ProfileSnapshotProvider(
        Path(profile_snapshot).expanduser().absolute(), context.robot.platform_id,
        profile_digest.removeprefix("sha256:"), profile_snapshot_sha256.removeprefix("sha256:"),
        site_compatibility_key,
    )
    identity = _owner_call(provider.read)
    snapshot_bytes = Path(profile_snapshot).read_bytes()
    if _sha256(snapshot_bytes) != identity.snapshot_sha256:
        raise CoverageError("STALE_PROFILE", "Profile snapshot changed after owner validation")
    snapshot = json.loads(snapshot_bytes)
    expected_yaml = snapshot["digest_manifest"]["artifacts"].get("profile_yaml")
    if not context.profile_path or _file_hash(context.profile_path) != expected_yaml:
        raise CoverageError("PROFILE_GEOMETRY_MISMATCH", "Planning Profile YAML is not the validated snapshot's profile_yaml")
    if context.robot.digest.removeprefix("sha256:") != expected_yaml:
        raise CoverageError("PROFILE_GEOMETRY_MISMATCH", "Selected geometry digest differs from the validated Profile YAML")
    models = _module("agt_navigation_map_runtime.models")
    contracts = _module("agt_navigation_map_runtime.site_contracts")
    validator = _module("agt_navigation_map_runtime.validator")
    summaries = _module("agt_navigation_map_runtime.summary_builder")
    manifest = _owner_call(contracts.load_site_yaml, site_root, "manifest.yaml")
    candidate = models.SiteCandidate(
        models.SiteKey(manifest["site"]["id"], manifest["site"]["revision"]),
        site_root, site_root / "manifest.yaml",
    )
    checked = _owner_call(validator.SiteValidator(identity.site_compatibility_key, Path(site_schema)).validate, candidate)
    if not checked.valid:
        raise CoverageError("SITE_INVALID", "; ".join(checked.blocker_messages))
    summary = _owner_call(summaries.build_site_summary, candidate, checked, active=False)
    if not summary.valid:
        raise CoverageError("SITE_INVALID", "; ".join(summary.validation_errors))
    decision = compatibility.evaluate_compatibility(identity, checked.manifest)
    if decision.status != "COMPATIBLE":
        raise CoverageError(decision.blocker_code, decision.message)
    if (not context.grid.yaml_path or not context.grid.image_path
            or _file_hash(context.grid.yaml_path) != summary.navigation_yaml_sha256
            or _file_hash(context.grid.image_path) != summary.navigation_image_sha256):
        raise CoverageError("MAP_GEOMETRY_MISMATCH", "Planning raster differs from the validated Site navigation map")
    binding = {"map_id": summary.map_id, "map_version_id": summary.map_version_id,
               "map_content_sha256": "sha256:" + summary.map_hash}
    if context.map_binding != binding:
        raise CoverageError("MAP_BINDING_MISMATCH", "Planning Map reference differs from the exact Site summary")
    return {"map_binding": binding, "profile_digest": identity.profile_digest,
            "profile_snapshot_sha256": identity.snapshot_sha256,
            "site_manifest_sha256": summary.manifest_sha256}


def publish_ready(context: PlanningContext, result: PlanResult, route_store: str | Path,
                  route_id: str, revision: int, *, review_confirmed: bool = False,
                  site_schema: str | Path | None = None,
                  profile_snapshot: str | Path | None = None,
                  profile_digest: str | None = None,
                  profile_snapshot_sha256: str | None = None,
                  site_compatibility_key: str | None = None,
                  route_reference_point: str | None = None) -> dict[str, Any]:
    """Publish only a reviewed complete forward Route accepted by the existing reader.

    ``route_reference_point='base_link'`` is an explicit consumer agreement,
    not a guessed reference. The current Route contract does not define how
    steering curvature transforms to an offset body reference, so nonzero
    rear-axle/base offsets remain draft-only until the shared contract supports
    them. No active-map/runtime state is changed.
    """
    route_id, revision = _identifier(route_id), _revision(revision)
    assert_inputs_current(context)
    _assert_plan_binding(context, result)
    if review_confirmed is not True:
        raise CoverageError("REVIEW_REQUIRED", "Review the current complete route before READY publication")
    if not result.complete or result.missing or not result.points:
        raise CoverageError("INCOMPLETE_COVERAGE", "Partial routes cannot be published as a complete READY task")
    if any(point.direction != "F" for point in result.points):
        raise CoverageError("REVERSE_UNSUPPORTED", "The current READY reader accepts only complete all-forward routes")
    _assert_task_coverage(context, result)
    speeds = {point.v_ref for point in result.points}
    if len(speeds) != 1 or any(not math.isfinite(speed) or speed <= 0 for speed in speeds):
        raise CoverageError("VARIABLE_SPEED_UNSUPPORTED", "The current READY reader requires one positive uniform speed")
    robot = context.robot
    if not (robot.geometry_verified and robot.reference_verified and robot.limits_verified):
        raise CoverageError("PROFILE_UNVERIFIED", "Geometry, reference point and limits must all be verified")
    if route_reference_point != "base_link":
        raise CoverageError("REFERENCE_POINT_UNCONFIRMED", "An explicit base_link Route consumer agreement is required")
    if any(abs(value) > 1e-12 for value in robot.rear_axle_to_base):
        raise CoverageError("REFERENCE_POINT_UNSUPPORTED", "Offset base_link Route curvature/reference semantics require a shared consumer contract")
    if not all((site_schema, profile_snapshot, profile_digest, profile_snapshot_sha256, site_compatibility_key)):
        raise CoverageError("PUBLICATION_CONTEXT_REQUIRED", "Site schema and approved Profile snapshot binding are required")
    for filename in (site_schema, profile_snapshot, context.profile_path,
                     context.grid.yaml_path, context.grid.image_path):
        if str(Path(filename).expanduser().resolve()) not in context.input_files:
            raise CoverageError("UNBOUND_PUBLICATION_INPUT", f"Publication input was not snapshotted during planning: {filename}")
    # Both owner imports and owner validation are mandatory, never replaced by
    # a mirrored schema check or a fixture-only publisher.
    authority = _module("agt_navigation_route_runtime.route_runtime")
    from .validation import validate_path
    validation = validate_path(result.points, context)
    if (not validation.get("geometric_valid") or not validation.get("kinematic_valid")
            or validation.get("dynamic_status") != "VALID"):
        raise CoverageError("ROUTE_NOT_VALIDATED", "READY requires geometric, kinematic and dynamic validation")
    binding_args = dict(site_schema=site_schema, profile_snapshot=profile_snapshot,
                        profile_digest=profile_digest, profile_snapshot_sha256=profile_snapshot_sha256,
                        site_compatibility_key=site_compatibility_key)
    bindings = resolve_site_binding(context, **binding_args)
    root_path = Path(os.path.abspath(Path(route_store).expanduser()))
    if not _disjoint(root_path.resolve(), Path(context.site_root).resolve()):
        raise CoverageError("ROUTE_STORE_OVERLAPS_SITE", "Companion Route store must be disjoint from the immutable Site")
    root = _absolute_directory(root_path, create=True)
    destination_parent = _absolute_directory(root / "routes" / route_id, create=True)
    destination = destination_parent / str(revision)
    if destination.exists() or destination.is_symlink():
        raise CoverageError("REVISION_EXISTS", f"Refusing to replace {destination}")
    stage = Path(tempfile.mkdtemp(prefix="route-stage-", dir=root))
    try:
        checked_points = [replace(point, curvature=validation["per_point_curvature"][index],
                                  clearance=validation["per_point_clearance"][index])
                          for index, point in enumerate(result.points)]
        csv_bytes = _route_bytes(checked_points, offset=robot.rear_axle_to_base)
        manifest = _manifest(context, csv_bytes, route_id, revision, status="READY",
                             map_binding=bindings["map_binding"], profile_digest=bindings["profile_digest"])
        manifest_bytes = yaml.safe_dump(manifest, sort_keys=False).encode("utf-8")
        reference = _reference(route_id, revision, manifest_bytes, csv_bytes)
        _write(stage / "route.csv", csv_bytes)
        _write(stage / "route.yaml", manifest_bytes)
        report = _report(context, result, output_reference="base_link")
        report.update(publication_validation=validation, publication_binding=bindings)
        _write(stage / "report.json", _json(report))
        expected_route = authority.RouteReference(
            route_id, revision, reference["route_manifest_sha256"], reference["path_sha256"]
        )
        expected_map = authority.MapBinding(
            bindings["map_binding"]["map_id"], bindings["map_binding"]["map_version_id"],
            bindings["map_binding"]["map_content_sha256"].removeprefix("sha256:"),
        )
        # Read through exactly the same bytes/identity validator as V3/V4.
        _owner_call(authority.load_route_asset,
            root, relative_route_dir=stage.name, expected_route=expected_route,
            expected_map=expected_map, expected_robot_profile_sha256=bindings["profile_digest"],
        )
        assert_inputs_current(context)
        if resolve_site_binding(context, **binding_args) != bindings:
            raise CoverageError("STALE_INPUT", "Site or Profile identity changed during publication")
        _rename_noreplace(stage, destination)
        return {"directory": str(destination), "status": "READY", "route_ref": reference,
                "map_binding": bindings["map_binding"], "eligible_for_execution": False}
    finally:
        if stage.exists():
            shutil.rmtree(stage)
