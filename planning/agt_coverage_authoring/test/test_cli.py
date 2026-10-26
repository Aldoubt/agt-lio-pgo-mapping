"""Headless application roundtrips with real geometry/reference planning."""
from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal

import numpy as np
import pytest
import yaml

from agt_coverage_authoring import cli
from agt_coverage_authoring.models import CoverageError


@pytest.fixture
def job(tmp_path):
    """A fully traversable single aisle; explicit exit needs no turn or mock planner."""
    source = Path(__file__).resolve().parents[1] / "examples"
    directory = tmp_path / "inputs"
    shutil.copytree(source, directory)
    structure_path = directory / "structure.geojson"
    structure = json.loads(structure_path.read_text())
    structure["features"] = [structure["features"][0], structure["features"][1]]
    structure["features"][1]["properties"].pop("corridor", None)
    structure["features"].append({
        "type": "Feature",
        "properties": {"id": "selected_work_area", "feature_type": "work_area", "review_status": "CONFIRMED"},
        "geometry": {"type": "Polygon", "coordinates": [[[3,2.6],[11,2.6],[11,3.4],[3,3.4],[3,2.6]]]},
    })
    structure_path.write_text(json.dumps(structure))
    request = yaml.safe_load((directory / "rows.yaml").read_text())
    request.update(start=[3.,3.,0.], end=[11.,3.,0.], heading_rad=0.)
    path = directory / "request.yaml"
    path.write_text(yaml.safe_dump(request))
    return path


def invoke(operation, request, output, *extra):
    return cli.main([operation, "--request", str(request), "--output", str(output), *map(str, extra)])


def read_json(path):
    return json.loads(Path(path).read_text())


def set_request(path, **updates):
    value = yaml.safe_load(path.read_text())
    value.update(updates)
    path.write_text(yaml.safe_dump(value))


def test_analyze_writes_metric_distance_and_readonly_report(job, tmp_path, capsys):
    before = {path: path.read_bytes() for path in job.parent.iterdir() if path.is_file()}
    output = tmp_path / "analysis"
    assert invoke("analyze", job, output) == 0
    with np.load(output / "clearance.npz") as layers:
        assert layers["distance_m"].ndim == 2
        assert layers["known"].shape == layers["distance_m"].shape
        assert np.isfinite(layers["distance_m"]).all()
    report = read_json(output / "report.json")
    assert report["complete"] is False
    assert report["provenance"]["input_files"]
    assert (output / "clearance.png").is_file()
    assert not (output / "plan.json").exists()
    assert all(path.read_bytes() == payload for path, payload in before.items())
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["eligible_for_execution"] is False


@pytest.mark.parametrize("mode", ["rows", "area"])
def test_reference_plan_and_draft_roundtrip(job, tmp_path, mode):
    set_request(job, mode=mode)
    planned, exported = tmp_path / "planned", tmp_path / "exported"
    assert invoke("plan", job, planned) == 0
    plan = read_json(planned / "plan.json")
    assert plan["complete"] is True
    assert len(plan["points"]) >= 2
    assert plan["report"]["mode"] == mode
    assert plan["report"]["eligible_for_execution"] is False
    assert plan["report"]["validation"]["geometric_valid"] is True
    assert plan["report"]["validation"]["dynamic_status"] == "UNKNOWN"
    assert plan["report"]["metrics"]["area"]["coverage_rate"] == pytest.approx(1.)
    assert invoke("export", job, exported, "--plan", planned / "plan.json", "--format", "draft") == 0
    with (exported / "route.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    manifest = yaml.safe_load((exported / "route.yaml").read_text())
    assert manifest["status"] == "DRAFT"
    assert manifest["route_csv_sha256"] == "sha256:" + hashlib.sha256((exported / "route.csv").read_bytes()).hexdigest()
    assert [(float(p["x"]), float(p["y"]), p["direction"]) for p in rows] == [
        (p["x"], p["y"], p["direction"]) for p in plan["points"]]


def test_changed_request_rejects_saved_plan_without_output(job, tmp_path, capsys):
    planned, exported = tmp_path / "planned", tmp_path / "exported"
    assert invoke("plan", job, planned) == 0
    set_request(job, operation_width_m=.9)
    assert invoke("export", job, exported, "--plan", planned / "plan.json") == 2
    assert not exported.exists()
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["code"] == "STALE_PLAN"


def test_signal_cancellation_leaves_no_published_or_staged_output(job, tmp_path, monkeypatch, capsys):
    real = cli.analyze_traversability
    def cancel_after_analysis(context):
        result = real(context)
        os.kill(os.getpid(), signal.SIGTERM)
        return result
    monkeypatch.setattr(cli, "analyze_traversability", cancel_after_analysis)
    previous = signal.getsignal(signal.SIGTERM)
    output = tmp_path / "cancelled"
    assert invoke("analyze", job, output) == 2
    assert signal.getsignal(signal.SIGTERM) == previous
    assert not output.exists()
    assert not list(tmp_path.glob(".coverage-*"))
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["code"] == "CANCELED"


def test_partial_plan_cannot_be_promoted_to_ready(job, tmp_path, capsys):
    publication = job.parent / "publication.yaml"
    publication.write_text(yaml.safe_dump({"route_id": "coverage", "revision": 1,
                                           "review_confirmed": True, "route_reference_point": "base_link"}))
    set_request(job, publication=publication.name)
    planned = tmp_path / "planned"
    assert invoke("plan", job, planned) == 0
    partial = read_json(planned / "plan.json")
    partial["complete"] = False
    partial["missing"] = [{"row_id": "aisle_a", "code": "NO_CONNECTION"}]
    (planned / "plan.json").write_text(json.dumps(partial))
    output = tmp_path / "companion"
    assert invoke("export", job, output, "--plan", planned / "plan.json", "--format", "ready",
                  "--publication", publication, "--confirm-review") == 2
    assert not (output / "routes/coverage/1").exists()
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["code"] == "INCOMPLETE_COVERAGE"


def test_validate_demotes_invalid_complete_claim(job, tmp_path):
    planned, validated = tmp_path / "planned", tmp_path / "validated"
    assert invoke("plan", job, planned) == 0
    payload = read_json(planned / "plan.json")
    payload["points"][1]["y"] += 1.0
    payload["complete"] = True
    (planned / "plan.json").write_text(json.dumps(payload))
    assert invoke("validate", job, validated, "--plan", planned / "plan.json") == 0
    checked = read_json(validated / "plan.json")
    assert checked["report"]["validation"]["valid"] is False
    assert checked["complete"] is False


@pytest.mark.parametrize("operation", ["analyze", "plan"])
def test_cli_rejects_new_outputs_inside_site(job, tmp_path, operation, capsys):
    site = tmp_path / "immutable_site"
    site.mkdir()
    (site / "manifest.yaml").write_text("fixture: immutable\n")
    set_request(job, site_root=str(site))
    output = site / "new_output"
    assert invoke(operation, job, output) == 2
    assert not output.exists()
    assert not list(site.glob(".coverage-*"))
    assert (site / "manifest.yaml").read_text() == "fixture: immutable\n"


def test_ready_cli_allows_existing_companion_store(job, tmp_path, monkeypatch):
    publication = job.parent / "publication.yaml"
    publication.write_text(yaml.safe_dump({"route_id": "coverage", "revision": 2,
                                           "review_confirmed": True, "route_reference_point": "base_link"}))
    set_request(job, publication=publication.name)
    planned = tmp_path / "planned"
    assert invoke("plan", job, planned) == 0
    root = tmp_path / "companion"
    root.mkdir()
    visited = []
    def owner_boundary(context, result, route_store, **settings):
        visited.append((route_store, settings))
        # This test only verifies CLI dispatch; real owner roundtrips are in test_assets.
        raise CoverageError("FIXTURE_OWNER_REACHED", "dispatch reached existing Route publisher")
    monkeypatch.setattr(cli, "publish_ready", owner_boundary)
    assert invoke("export", job, root, "--plan", planned / "plan.json", "--format", "ready",
                  "--publication", publication, "--confirm-review") == 2
    assert visited and visited[0][0] == root
    assert visited[0][1]["revision"] == 2
    assert visited[0][1]["review_confirmed"] is True


def test_publication_configuration_cannot_preapprove_future_route(job, tmp_path, capsys):
    publication = job.parent / "publication.yaml"
    publication.write_text(yaml.safe_dump({"route_id": "coverage", "revision": 1,
                                           "review_confirmed": True, "route_reference_point": "base_link"}))
    set_request(job, publication=publication.name)
    planned = tmp_path / "planned"
    assert invoke("plan", job, planned) == 0
    output = tmp_path / "companion"
    assert invoke("export", job, output, "--plan", planned / "plan.json", "--format", "ready",
                  "--publication", publication) == 2
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["code"] == "REVIEW_REQUIRED"
    assert not output.exists()
