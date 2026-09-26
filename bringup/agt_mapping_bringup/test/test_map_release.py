import json
from pathlib import Path
import subprocess

import pytest
from agt_mapping_bringup import map_release, review


@pytest.fixture
def release_environment(tmp_path, monkeypatch):
    ws = tmp_path / 'ros2_ws'
    ws.mkdir()
    source = ws / 'source'
    source.mkdir()
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(map_release, 'verify_artifact', lambda value: source)
    return ws, source


def prepare_args(ws, source):
    return ['prepare', '--source', str(source), '--map-id', 'bunker_mid360',
            '--version', 'test-v1', '--workspace', str(ws)]


def test_default_review_selects_traversability(tmp_path):
    plan = review.make_plan(tmp_path, tmp_path / 'map_package')
    assert '--config' in plan['converter_command']
    assert plan['converter_command'][-1].endswith('projection_traversability.yaml')


def test_explicit_review_configuration_remains_supported(tmp_path):
    config = tmp_path / 'legacy.yaml'
    plan = review.make_plan(tmp_path, tmp_path, config)
    assert plan['converter_command'][-1] == str(config)


def test_prepare_dry_run_never_creates_output(release_environment, capsys):
    ws, source = release_environment
    assert map_release.main(prepare_args(ws, source) + ['--dry-run']) == 0
    plan = json.loads(capsys.readouterr().out)
    assert len(plan['commands']) == 4
    assert plan['commands'][0][2] == 'agt_pcd2grid_exporter'
    assert plan['commands'][-1][3] == 'build_map_candidate'
    assert not (ws / 'experiments').exists()
    assert not (ws / 'maps').exists()


def test_yhs_prepare_requires_explicit_profile_projection(release_environment, capsys):
    ws, source = release_environment
    options = ['prepare', '--source', str(source), '--map-id', 'yhs_mid360',
               '--version', 'test-v1', '--robot', 'yhs_v1', '--workspace', str(ws), '--dry-run']
    assert map_release.main(options) == 2
    assert 'requires explicit --config' in capsys.readouterr().err
    config = ws / 'yhs_measured_projection.yaml'
    config.write_text('robot_profile: bunker_v1\nfield_verified: true\nverified_by: fixture\n')
    assert map_release.main(options + ['--config', str(config)]) == 2
    assert 'YHS projection requires robot_profile: yhs_v1' in capsys.readouterr().err
    config.write_text('robot_profile: yhs_v1\nfield_verified: true\nverified_by: test fixture only\n')
    assert map_release.main(options + ['--config', str(config)]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan['robot'] == 'yhs_v1' and plan['config'] == str(config)
    assert not (ws / 'experiments').exists()


def test_prepare_failure_stops_before_candidate_build(release_environment, monkeypatch):
    ws, source = release_environment
    calls = []
    def fail(command, check):
        calls.append(command)
        raise subprocess.CalledProcessError(1, command)
    monkeypatch.setattr(map_release.subprocess, 'run', fail)
    assert map_release.main(prepare_args(ws, source)) == 2
    assert len(calls) == 1
    assert not (ws / 'maps').exists()


def test_prepare_never_overwrites_version(release_environment):
    ws, source = release_environment
    (ws / 'experiments/mapping/exports/bunker_mid360/test-v1').mkdir(parents=True)
    assert map_release.main(prepare_args(ws, source) + ['--dry-run']) == 2


@pytest.mark.parametrize('name', ['../escape', '/tmp/escape', '.', 'x/y', ''])
def test_unsafe_identifier_rejected(name):
    with pytest.raises(ValueError):
        map_release._identifier(name)


def test_source_integrity_failure_blocks_prepare(release_environment, monkeypatch):
    ws, source = release_environment
    def invalid(value):
        raise ValueError('checksum mismatch')
    monkeypatch.setattr(map_release, 'verify_artifact', invalid)
    assert map_release.main(prepare_args(ws, source) + ['--dry-run']) == 2
    assert not (ws / 'experiments').exists()


def test_publication_requires_human_acceptance(tmp_path):
    assert map_release.main(['publish', '--candidate', str(tmp_path), '--dry-run']) == 2


def test_publication_rejects_outside_experiments(tmp_path):
    assert map_release.main(['publish', '--candidate', str(tmp_path),
                             '--confirm-reviewed', '--dry-run']) == 2


def test_yhs_publication_requires_matching_prep_plan(release_environment, capsys):
    ws, _ = release_environment
    candidate = ws / 'experiments/mapping/candidates/yhs_mid360/test-v1'
    candidate.mkdir(parents=True)
    (candidate / 'candidate_state.yaml').write_text('state: BUILT\n')
    args = ['publish', '--candidate', str(candidate), '--robot', 'yhs_v1',
            '--workspace', str(ws), '--confirm-reviewed', '--dry-run']
    assert map_release.main(args) == 2
    assert 'missing recorded YHS prepare plan' in capsys.readouterr().err
    plan = ws / 'experiments/mapping/exports/yhs_mid360/test-v1/release_plan.json'
    plan.parent.mkdir(parents=True)
    plan.write_text(json.dumps({'candidate': str(candidate), 'robot': 'bunker_v1'}))
    assert map_release.main(args) == 2
    assert 'provenance mismatch' in capsys.readouterr().err
    plan.write_text(json.dumps({'candidate': str(candidate), 'robot': 'yhs_v1'}))
    assert map_release.main(args) == 0
    out = json.loads(capsys.readouterr().out)
    assert out['robot'] == 'yhs_v1'
    assert out['commands'][0][-2:] == ['--robot', 'yhs_v1']
    assert '--activate' not in out['commands'][0]


def test_publication_uses_existing_transaction_no_legacy_activation(release_environment, capsys):
    ws, _ = release_environment
    candidate = ws / 'experiments/mapping/candidates/site/v1'
    candidate.mkdir(parents=True)
    (candidate / 'candidate_state.yaml').write_text('state: BUILT\n')
    assert map_release.main(['publish', '--candidate', str(candidate),
                             '--workspace', str(ws), '--confirm-reviewed', '--dry-run']) == 0
    plan = json.loads(capsys.readouterr().out)
    command = plan['commands'][0]
    assert command[3] == 'promote_map_candidate'
    assert '--activate' not in command
    assert str(ws / 'maps') in command
    assert not (ws / 'maps').exists()
