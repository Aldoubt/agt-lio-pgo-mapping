import json
from pathlib import Path

from agt_mapping_bringup import review


def completed_output(tmp_path):
    output = tmp_path / 'run'
    artifact = output / 'map_package'
    artifact.mkdir(parents=True)
    (artifact / 'map.pcd').write_bytes(b'pcd')
    (artifact / 'manifest.yaml').write_text('schema_version: 1\n', encoding='utf-8')
    (output / 'session.json').write_text(json.dumps({
        'status': 'completed',
        'artifact_verified': True,
        'artifact_path': str(artifact),
    }), encoding='utf-8')
    return output, artifact


def test_plan_uses_only_in_repository_review_tools(tmp_path):
    output, artifact = completed_output(tmp_path)
    plan = review.make_plan(output, artifact)
    commands = plan['converter_command'] + plan['studio_command']
    assert 'agt_pcd2grid_exporter' in commands
    assert 'agt_map_studio' in commands
    assert not any('navigation_v3' in item for item in commands)
    assert 'agt_map_converter' not in commands
    assert 'agt_map_manager' not in commands


def test_dry_run_validates_and_does_not_create_review_directory(tmp_path, capsys):
    output, _ = completed_output(tmp_path)
    assert review.main([str(output), '--dry-run']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['base_map'].endswith('/map_review/base')
    assert not (output / 'map_review').exists()


def test_rejects_unfinished_mapping_output(tmp_path, capsys):
    output, _ = completed_output(tmp_path)
    (output / 'session.json').write_text(json.dumps({
        'status': 'cancelled', 'artifact_verified': False,
    }), encoding='utf-8')
    assert review.main([str(output), '--dry-run']) == 2
    assert 'not complete' in capsys.readouterr().err


def test_conversion_precedes_studio_and_produces_expected_paths(tmp_path, monkeypatch):
    output, _ = completed_output(tmp_path)
    calls = []

    class Result:
        returncode = 0

    def fake_run(command, check):
        assert check is False
        calls.append(command)
        if 'agt_pcd2grid_exporter' in command:
            base = output / 'map_review' / 'base'
            base.mkdir(parents=True)
            (base / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
            (base / 'map.yaml').write_text('image: map.pgm\n', encoding='utf-8')
        if 'agt_map_studio' in command:
            confirmed = output / 'map_review' / 'confirmed'
            confirmed.mkdir(parents=True)
            (confirmed / 'review_status.yaml').write_text(
                'status: confirmed\n', encoding='utf-8'
            )
        return Result()

    monkeypatch.setattr(review.subprocess, 'run', fake_run)
    assert review.main([str(output)]) == 0
    assert calls[0][2:4] == ['agt_pcd2grid_exporter', 'pcd2grid_exporter']
    assert calls[1][2:4] == ['agt_map_studio', 'map_viewer']
    assert '--review-package' in calls[1]


def test_closing_editor_without_confirmation_is_an_error(tmp_path, monkeypatch, capsys):
    output, _ = completed_output(tmp_path)
    base = output / 'map_review' / 'base'
    base.mkdir(parents=True)
    (base / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
    (base / 'map.yaml').write_text('image: map.pgm\n', encoding='utf-8')

    class Result:
        returncode = 0

    monkeypatch.setattr(review.subprocess, 'run', lambda command, check: Result())
    assert review.main([str(output)]) == 2
    assert 'without confirming' in capsys.readouterr().err
