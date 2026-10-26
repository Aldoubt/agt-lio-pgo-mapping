from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from agt_mapping_artifacts.artifact_writer import EMPTY_PCD, sha256
from agt_mapping_artifacts.derived_export import (
    finish_cleanup, prepare_cleanup, publish_directory, snapshot_source,
)
from agt_mapping_artifacts.map_package_exporter import _write_integrity_files
from agt_mapping_artifacts.validation import _checksum_index, verify_source_integrity


@pytest.fixture
def source(tmp_path):
    root = tmp_path / 'source'
    (root / 'patches').mkdir(parents=True)
    for name in ('map.pcd', 'patches/0.pcd'):
        (root / name).write_text(EMPTY_PCD)
    (root / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
    (root / 'poses_timed.txt').write_text('0.pcd 1 0 0 0 1 0 0 0\n')
    (root / 'calibration.yaml').write_text('status: synthetic_fixture\n')
    (root / 'metadata.yaml').write_text('fixture: true\n')
    _write_integrity_files(root)
    return root


def prepare_tree(tmp_path, source):
    stage, output = tmp_path / '.staging-test', tmp_path / 'result'
    stage.mkdir()
    (stage / 'projection.yaml').write_text(yaml.safe_dump({
        'projection_mode': 'traversability',
        'robot': {'footprint': [[1, 1], [1, -1], [-1, 0]],
                  'base_from_body': {'xyz': [0, 0, 1], 'rpy': [0, 0, 0]}},
    }))
    prepare_cleanup(source, stage, output, '0.1.0', 'clouds')
    (stage / 'clouds').mkdir()
    for name in ('static_map.pcd', 'removed_candidates.pcd'):
        (stage / 'clouds' / name).write_text(EMPTY_PCD)
    (stage / 'clouds/cloud_cleanup.yaml').write_text('removed_candidate_points: 0\n')
    (stage / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xcd')
    (stage / 'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n')
    (stage / 'metadata.yaml').write_text('input_points: 0\n')
    return stage, output


def test_cleanup_is_complete_bound_and_does_not_modify_source(tmp_path, source):
    original = snapshot_source(source)
    stage, output = prepare_tree(tmp_path, source)
    finish_cleanup(stage, output)
    assert not stage.exists()
    assert snapshot_source(source) == original
    assert not (output / '.input_snapshot.yaml').exists()
    metadata = yaml.safe_load((output / 'metadata.yaml').read_text())['derivation']
    assert metadata['source'] == original
    assert metadata['projection_sha256'] == sha256(output / 'projection.yaml')
    assert metadata['producer'] == {'package': 'agt_pcd2grid_exporter', 'version': '0.1.0'}
    assert metadata['status'] == 'REVIEW_ONLY_NOT_READY'
    index = _checksum_index(output / 'checksums.sha256')
    assert set(index) == {p.relative_to(output).as_posix() for p in output.rglob('*')
                          if p.is_file() and p.name != 'checksums.sha256'}
    assert all(sha256(output / name) == digest for name, digest in index.items())
    manifest = yaml.safe_load((output / 'manifest.yaml').read_text())
    assert manifest['format_version'] == 1
    assert manifest['checksums'] == {k: v for k, v in index.items() if k != 'manifest.yaml'}


@pytest.mark.parametrize('changed', ['source', 'projection', 'missing_cloud'])
def test_stale_or_incomplete_export_never_publishes(tmp_path, source, changed):
    stage, output = prepare_tree(tmp_path, source)
    if changed == 'source':
        (source / 'calibration.yaml').write_text('status: changed\n')
        _write_integrity_files(source)
    elif changed == 'projection':
        with (stage / 'projection.yaml').open('a') as stream:
            stream.write('resolution: 0.5\n')
    else:
        (stage / 'clouds/static_map.pcd').unlink()
    with pytest.raises(ValueError):
        finish_cleanup(stage, output)
    assert not output.exists()


def test_even_empty_existing_destination_is_never_replaced(tmp_path, source):
    stage, output = prepare_tree(tmp_path, source)
    output.mkdir()
    with pytest.raises(FileExistsError):
        finish_cleanup(stage, output)
    assert output.is_dir() and not list(output.iterdir())


def test_source_child_and_symlink_destinations_rejected(tmp_path, source):
    stage, _ = prepare_tree(tmp_path, source)
    with pytest.raises(ValueError, match='outside protected'):
        prepare_cleanup(source, stage, source / 'cleanup', '0.1.0')
    alias = tmp_path / 'source-alias'
    alias.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError, match='outside protected'):
        prepare_cleanup(source, stage, alias / 'cleanup', '0.1.0')
    dangling = tmp_path / 'dangling-output'
    dangling.symlink_to(tmp_path / 'absent')
    with pytest.raises(FileExistsError):
        prepare_cleanup(source, stage, dangling, '0.1.0')


def test_unknown_source_layout_and_payload_tampering_fail_closed(source):
    (source / 'patches/0.pcd').write_text('changed')
    with pytest.raises(ValueError, match='checksum'):
        verify_source_integrity(source)
    _write_integrity_files(source)
    (source / 'manifest.yaml').write_text('package_kind: unknown\nschema_version: 77\n')
    with pytest.raises(ValueError, match='Unsupported'):
        verify_source_integrity(source)


def test_actual_module_entrypoint_performs_publication(tmp_path, source):
    stage, output = prepare_tree(tmp_path, source)
    completed = subprocess.run(
        [sys.executable, '-m', 'agt_mapping_artifacts.derived_export', 'finish',
         '--stage', str(stage), '--output', str(output)], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    assert (output / 'manifest.yaml').is_file()


def test_atomic_publisher_rejects_existing_revision(tmp_path):
    stage, output = tmp_path / '.stage', tmp_path / 'revision'
    stage.mkdir()
    (stage / 'payload').write_text('new')
    output.mkdir()
    (output / 'payload').write_text('old')
    with pytest.raises(FileExistsError):
        publish_directory(stage, output)
    assert (output / 'payload').read_text() == 'old'


def test_changed_selected_projection_file_invalidates_prepared_result(tmp_path, source):
    stage, output = prepare_tree(tmp_path, source)
    selected = tmp_path / 'selected_robot_projection.yaml'
    selected.write_bytes((stage / 'projection.yaml').read_bytes())
    (stage / 'input_projection.yaml').write_bytes(selected.read_bytes())
    prepare_cleanup(source, stage, output, '0.1.0', 'clouds', selected)
    selected.write_text(selected.read_text() + '\nresolution: 0.25\n')
    with pytest.raises(ValueError, match='Selected projection config changed'):
        finish_cleanup(stage, output)
    assert not output.exists()
