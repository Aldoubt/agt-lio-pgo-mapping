import hashlib
from pathlib import Path

import yaml
import pytest

from agt_mapping_artifacts.map_package_exporter import _write_integrity_files
from agt_mapping_artifacts.keyframe_blocks import _read_pose_records
from agt_map_refinement_core.pcd import read_pcd

from agt_map_refinement_core.pipeline import refine_map_package


PCD = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 4
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4
DATA ascii
0 0 0 1
1 1 0 1
2 2 0 1
4 4 0 1
"""


def make_source(tmp_path: Path) -> Path:
    source = tmp_path / 'source'
    (source / 'patches').mkdir(parents=True)
    (source / 'map.pcd').write_text(PCD)
    (source / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
    (source / 'poses_timed.txt').write_text('0.pcd 1.0 0 0 0 1 0 0 0\n')
    (source / 'patches' / '0.pcd').write_text(PCD)
    (source / 'metadata.yaml').write_text('backend: PGO\n')
    (source / 'calibration.yaml').write_text('lidar_to_imu: identity\n')
    _write_integrity_files(source)
    return source


def run(tmp_path, rules):
    rules_path = tmp_path / 'refinement.yaml'
    rules_path.write_text(yaml.safe_dump({'version': 1, 'operations': rules}))
    return refine_map_package(make_source(tmp_path), rules_path, tmp_path / 'refined')


def test_empty_rules_preserve_points_and_publish_derivatives(tmp_path):
    result = run(tmp_path, [])
    assert [[float(v) for v in row] for row in read_pcd(result / 'map.pcd').rows] == [
        [float(v) for v in row] for row in read_pcd(tmp_path / 'source' / 'map.pcd').rows]
    assert (result / 'nav_map.pgm').read_bytes().startswith(b'P5\n')
    assert (result / 'nav_map.yaml').is_file()
    assert yaml.safe_load((result / 'filter_report.yaml').read_text())['removed_by_rules'] == 0
    manifest = yaml.safe_load((result / 'manifest.yaml').read_text())
    assert manifest['package_kind'] == 'refined_mapping_source'
    checksums = {name: digest for digest, name in
                 (line.split('  ', 1) for line in (result / 'checksums.sha256').read_text().splitlines())}
    for name, digest in checksums.items():
        assert hashlib.sha256((result / name).read_bytes()).hexdigest() == digest
        assert manifest['checksums'][name] == digest


def test_polygon_and_box_remove_points(tmp_path):
    result = run(tmp_path, [{
        'type': 'remove_polygon', 'points': [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]],
    }, {
        'type': 'remove_box', 'min': {'x': 1.9, 'y': 1.9, 'z': -1},
        'max': {'x': 2.1, 'y': 2.1, 'z': 1},
    }])
    rows = [[float(v) for v in row] for row in read_pcd(result / 'map.pcd').rows]
    assert [1, 1, 0, 1] not in rows
    assert [2, 2, 0, 1] not in rows
    assert [4, 4, 0, 1] in rows
    patch = [[float(v) for v in row] for row in read_pcd(result / 'patches' / '0.pcd').rows]
    assert patch == rows


def test_forbidden_zone_is_exported_to_nav_map(tmp_path):
    result = run(tmp_path, [{
        'type': 'forbidden_zone', 'polygon': [[3, 3], [5, 3], [5, 5], [3, 5]],
    }])
    assert [4, 4, 0, 1] in [[float(v) for v in row] for row in read_pcd(result / 'map.pcd').rows]
    assert 100 in (result / 'nav_map.pgm').read_bytes()[3:]


def snapshot(root):
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in root.rglob('*') if p.is_file()}


def test_map_frame_deletion_survives_patch_rebuild_and_preserves_fields_and_source(tmp_path):
    source = make_source(tmp_path)
    # T_map_body: a 90-degree yaw and translation; body (1,1,0) becomes (9,3,0).
    q = '0.7071067811865476'
    (source / 'poses.txt').write_text(f'0.pcd 10 2 0 {q} 0 0 {q}\n')
    (source / 'poses_timed.txt').write_text(f'0.pcd 1.0 10 2 0 {q} 0 0 {q}\n')
    _write_integrity_files(source)
    before = snapshot(source)
    rules = tmp_path / 'rules.yaml'
    rules.write_text(yaml.safe_dump({'version': 1, 'operations': [{
        'type': 'remove_box', 'min': {'x': 8.9, 'y': 2.9, 'z': -1},
        'max': {'x': 9.1, 'y': 3.1, 'z': 1},
    }]}))
    result = refine_map_package(source, rules, tmp_path / 'refined', write_preview_nav_map=False)
    patch = read_pcd(result / 'patches' / '0.pcd')
    assert [1.0, 1.0, 0.0, 1.0] not in [[float(v) for v in row] for row in patch.rows]
    pose = _read_pose_records(result)[0]
    rebuilt = [list(pose['rotation'].dot([float(v) for v in row[:3]]) + pose['position'])
               + [float(row[3])] for row in patch.rows]
    assert [[float(v) for v in row] for row in read_pcd(result / 'map.pcd').rows] == rebuilt
    assert all(abs(row[0] - 9) > .1 or abs(row[1] - 3) > .1 for row in rebuilt)
    assert snapshot(source) == before
    for filename in ('poses.txt', 'poses_timed.txt', 'calibration.yaml'):
        assert (result / filename).read_bytes() == before[filename]


@pytest.mark.parametrize('bad_pose', [
    '0.pcd 1 0 0 0 0 0 0 0\n',  # zero quaternion
    '0.pcd 1 nan 0 0 1 0 0 0\n',
    '../0.pcd 1 0 0 0 1 0 0 0\n',
    '1.pcd 1 0 0 0 1 0 0 0\n',  # mismatched plain-pose reference
    '0.pcd 1 5 0 0 1 0 0 0\n',  # mismatched plain-pose transform
])
def test_bad_pose_rejected_without_output(tmp_path, bad_pose):
    source = make_source(tmp_path)
    (source / 'poses_timed.txt').write_text(bad_pose)
    _write_integrity_files(source)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    with pytest.raises(ValueError):
        refine_map_package(source, rules, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()
    assert not list(tmp_path.glob('.refined-*'))


def test_empty_patch_keeps_pose_and_removes_all_observations(tmp_path):
    rules = tmp_path / 'rules.yaml'
    rules.write_text(yaml.safe_dump({'operations': [
        {'type': 'remove_height_band', 'min_z': -1, 'max_z': 1}]}))
    result = refine_map_package(make_source(tmp_path), rules, tmp_path / 'refined',
                                write_preview_nav_map=False)
    assert read_pcd(result / 'patches' / '0.pcd').rows == []
    assert read_pcd(result / 'map.pcd').rows == []
    assert (result / 'poses.txt').read_bytes() == (tmp_path / 'source' / 'poses.txt').read_bytes()


def test_refuse_existing_and_source_nested_output(tmp_path):
    source = make_source(tmp_path)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    before = snapshot(source)
    existing = tmp_path / 'existing'
    existing.mkdir()
    with pytest.raises(FileExistsError):
        refine_map_package(source, rules, existing)
    with pytest.raises(ValueError, match='outside'):
        refine_map_package(source, rules, source / 'derived')
    assert snapshot(source) == before


def test_failure_and_concurrent_rule_change_discard_staging(tmp_path, monkeypatch):
    from agt_map_refinement_core import pipeline
    source = make_source(tmp_path)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    original = pipeline._write_integrity

    def changing_rules(stage, parent):
        original(stage, parent)
        rules.write_text('operations: []\n# changed\n')

    monkeypatch.setattr(pipeline, '_write_integrity', changing_rules)
    with pytest.raises(ValueError, match='rules changed'):
        refine_map_package(source, rules, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()
    assert not list(tmp_path.glob('.refined-*'))


def test_write_failure_discards_staging_without_changing_source(tmp_path, monkeypatch):
    from agt_map_refinement_core import pipeline
    source = make_source(tmp_path)
    before = snapshot(source)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')

    def fail_write(*args, **kwargs):
        raise OSError('injected disk write failure')

    monkeypatch.setattr(pipeline, 'write_pcd', fail_write)
    with pytest.raises(OSError, match='disk write'):
        refine_map_package(source, rules, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()
    assert not list(tmp_path.glob('.refined-*'))
    assert snapshot(source) == before


def test_source_changed_during_derivation_cannot_publish(tmp_path, monkeypatch):
    from agt_map_refinement_core import pipeline
    source = make_source(tmp_path)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    original = pipeline._write_integrity

    def mutate_parent(stage, parent):
        original(stage, parent)
        with (source / 'poses.txt').open('a') as stream:
            stream.write('# concurrent change\n')

    monkeypatch.setattr(pipeline, '_write_integrity', mutate_parent)
    with pytest.raises(ValueError, match='checksum'):
        refine_map_package(source, rules, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()
    assert not list(tmp_path.glob('.refined-*'))


def test_symlink_source_rejected(tmp_path):
    source = make_source(tmp_path)
    link = tmp_path / 'source-link'
    link.symlink_to(source, target_is_directory=True)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    with pytest.raises(ValueError, match='symbolic link'):
        refine_map_package(link, rules, tmp_path / 'out')


def test_racing_empty_output_directory_is_not_replaced(tmp_path, monkeypatch):
    from agt_map_refinement_core import pipeline
    source = make_source(tmp_path)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    original = pipeline.publish_directory

    def racing_publish(stage, output):
        output.mkdir()
        original(stage, output)

    monkeypatch.setattr(pipeline, 'publish_directory', racing_publish)
    with pytest.raises(FileExistsError):
        refine_map_package(source, rules, tmp_path / 'out')
    assert list((tmp_path / 'out').iterdir()) == []
    assert not list(tmp_path.glob('.refined-*'))


def test_refinement_invalidates_parent_localization_assets(tmp_path):
    source = make_source(tmp_path)
    (source / 'metadata.yaml').write_text(yaml.safe_dump({
        'backend': 'PGO', 'relocalization': {'available': True, 'path': 'relocalization'},
    }))
    _write_integrity_files(source)
    rules = tmp_path / 'rules.yaml'
    rules.write_text('operations: []\n')
    result = refine_map_package(source, rules, tmp_path / 'out')
    metadata = yaml.safe_load((result / 'metadata.yaml').read_text())
    assert metadata['relocalization']['available'] is False
    assert metadata['artifact_kind'] == 'refined_mapping_source'


def test_reordered_point_fields_keep_retained_intensity(tmp_path):
    from agt_map_refinement_core.pcd import write_pcd
    source = make_source(tmp_path)
    cloud = read_pcd(source / 'map.pcd')
    cloud.fields = ['intensity', 'z', 'y', 'x']
    cloud.header = [line if not line.startswith('FIELDS ') else 'FIELDS intensity z y x'
                    for line in cloud.header]
    cloud.rows = [['42.5', '0', '0', '0'], ['9', '0', '1', '1'], ['-4.25', '0', '4', '4']]
    write_pcd(source / 'map.pcd', cloud, 'binary')
    write_pcd(source / 'patches' / '0.pcd', cloud, 'binary')
    _write_integrity_files(source)
    rules = tmp_path / 'rules.yaml'
    rules.write_text(yaml.safe_dump({'operations': [{
        'type': 'remove_sphere', 'center': {'x': 1, 'y': 1, 'z': 0}, 'radius': .1,
    }]}))
    output = refine_map_package(source, rules, tmp_path / 'out', write_preview_nav_map=False)
    for relative in ('map.pcd', 'patches/0.pcd'):
        kept = read_pcd(output / relative)
        assert kept.fields == cloud.fields
        assert [[float(value) for value in row] for row in kept.rows] == [
            [42.5, 0, 0, 0], [-4.25, 0, 4, 4]]
