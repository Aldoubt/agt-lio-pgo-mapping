import struct
from pathlib import Path

import pytest
import yaml

from agt_map_refinement_core.pcd import read_pcd, write_pcd
from agt_map_refinement_core.pipeline import refine_map_package
from agt_map_refinement_core.rules import load_rules, point_is_removed


HEADER = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA {fmt}
"""
POINTS = [(0.0, 0.0, 0.0, 1.0), (1.0, 1.0, 0.0, 1.0), (2.0, 2.0, 3.0, 1.0), (4.0, 4.0, 0.0, 1.0)]


def write_binary(path: Path) -> None:
    payload = b''.join(struct.pack('<ffff', *point) for point in POINTS)
    path.write_bytes(HEADER.format(n=len(POINTS), fmt='binary').encode('ascii') + payload)


def make_source(tmp_path: Path) -> Path:
    source = tmp_path / 'source'
    (source / 'patches').mkdir(parents=True)
    write_binary(source / 'map.pcd')
    (source / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
    (source / 'poses_timed.txt').write_text('0.pcd 1.0 0 0 0 1 0 0 0\n')
    (source / 'patches' / '0.pcd').write_text(HEADER.format(n=0, fmt='ascii'))
    (source / 'calibration.yaml').write_text('lidar_to_imu: identity\n')
    (source / 'metadata.yaml').write_text('backend: PGO\n')
    (source / 'manifest.yaml').write_text('schema_version: 1\n')
    (source / 'checksums.sha256').write_text('')
    return source


def test_binary_round_trip_preserves_values(tmp_path):
    write_binary(tmp_path / 'in.pcd')
    pcd = read_pcd(tmp_path / 'in.pcd')
    assert pcd.source_format == 'binary'
    assert write_pcd(tmp_path / 'out.pcd', pcd) == 'binary'
    again = read_pcd(tmp_path / 'out.pcd')
    assert [[float(v) for v in row] for row in again.rows] == [list(p) for p in POINTS]
    assert write_pcd(tmp_path / 'out_ascii.pcd', pcd, 'ascii') == 'ascii'
    assert 'DATA ascii' in (tmp_path / 'out_ascii.pcd').read_text()


def test_new_rule_types_remove_expected_points():
    operations = [
        {'type': 'remove_sphere', 'center': {'x': 1, 'y': 1, 'z': 0}, 'radius': 0.5},
        {'type': 'remove_height_band', 'min_z': 2.5, 'max_z': 3.5},
        {'type': 'remove_polygon', 'points': [[3.5, 3.5], [4.5, 3.5], [4.5, 4.5], [3.5, 4.5]],
         'z_range': [1.0, 2.0]},
    ]
    assert point_is_removed(1.0, 1.0, 0.0, operations)
    assert point_is_removed(2.0, 2.0, 3.0, operations)
    assert not point_is_removed(4.0, 4.0, 0.0, operations)  # outside z_range
    assert not point_is_removed(0.0, 0.0, 0.0, operations)


def test_rule_validation_rejects_bad_geometry(tmp_path):
    for bad in (
        {'type': 'remove_sphere', 'center': {'x': 0, 'y': 0}, 'radius': 1},
        {'type': 'remove_sphere', 'center': {'x': 0, 'y': 0, 'z': 0}, 'radius': 0},
        {'type': 'remove_height_band', 'min_z': 1, 'max_z': 0},
        {'type': 'remove_polygon', 'points': [[0, 0], [1, 0], [1, 1]], 'z_range': [1, 0]},
        {'type': 'remove_box', 'min': {'x': 0, 'y': 0}, 'max': {'x': 1, 'y': 1, 'z': 1}},
    ):
        path = tmp_path / 'rules.yaml'
        path.write_text(yaml.safe_dump({'version': 1, 'operations': [bad]}))
        with pytest.raises(ValueError):
            load_rules(path)


def test_refine_keeps_binary_format_and_copies_calibration(tmp_path):
    rules = tmp_path / 'refinement.yaml'
    rules.write_text(yaml.safe_dump({'version': 1, 'operations': [
        {'type': 'remove_sphere', 'center': {'x': 1, 'y': 1, 'z': 0}, 'radius': 0.1}]}))
    result = refine_map_package(make_source(tmp_path), rules, tmp_path / 'refined')
    refined = read_pcd(result / 'map.pcd')
    assert refined.source_format == 'binary'
    assert len(refined.rows) == 3
    assert (result / 'calibration.yaml').is_file()
    report = yaml.safe_load((result / 'filter_report.yaml').read_text())
    assert report['removed_by_rules'] == 1
    assert report['map_pcd_format'] == 'binary'
    manifest = yaml.safe_load((result / 'manifest.yaml').read_text())
    assert manifest['package_kind'] == 'refined_mapping_source'
    assert len(manifest['parent_map_sha256']) == 64


def test_refine_can_force_ascii_and_skip_preview(tmp_path):
    rules = tmp_path / 'refinement.yaml'
    rules.write_text(yaml.safe_dump({'version': 1, 'operations': []}))
    result = refine_map_package(make_source(tmp_path), rules, tmp_path / 'refined',
                                pcd_format='ascii', write_preview_nav_map=False)
    assert 'DATA ascii' in (result / 'map.pcd').read_text()
    assert not (result / 'nav_map.pgm').exists()
