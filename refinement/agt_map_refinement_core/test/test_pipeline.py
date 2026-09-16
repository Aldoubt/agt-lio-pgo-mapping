import hashlib
from pathlib import Path

import yaml

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
    (source / 'manifest.yaml').write_text('schema_version: 1\n')
    (source / 'checksums.sha256').write_text('')
    return source


def run(tmp_path, rules):
    rules_path = tmp_path / 'refinement.yaml'
    rules_path.write_text(yaml.safe_dump({'version': 1, 'operations': rules}))
    return refine_map_package(make_source(tmp_path), rules_path, tmp_path / 'refined')


def test_empty_rules_preserve_points_and_publish_derivatives(tmp_path):
    result = run(tmp_path, [])
    assert (result / 'map.pcd').read_text() == PCD
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
    assert '1 1 0 1' not in (result / 'map.pcd').read_text()
    assert '2 2 0 1' not in (result / 'map.pcd').read_text()
    assert '4 4 0 1' in (result / 'map.pcd').read_text()


def test_forbidden_zone_is_exported_to_nav_map(tmp_path):
    result = run(tmp_path, [{
        'type': 'forbidden_zone', 'polygon': [[3, 3], [5, 3], [5, 5], [3, 5]],
    }])
    assert '4 4 0 1' in (result / 'map.pcd').read_text()
    assert 100 in (result / 'nav_map.pgm').read_bytes()[3:]