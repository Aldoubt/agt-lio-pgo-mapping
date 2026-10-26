"""Optional real PCL regression; no mocked grid builder stands in for this check."""
import math
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from agt_mapping_artifacts.map_package_exporter import _write_integrity_files
from agt_map_refinement_core.pcd import AsciiPcd, read_pcd, write_pcd
from agt_map_refinement_core.pipeline import refine_map_package


def write_points(path, points):
    header = [
        'VERSION 0.7', 'FIELDS x y z intensity', 'SIZE 4 4 4 4', 'TYPE F F F F',
        'COUNT 1 1 1 1', f'WIDTH {len(points)}', 'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0', f'POINTS {len(points)}', 'DATA binary',
    ]
    write_pcd(path, AsciiPcd(header, ['x', 'y', 'z', 'intensity'],
                            [[repr(value) for value in point] for point in points], 'binary'))


def pixel_at(directory, x, y):
    metadata = yaml.safe_load((directory / 'map.yaml').read_text())
    magic, shape, maximum, pixels = (directory / 'map.pgm').read_bytes().split(b'\n', 3)
    assert magic == b'P5' and maximum == b'255'
    width, height = map(int, shape.split())
    assert len(pixels) == width * height
    column = math.floor((x - metadata['origin'][0]) / metadata['resolution'])
    row = math.floor((y - metadata['origin'][1]) / metadata['resolution'])
    assert 0 <= column < width and 0 <= row < height
    return pixels[(height - 1 - row) * width + column]


def test_real_pcl_reprojection_does_not_restore_deleted_obstacle_or_reject_empty_patch(tmp_path):
    binary = os.environ.get('AGT_PCD2GRID_EXECUTABLE')
    if not binary:
        pytest.skip('Set AGT_PCD2GRID_EXECUTABLE to run the native PCL reprojection regression')
    executable = Path(binary).expanduser().resolve(strict=True)
    source = tmp_path / 'source'
    (source / 'patches').mkdir(parents=True)
    floor = [[-1.95 + .4 * i, -1.95 + .4 * j, 0.0, 3.0]
             for i in range(16) for j in range(10)]
    post = [[2.05, 1.05, .5, 7.0], [2.05, 1.05, .8, 11.0]]
    poses, timed, merged = [], [], []
    for index, tx in enumerate((0.0, .5, 1.0, 1.5)):
        # The final patch becomes empty after deleting the post; preserve its pose.
        observed = floor + post if index < 3 else post
        body = [[x - tx, y, z - 1.0, intensity] for x, y, z, intensity in observed]
        write_points(source / 'patches' / f'{index}.pcd', body)
        # Regenerate the map from the exact float32 patch rows, just like the producer.
        patch = read_pcd(source / 'patches' / f'{index}.pcd')
        merged.extend([[float(row[0]) + tx, float(row[1]), float(row[2]) + 1.0, float(row[3])]
                       for row in patch.rows])
        poses.append(f'{index}.pcd {tx} 0 1 1 0 0 0\n')
        timed.append(f'{index}.pcd {100 + index} {tx} 0 1 1 0 0 0\n')
    write_points(source / 'map.pcd', merged)
    (source / 'poses.txt').write_text(''.join(poses))
    (source / 'poses_timed.txt').write_text(''.join(timed))
    (source / 'metadata.yaml').write_text(yaml.safe_dump({
        'package_kind': 'mapping_source', 'patches_frame': 'body', 'pose_semantics': 'T_map_body',
        'frames': {'map': 'map', 'mapping_body': 'body'},
    }))
    (source / 'calibration.yaml').write_text('status: synthetic regression fixture\n')
    _write_integrity_files(source)
    before = {p.relative_to(source).as_posix(): p.read_bytes() for p in source.rglob('*') if p.is_file()}
    config = tmp_path / 'projection.yaml'
    config.write_text(yaml.safe_dump({
        'projection_mode': 'traversability', 'resolution': .1,
        'temporal_filter': {'enabled': False}, 'occupied_threshold': 2,
        'postprocess': {'closing_radius_cells': 0, 'min_component_cells': 1},
        'robot': {'base_from_body': {'xyz': [0, 0, 1], 'rpy': [0, 0, 0]},
                  'footprint': [[.3, .2], [.3, -.2], [-.3, -.2], [-.3, .2]]},
        'traversability': {'self_filter': {'enabled': False}, 'hole_fill_max_area': 0,
                           'obstacle_evidence': {'min_observation_range': 0}},
    }))

    def project(package, output):
        completed = subprocess.run([str(executable), '--package', str(package),
                                    '--output', str(output), '--config', str(config)],
                                   capture_output=True, text=True, timeout=30)
        assert completed.returncode == 0, completed.stdout + completed.stderr

    original_grid = tmp_path / 'original-grid'
    project(source, original_grid)
    assert pixel_at(original_grid, 2.05, 1.05) == 0, 'The fixture must first contain the occupied post'
    rules = tmp_path / 'rules.yaml'
    rules.write_text(yaml.safe_dump({'operations': [{
        'type': 'remove_box', 'min': {'x': 1.95, 'y': .95, 'z': .3},
        'max': {'x': 2.15, 'y': 1.15, 'z': 1.0},
    }]}))
    refined = refine_map_package(source, rules, tmp_path / 'refined', write_preview_nav_map=False)
    assert read_pcd(refined / 'patches' / '3.pcd').rows == []
    rebuilt_grid = tmp_path / 'rebuilt-grid'
    project(refined, rebuilt_grid)
    assert pixel_at(rebuilt_grid, 2.05, 1.05) != 0, 'Deleted patch observations must not return as obstacles'
    assert {p.relative_to(source).as_posix(): p.read_bytes()
            for p in source.rglob('*') if p.is_file()} == before
