"""Exercise the real native executable's transaction boundary (no fake producer)."""
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from agt_mapping_artifacts.artifact_writer import sha256
from agt_mapping_artifacts.map_package_exporter import _write_integrity_files


def tree_bytes(root):
    return {path.relative_to(root).as_posix(): path.read_bytes()
            for path in root.rglob('*') if path.is_file()}


def pcd(path, points):
    path.write_text('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                    'TYPE F F F F\nCOUNT 1 1 1 1\n'
                    f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                    + ''.join(' '.join(map(str, row)) + '\n' for row in points))


@pytest.fixture
def native_case(tmp_path):
    binary = os.environ.get('AGT_PCD2GRID_EXECUTABLE')
    if not binary:
        pytest.skip('Set AGT_PCD2GRID_EXECUTABLE for native CLI transaction checks')
    source = tmp_path / 'source'
    (source / 'patches').mkdir(parents=True)
    ground = [[.05 + .4 * x, -.95 + .4 * y, 0, 1] for x in range(12) for y in range(8)]
    # This static post is only observed from nearby; it intentionally exercises
    # the documented heuristic boundary, without claiming semantic detection.
    post = [[.55, 1.05, .5, 7], [.55, 1.05, .8, 9]]
    world = ground + post
    plain, timed = [], []
    for index, x in enumerate((0, .5, 1)):
        pcd(source / 'patches' / f'{index}.pcd',
            [[px - x, py, pz - 1, intensity] for px, py, pz, intensity in world])
        plain.append(f'{index}.pcd {x} 0 1 1 0 0 0\n')
        timed.append(f'{index}.pcd {100 + index} {x} 0 1 1 0 0 0\n')
    pcd(source / 'map.pcd', world * 3)
    (source / 'poses.txt').write_text(''.join(plain))
    (source / 'poses_timed.txt').write_text(''.join(timed))
    (source / 'calibration.yaml').write_text('status: synthetic_test_fixture\n')
    (source / 'metadata.yaml').write_text('fixture: true\n')
    _write_integrity_files(source)
    config = tmp_path / 'projection.yaml'
    config.write_text(yaml.safe_dump({
        'projection_mode': 'traversability', 'resolution': .1,
        'temporal_filter': {'enabled': False}, 'occupied_threshold': 2,
        'postprocess': {'closing_radius_cells': 0, 'min_component_cells': 1},
        'robot': {'base_from_body': {'xyz': [0, 0, 1], 'rpy': [0, 0, 0]},
                  'footprint': [[.3, .2], [.3, -.2], [-.3, -.2], [-.3, .2]]},
        'traversability': {'self_filter': {'enabled': False}, 'hole_fill_max_area': 0,
                           'obstacle_evidence': {'min_observation_range': 2}},
    }))

    def run(output, cloud=None, include_config=True):
        arguments = [binary, '--package', str(source), '--output', str(output)]
        if include_config:
            arguments += ['--config', str(config)]
        if cloud is not None:
            arguments += ['--cloud-output', str(cloud)]
        return subprocess.run(arguments, capture_output=True, text=True, timeout=30)

    return source, config, run


def test_complete_cli_publication_matches_unadorned_grid_and_records_heuristic_limit(tmp_path, native_case):
    source, config, run = native_case
    before = tree_bytes(source)
    plain, review = tmp_path / 'plain', tmp_path / 'review'
    completed = run(plain)
    assert completed.returncode == 0, completed.stderr
    completed = run(review, 'clouds')
    assert completed.returncode == 0, completed.stderr
    assert (review / 'map.pgm').read_bytes() == (plain / 'map.pgm').read_bytes()
    assert (review / 'map.yaml').read_bytes() == (plain / 'map.yaml').read_bytes()
    metadata = yaml.safe_load((review / 'metadata.yaml').read_text())['derivation']
    assert metadata['projection_sha256'] == sha256(review / 'projection.yaml')
    assert metadata['projection_input'] == {'path': str(config), 'sha256': sha256(config)}
    assert (review / 'input_projection.yaml').read_bytes() == config.read_bytes()
    assert metadata['source']['files']['manifest.yaml'] == sha256(source / 'manifest.yaml')
    assert metadata['cloud_subdirectory'] == 'clouds'
    assert 'near-only static obstacles can be removed' in metadata['limitations']
    report = yaml.safe_load((review / 'clouds/cloud_cleanup.yaml').read_text())
    assert report['removed_candidate_points'] == 6  # Static near-only post, in all three frames.
    assert tree_bytes(source) == before
    published = tree_bytes(review)
    assert run(review, 'clouds').returncode != 0
    assert tree_bytes(review) == published
    assert not list(tmp_path.glob('.review.staging-*'))


@pytest.mark.parametrize('mode', ['external_cloud', 'source_child', 'write_failure', 'no_config'])
def test_rejected_cli_request_has_no_partial_output(tmp_path, native_case, mode):
    source, _config, run = native_case
    before = tree_bytes(source)
    output = source / 'child' if mode == 'source_child' else tmp_path / 'review'
    cloud = tmp_path / 'external' if mode == 'external_cloud' else (
        'map.pgm/clouds' if mode == 'write_failure' else 'clouds')
    result = run(output, cloud, include_config=mode != 'no_config')
    assert result.returncode != 0
    assert not output.exists()
    assert not list(tmp_path.glob('.review.staging-*'))
    assert not (tmp_path / 'external').exists()
    assert tree_bytes(source) == before
