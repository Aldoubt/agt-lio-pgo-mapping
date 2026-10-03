"""Synthetic geometry only; these IDs never label a real greenhouse map."""
import copy
import csv
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_greenhouse_annotation.topology import (
    align_backend, label_pose, labels_for_package, load_map_package, load_topology,
    new_topology, project_to_row, save_topology, scene_correspondence, sha256_file,
    transform_pose)
from agt_greenhouse_annotation.topology import benchmark_scenes
from agt_greenhouse_annotation.validator import validate_topology


def make_package(root, backend='point_lio', offset=(0, 0), timestamps=(100., 101., 102.)):
    root.mkdir()
    (root/'patches').mkdir()
    (root/'map.pcd').write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n0 0 0\n2 0 0\n4 0 0\n')
    metadata = dict(frames=dict(map='test_map'), mapping_backend=dict(id=backend, source_commit='synthetic_commit'),
                    source=dict(rosbag=str(root.parent/'synthetic_bag')))
    (root/'metadata.yaml').write_text(yaml.safe_dump(metadata))
    bag = root.parent/'synthetic_bag'
    bag.mkdir(exist_ok=True)
    (bag/'metadata.yaml').write_text('synthetic test bag only\n')
    poses = []
    for i, timestamp in enumerate(timestamps):
        poses.append(f'{i}.pcd {timestamp} {2*i+offset[0]} {offset[1]} 0 1 0 0 0')
        (root/'patches'/f'{i}.pcd').write_text((root/'map.pcd').read_text())
    (root/'poses_timed.txt').write_text('\n'.join(poses)+'\n')
    files = {str(p.relative_to(root)): sha256_file(p) for p in root.rglob('*') if p.is_file()}
    (root/'manifest.yaml').write_text(yaml.safe_dump(dict(schema_version=1, files=files)))
    return load_map_package(root)


@pytest.fixture
def package(tmp_path):
    return make_package(tmp_path/'map_package')


@pytest.fixture
def topology(package):
    topo = new_topology(package)
    topo['rows'] = [dict(id='SYNTHETIC_R1', centerline=[[0, 0], [10, 0]],
                         nominal_width_m=2., direction='bidirectional', confidence='confirmed', notes='synthetic')]
    topo['headlands'] = [dict(id='SYNTHETIC_H1', polygon=[[10, -2], [12, -2], [12, 2], [10, 2]], confidence='confirmed')]
    return topo


def test_row_centerline_projection_and_along_row_s(topology):
    projection = project_to_row(topology['rows'][0], 3, .5, 0)
    assert projection['along_row_s_m'] == pytest.approx(3)
    assert projection['projection_xy'] == pytest.approx([3, 0])
    assert projection['distance_m'] == pytest.approx(.5)


def test_signed_lateral_d_and_heading(topology):
    row = topology['rows'][0]
    assert project_to_row(row, 3, .5, math.pi/4)['lateral_d_m'] == pytest.approx(.5)
    assert project_to_row(row, 3, -.5, 0)['lateral_d_m'] == pytest.approx(-.5)
    assert project_to_row(row, 3, 0, math.pi/4)['relative_heading_deg'] == pytest.approx(45)
    bent = dict(id='SYNTHETIC_BEND', centerline=[[0, 0], [2, 0], [2, 3]])
    projection = project_to_row(bent, 2.4, 2., math.pi/2)
    assert projection['along_row_s_m'] == pytest.approx(4)
    assert projection['lateral_d_m'] == pytest.approx(-.4)
    assert projection['relative_heading_deg'] == pytest.approx(0)


def test_headland_polygon_assignment(topology):
    label = label_pose(topology, 11, 0, 0)
    assert label['zone_type'] == 'HEADLAND'
    assert label['headland_id'] == 'SYNTHETIC_H1'


def test_unknown_assignment_respects_max_and_corridor(topology):
    assert label_pose(topology, 3, 2, 0)['physical_row_id'] == 'UNKNOWN'
    # Beyond the endpoint, small d alone does not imply belonging to the row.
    assert label_pose(topology, -3, 0, 0)['zone_type'] == 'UNKNOWN'
    topology['assignment']['max_row_assignment_distance_m'] = .2
    assert label_pose(topology, 3, .3, 0)['zone_type'] == 'UNKNOWN'
    assert label_pose(topology, 3, .1, 0)['physical_row_id'] == 'SYNTHETIC_R1'


def test_unconfirmed_manual_geometry_never_assigns_physical_id(topology):
    topology['rows'][0]['confidence'] = 'unconfirmed'
    assert label_pose(topology, 3, 0, 0)['physical_row_id'] == 'UNKNOWN'
    topology['headlands'][0]['confidence'] = 'unconfirmed'
    assert label_pose(topology, 11, 0, 0)['zone_type'] == 'UNKNOWN'


def test_overlap_tie_stays_unknown(topology):
    second = copy.deepcopy(topology['rows'][0])
    second.update(id='SYNTHETIC_R2', centerline=[[0, .1], [10, .1]])
    topology['rows'].append(second)
    assert label_pose(topology, 3, .05, 0)['physical_row_id'] == 'UNKNOWN'
    result = validate_topology(topology)
    assert not result['valid']
    assert any('overlap' in e for e in result['errors'])


def test_save_load_roundtrip_and_provenance(topology, package, tmp_path):
    output = tmp_path/'annotation'/'greenhouse_topology.yaml'
    manifest = save_topology(topology, output, package)
    assert load_topology(output) == topology
    assert manifest['annotation_file_sha256'] == sha256_file(output)
    assert manifest['source']['backend_commit'] == 'synthetic_commit'
    assert manifest['annotation_tool_source_sha256']
    assert output.with_suffix('.geojson').exists()
    assert len(list(csv.DictReader((output.parent/'keyframe_topology_labels.csv').open()))) == 3
    result = validate_topology(topology, package, output, output.parent/'keyframe_topology_labels.csv', True)
    assert result['valid'], result
    output.write_text(output.read_text()+'\n# tampered\n')
    assert any('provenance' in e for e in validate_topology(topology, package, output)['errors'])


def test_frozen_annotation_is_immutable(topology, package, tmp_path):
    topology['annotation'].update(status='frozen', manual_review_confirmed=True)
    output = tmp_path/'annotation'/'greenhouse_topology.yaml'
    save_topology(topology, output, package)
    save_topology(topology, output, package)  # identical export is permitted
    topology['rows'][0]['notes'] = 'changed'
    with pytest.raises(ValueError, match='immutable'):
        save_topology(topology, output, package)


def test_frozen_requires_manual_review_and_geometry(package):
    topology = new_topology(package)
    topology['annotation'].update(status='frozen', manual_review_confirmed=False)
    result = validate_topology(topology, package)
    assert any('review' in e for e in result['errors'])
    assert any('empty' in e for e in result['errors'])


def test_source_manifest_frame_and_nan_validation(topology, package):
    topology['source']['map_package_manifest_sha256'] = 'wrong'
    topology['source']['map_frame'] = 'wrong_frame'
    topology['rows'][0]['centerline'][0][0] = float('nan')
    result = validate_topology(topology, package)
    assert not result['valid']
    assert any('manifest' in e for e in result['errors'])
    assert any('frame' in e for e in result['errors'])
    assert any('NaN' in e for e in result['errors'])


def test_scene_timestamp_and_duplicate_ids(topology, package):
    scene = dict(scene_id='SYNTHETIC_SCENE', scene_type='ROW_MIDDLE', bag_timestamp=1000, keyframe=0)
    topology['scenes'] = [scene, copy.deepcopy(scene)]
    result = validate_topology(topology, package)
    assert any('scene IDs' in e for e in result['errors'])
    assert any('timestamp' in e for e in result['errors'])


def test_backend_correspondence_rigid_transform_and_candidate_mapping(topology, package, tmp_path):
    other = make_package(tmp_path/'other', 'fast_livo2_lio', offset=(20, 7))
    transform = align_backend(package, other)
    topology['backend_transforms'][other.backend_id] = transform
    assert transform['residual_xy_m']['max'] < 1e-8
    assert transform_pose(transform['matrix'], 22, 7, 0) == pytest.approx((2, 0, 0))
    labels = labels_for_package(topology, other)
    assert labels[1]['physical_row_id'] == 'SYNTHETIC_R1'
    assert labels[1]['along_row_s_m'] == pytest.approx(2)
    assert validate_topology(topology, package)['valid']
    topology['scenes'] = [dict(scene_id='SYNTHETIC_SCENE', scene_type='ROW_MIDDLE', bag_timestamp=101, keyframe=1)]
    correspondence = scene_correspondence(topology, package, other)
    assert correspondence[0]['pointlio_keyframe'] == 1
    assert correspondence[0]['fastlivo2_keyframe'] == 1
    assert correspondence[0]['fastlivo2_dt'] == 0


def test_other_backend_requires_explicit_transform(topology, tmp_path):
    other = make_package(tmp_path/'other', 'fast_livo2_lio')
    with pytest.raises(ValueError, match='no .*transform'):
        labels_for_package(topology, other)


def test_headland_self_intersection_rejected(topology, package):
    topology['headlands'][0]['polygon'] = [[0, 0], [1, 1], [1, 0], [0, 1]]
    assert any('polygon' in e for e in validate_topology(topology, package)['errors'])


def test_labels_csv_coverage_is_checked(topology, package, tmp_path):
    labels = tmp_path/'bad.csv'
    labels.write_text('keyframe,zone_type,physical_row_id,headland_id\n0,ROW,SYNTHETIC_R1,UNKNOWN\n')
    result = validate_topology(topology, package, labels_path=labels)
    assert any('coverage' in e for e in result['errors'])


def test_manual_benchmark_scene_export_and_no_auto_scene_creation(tmp_path):
    package = make_package(tmp_path/'seven_frame_map', timestamps=tuple(100.+i for i in range(7)))
    topology = new_topology(package)
    topology['scenes'] = [
        dict(scene_id='SYNTHETIC_MIDDLE', scene_type='ROW_MIDDLE', bag_timestamp=103., keyframe=3),
        dict(scene_id='SYNTHETIC_EDGE', scene_type='ROW_ENTRY', bag_timestamp=100., keyframe=0),
        dict(scene_id='SYNTHETIC_OTHER', scene_type='OTHER', bag_timestamp=103., keyframe=3)]
    exported = benchmark_scenes(topology, package)
    assert len(exported['scenes']) == 1
    assert exported['scenes'][0]['id'] == 'SYNTHETIC_MIDDLE'
    assert exported['scenes'][0]['type'] == 'row_middle'
    assert exported['scenes'][0]['keyframe'] == 3
    assert len(exported['excluded_manual_scenes']) == 2
    topology['scenes'] = []
    assert benchmark_scenes(topology, package)['scenes'] == []
    assert benchmark_scenes(topology, package)['status'] == 'NO_RUNNABLE_MANUAL_SCENES'


def test_scene_timestamp_declared_keyframe_mismatch(topology, package):
    topology['scenes'] = [dict(scene_id='SYNTHETIC_SCENE', scene_type='ROW_MIDDLE', bag_timestamp=102., keyframe=0)]
    assert any('declared keyframe mismatch' in e for e in validate_topology(topology, package)['errors'])


def test_freeze_cli_requires_explicit_manual_confirmation(topology, package, tmp_path):
    from agt_greenhouse_annotation.freeze import main
    draft = tmp_path/'draft'/'greenhouse_topology.yaml'
    frozen = tmp_path/'frozen'/'greenhouse_topology.yaml'
    save_topology(topology, draft, package)
    with pytest.raises(SystemExit):
        main(['--topology', str(draft), '--output', str(frozen)])
    assert main(['--topology', str(draft), '--output', str(frozen), '--confirm-manual-review']) == 0
    saved = load_topology(frozen)
    assert saved['annotation']['status'] == 'frozen'
    assert saved['annotation']['manual_review_confirmed'] is True


def test_source_changed_metadata_detected_on_load(package):
    metadata = package.path/'metadata.yaml'
    metadata.write_text(metadata.read_text()+'\n# user change\n')
    with pytest.raises(ValueError, match='checksum mismatch'):
        load_map_package(package.path)


def test_new_annotation_cannot_overwrite_frozen_directory_sidecars(topology, package, tmp_path):
    topology['annotation'].update(status='frozen', manual_review_confirmed=True)
    frozen = tmp_path/'version'/'greenhouse_topology.yaml'
    save_topology(topology, frozen, package)
    labels_hash = sha256_file(frozen.parent/'keyframe_topology_labels.csv')
    new_draft = copy.deepcopy(topology)
    new_draft['annotation'].update(status='draft', manual_review_confirmed=False)
    with pytest.raises(ValueError, match='new directory'):
        save_topology(new_draft, frozen.parent/'another_topology.yaml', package)
    assert sha256_file(frozen.parent/'keyframe_topology_labels.csv') == labels_hash


def test_validator_reports_bad_collection_types_without_crashing(topology, package):
    topology['rows'] = [None]
    topology['headlands'] = 'invalid'
    topology['scenes'] = {'not': 'a list'}
    topology['backend_transforms'] = {'bad_backend': {'matrix': [[1], [2, 3]]}}
    result = validate_topology(topology, package)
    assert not result['valid']
    assert any('list of mappings' in e for e in result['errors'])
    assert any('matrix' in e for e in result['errors'])
