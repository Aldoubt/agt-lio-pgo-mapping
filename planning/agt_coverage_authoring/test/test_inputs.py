"""Actual file input, source-snapshot and semantic review boundaries."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from agt_coverage_authoring import io
from agt_coverage_authoring.models import CoverageError


def write(path, value):
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    return path


def feature(identity, kind, geometry, **properties):
    return {'type': 'Feature', 'properties': {'id': identity, 'feature_type': kind,
            'review_status': 'CONFIRMED', **properties}, 'geometry': geometry}


def polygon(x0, y0, x1, y1):
    return {'type': 'Polygon', 'coordinates': [[[x0, y0], [x1, y0], [x1, y1],
                                                [x0, y1], [x0, y0]]]}


@pytest.fixture
def input_bundle(tmp_path):
    image = tmp_path / 'map.pgm'
    Image.fromarray(np.full((100, 100), 254, dtype=np.uint8)).save(image)
    map_doc = {'image': image.name, 'resolution': .1, 'origin': [0, 0, 0],
               'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
    profile = {'platform': {'name': 'mkmini_preview', 'kinematics': 'ackermann',
               'geometry': {'wheelbase': .6, 'min_turning_radius': 1.5,
                            'reference_point_assumption': 'geometric_center',
                            'navigation_footprint': [[-.4, -.3], [.4, -.3], [.4, .3], [-.4, .3]]},
               'limits': {'max_forward_velocity': .2, 'max_reverse_velocity': .1}}}
    structure = {'type': 'FeatureCollection', 'frame_id': 'map',
                 'review': {'status': 'CONFIRMED'},
                 'features': [feature('field', 'field_boundary', polygon(0, 0, 10, 10)),
                              feature('row', 'aisle', {'type': 'LineString', 'coordinates': [[2, 2], [8, 2]]},
                                      corridor=polygon(1, 1, 9, 3)),
                              feature('headland', 'headland', polygon(.5, .5, 1.5, 9.5))]}
    request = {'map_yaml': 'map.yaml', 'robot_profile': 'robot.yaml', 'structure': 'structure.geojson',
               'mode': 'rows', 'start': [2, 2, 0], 'end': [8, 2, 0], 'backend': 'reference'}
    paths = {'map': write(tmp_path / 'map.yaml', map_doc),
             'profile': write(tmp_path / 'robot.yaml', profile),
             'structure': write(tmp_path / 'structure.geojson', structure),
             'request': write(tmp_path / 'request.yaml', request), 'image': image}
    return paths, {'map': map_doc, 'profile': profile, 'structure': structure, 'request': request}


def test_request_loads_reviewed_rows_headlands_and_immutable_raster(input_bundle):
    paths, _ = input_bundle
    context = io.load_request(paths['request'])
    assert len(context.rows) == len(context.headlands) == 1
    assert context.work_area.area == pytest.approx(16)
    assert context.robot.rear_axle_to_base == (.3, 0)
    assert not context.robot.reference_verified
    assert not context.grid.data.flags.writeable
    assert set(context.input_files) == {str(path) for path in paths.values()}
    io.assert_current(context.input_files)


@pytest.mark.parametrize('change,code', [
    ({'mode': 'unsupported'}, 'INVALID_MODE'),
    ({'mode': 'area'}, 'AREA_INPUT_REQUIRED'),
    ({'operation_width_m': 0}, 'INVALID_INPUT'),
    ({'clearance_margin_m': -.1}, 'INVALID_INPUT'),
    ({'allow_reverse': 'yes'}, 'INVALID_INPUT'),
    ({'start': [1, 2]}, 'INVALID_INPUT'),
    ({'backend_options': []}, 'INVALID_INPUT'),
    ({'map_binding': []}, 'INVALID_INPUT'),
    ({'planning_timeout_s': 1e6}, 'INVALID_TIMEOUT'),
    ({'sample_step_m': 1e-12}, 'INVALID_SAMPLE_STEP'),
    ({'map_yaml': None}, 'INVALID_INPUT'),
    ({'unknown_setting': 1}, 'UNKNOWN_OPTION'),
])
def test_request_rejects_invalid_settings(input_bundle, change, code):
    paths, docs = input_bundle
    write(paths['request'], dict(docs['request'], **change))
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == code


def test_area_mode_preserves_work_target_and_has_no_implicit_width(input_bundle):
    paths, docs = input_bundle
    write(paths['request'], dict(docs['request'], mode='area', operation_width_m=.7))
    context = io.load_request(paths['request'])
    assert context.operation_width == .7
    assert context.work_area.area == 16


@pytest.mark.parametrize('text', ['value: 1\nvalue: 2\n', 'value: .nan\n', 'value: &a [*a]\n'])
def test_document_rejects_duplicate_nonfinite_and_recursive_values(tmp_path, text):
    path = tmp_path / 'bad.yaml'
    path.write_text(text)
    with pytest.raises(CoverageError):
        io.read_document(path)


def test_missing_and_malformed_document_have_domain_errors(tmp_path):
    with pytest.raises(CoverageError):
        io.read_document(tmp_path / 'missing')
    path = tmp_path / 'invalid.yaml'
    path.write_text('key: [\n')
    with pytest.raises(CoverageError):
        io.read_document(path)


def test_document_budget_override_does_not_relax_configuration_defaults(tmp_path):
    path = tmp_path / 'plan.json'
    # Serialized plan-like objects can use a larger explicit budget. A caller
    # cannot accidentally change the defaults used by subsequent config reads.
    path.write_text(json.dumps({'points': list(range(io.MAX_DOCUMENT_NODES))}))
    with pytest.raises(CoverageError) as error:
        io.read_document(path)
    assert error.value.code == 'INPUT_TOO_COMPLEX'
    assert len(io.read_document(path, max_nodes=io.MAX_DOCUMENT_NODES + 2)['points']) == io.MAX_DOCUMENT_NODES
    with pytest.raises(CoverageError):
        io.read_document(path)
    with pytest.raises(CoverageError) as error:
        io.read_document(path, max_bytes=8, max_nodes=io.MAX_DOCUMENT_NODES + 2)
    assert error.value.code == 'INPUT_TOO_LARGE'
    for kwargs in ({'max_bytes': True}, {'max_nodes': 0}):
        with pytest.raises(CoverageError):
            io.read_document(path, **kwargs)


def test_nan_inside_geometry_is_rejected_before_shapely(input_bundle):
    paths, docs = input_bundle
    document = deepcopy(docs['structure'])
    document['features'][0]['geometry']['coordinates'][0].insert(2, [float('nan'), 4])
    paths['structure'].write_text(json.dumps(document))
    with pytest.raises(CoverageError):
        io.load_request(paths['request'])


@pytest.mark.parametrize('mutate,code', [
    (lambda doc: doc['review'].update(status='DRAFT'), 'REVIEW_REQUIRED'),
    (lambda doc: doc['features'][1]['properties'].update(review_status='DRAFT'), 'REVIEW_REQUIRED'),
    (lambda doc: doc['features'][1]['properties'].update(id='field'), 'STRUCTURE_ID'),
    (lambda doc: doc['features'][1]['properties'].update(allowed_direction='reverse'), 'INVALID_DIRECTION'),
    (lambda doc: doc['features'][1]['geometry'].update(coordinates=[[2, 2], [2, 2]]), 'INVALID_GEOMETRY'),
    (lambda doc: doc['features'][1]['properties'].update(corridor=polygon(-1, 1, 9, 3)), 'CORRIDOR_OUTSIDE_BOUNDARY'),
    (lambda doc: doc['features'][2].update(geometry=polygon(-1, 0, 1, 2)), 'HEADLAND_OUTSIDE_BOUNDARY'),
])
def test_structure_preserves_review_and_containment(input_bundle, mutate, code):
    paths, docs = input_bundle
    document = deepcopy(docs['structure'])
    mutate(document)
    write(paths['structure'], document)
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == code


def test_source_annotation_and_map_hashes_are_checked(input_bundle, tmp_path):
    paths, docs = input_bundle
    annotation = write(tmp_path / 'review.json', {'reviewed': True})
    document = deepcopy(docs['structure'])
    document['source'] = {'annotation_path': annotation.name, 'annotation_sha256': io.file_digest(annotation),
                          'map_yaml_sha256': io.file_digest(paths['map']),
                          'map_image_sha256': io.file_digest(paths['image'])}
    write(paths['structure'], document)
    context = io.load_request(paths['request'])
    assert str(annotation) in context.input_files
    annotation.write_text('{"reviewed": false}\n')
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == 'STALE'


def test_map_source_mismatch_is_not_silently_rebound(input_bundle):
    paths, docs = input_bundle
    document = dict(docs['structure'], source={'map_image_sha256': 'sha256:' + 'a' * 64})
    write(paths['structure'], document)
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == 'STALE'


def test_request_change_between_read_and_binding_is_detected(input_bundle, monkeypatch):
    paths, _ = input_bundle
    original = io.read_document
    def read_then_change(path):
        value = original(path)
        if Path(path) == paths['request']:
            paths['request'].write_text(paths['request'].read_text() + '\n# concurrent edit\n')
        return value
    monkeypatch.setattr(io, 'read_document', read_then_change)
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == 'STALE'


def test_image_change_during_decode_is_detected(input_bundle, monkeypatch):
    paths, _ = input_bundle
    original = io.Image.open
    def change_then_open(path, *args, **kwargs):
        if Path(path) == paths['image']:
            Image.fromarray(np.zeros((100, 100), dtype=np.uint8)).save(path)
        return original(path, *args, **kwargs)
    monkeypatch.setattr(io.Image, 'open', change_then_open)
    with pytest.raises(CoverageError) as error:
        io.load_request(paths['request'])
    assert error.value.code == 'STALE'


def test_snapshot_detects_deleted_file(input_bundle):
    paths, _ = input_bundle
    context = io.load_request(paths['request'])
    paths['image'].unlink()
    with pytest.raises(CoverageError) as error:
        io.assert_current(context.input_files)
    assert error.value.code == 'STALE'


def test_profile_contradiction_and_missing_reference_are_rejected(input_bundle):
    paths, docs = input_bundle
    profile = deepcopy(docs['profile'])
    profile['platform']['kinematics'] = 'differential'
    write(paths['profile'], profile)
    with pytest.raises(CoverageError) as error:
        io.load_robot_profile(paths['profile'])
    assert error.value.code == 'PROFILE_UNSUPPORTED'
    profile['platform']['kinematics'] = 'ackermann'
    profile['platform']['geometry'].pop('reference_point_assumption')
    write(paths['profile'], profile)
    with pytest.raises(CoverageError) as error:
        io.load_robot_profile(paths['profile'])
    assert error.value.code == 'REFERENCE_POINT_UNKNOWN'


def test_missing_speed_cannot_inherit_verified_defaults(input_bundle):
    paths, docs = input_bundle
    profile = deepcopy(docs['profile'])
    profile['platform']['limits'] = {'limits_verified_on_vehicle': True}
    write(paths['profile'], profile)
    assert not io.load_robot_profile(paths['profile']).limits_verified


def test_trinary_pixels_y_flip_alpha_and_yaw(tmp_path):
    image = tmp_path / 'map.png'
    pixels = np.array([[[0, 0, 0, 255], [254, 254, 254, 255]],
                       [[205, 205, 205, 255], [254, 254, 254, 0]]], dtype=np.uint8)
    Image.fromarray(pixels).save(image)
    map_yaml = write(tmp_path / 'map.yaml', {'image': image.name, 'resolution': .1,
                    'origin': [1, 2, math.pi / 2], 'occupied_thresh': .65, 'free_thresh': .196})
    grid = io.load_grid(map_yaml)
    assert grid.data.tolist() == [[-1, -1], [100, 0]]
    assert grid.origin == (1, 2, math.pi / 2)


@pytest.mark.parametrize('change', [{'negate': True}, {'free_thresh': .8}, {'mode': 'scale'}, {'image': []}])
def test_invalid_map_metadata_is_rejected(input_bundle, change):
    paths, docs = input_bundle
    write(paths['map'], dict(docs['map'], **change))
    with pytest.raises(CoverageError):
        io.load_grid(paths['map'])
