"""Workbench structure review must remain distinct from crop-row evidence."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import pytest
import yaml

from agt_greenhouse_annotation.coverage_export import (
    export_coverage_structure, freeze_and_export_coverage, reviewed_corridor, structure_document,
)
from agt_greenhouse_annotation.structure_artifacts import (
    AisleProposal, GlobalRowProposal, decision_for_proposal, frozen_proposal_document,
)
from agt_greenhouse_annotation.topology import new_topology
from agt_coverage_authoring.io import load_request, load_structure
from agt_coverage_authoring.models import CoverageError
from test_topology import make_package


@pytest.fixture
def inputs(tmp_path):
    package = make_package(tmp_path / 'source')
    topology = new_topology(package)
    topology['coverage_boundary'] = {'id': 'GREENHOUSE', 'confidence': 'confirmed',
        'polygon': [[-1., -1.], [9., -1.], [9., 4.], [-1., 4.]]}
    topology['headlands'] = [{'id': 'HEADLAND', 'confidence': 'confirmed',
        'polygon': [[4.5, -.5], [8., -.5], [8., 3.], [4.5, 3.]]},
        {'id': 'ENTRY_HEADLAND', 'confidence': 'confirmed',
         'polygon': [[-.8, -.5], [.5, -.5], [.5, 3.], [-.8, 3.]]}]
    rows = [decision_for_proposal(GlobalRowProposal(f'AUTO-R{i}', (1., 0.), y,
        ((0., y), (5., y)), .2, .9, .9, {}), 'accepted', physical_row_id=f'CROP{i}',
        direction='bidirectional') for i, y in enumerate((0., 2.))]
    line = ((0., 1.), (5., 1.))
    aisle = AisleProposal('AUTO-A1', 'AUTO-R0', 'AUTO-R1', 'INTERIOR', line, (), 1., .45,
        {'coverage_review': reviewed_corridor(line, 1., 'along', confirmed=True)},
        decision='accepted', physical_aisle_id='AISLE1')
    Image.fromarray(np.full((60, 100), 254, dtype=np.uint8)).save(tmp_path / 'map.pgm')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': .1,
        'origin': [-1., -1., 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}))
    return package, topology, rows, [aisle], map_yaml


def export_fixture(inputs, tmp_path, **options):
    package, topology, rows, aisles, map_yaml = inputs
    output = tmp_path / 'review' / 'coverage.geojson'
    freeze_and_export_coverage(topology, rows, aisles, output.parent / 'annotation' / 'topology.yaml',
        map_yaml, output, package=package, map_from_reference=[0., 0., 0.],
        transform_review_confirmed=True, **options)
    return output


def test_crop_reference_cannot_be_coverage_even_after_manual_reference_review(inputs):
    _, topology, rows, _, _ = inputs
    candidate = frozen_proposal_document(topology, rows, [])
    assert candidate['rows'][0]['proposal_source']['semantic_role'] == 'crop_row_reference'
    with pytest.raises(ValueError, match='crop-row references cannot be drive paths'):
        structure_document(candidate, map_from_reference=[0., 0., 0.], transform_review_confirmed=True)


def test_only_explicit_aisle_review_exports_compatible_coverage_and_source_binding(inputs, tmp_path):
    result = export_fixture(inputs, tmp_path, headland_ids=['HEADLAND'])
    doc = json.loads(result.read_text())
    assert [f['properties']['feature_type'] for f in doc['features']] == ['field_boundary', 'aisle', 'headland']
    assert 'CROP0' not in [f['properties']['id'] for f in doc['features']]
    assert doc['source']['annotation_sha256'].startswith('sha256:')
    assert doc['source']['map_yaml_sha256'].startswith('sha256:')
    assert doc['source']['map_image_sha256'].startswith('sha256:')
    boundary, rows, _, work_area, _, headlands = load_structure(result)
    assert len(rows) == 1 and rows[0].id == 'AISLE1'
    assert rows[0].allowed_directions == ('along',)
    assert work_area.area == pytest.approx(5.)
    assert len(headlands) == 1 and boundary.covers(headlands[0])


def test_export_is_loadable_by_existing_planning_request_and_detects_map_changes(inputs, tmp_path):
    result = export_fixture(inputs, tmp_path)
    profile = Path(__file__).resolve().parents[3] / 'planning' / 'agt_coverage_authoring' / 'examples' / 'greenhouse_ackermann.yaml'
    # test file is under repo/tools/package/test; repository is parents[3].
    request = tmp_path / 'request.yaml'
    request.write_text(yaml.safe_dump({'map_yaml': str(inputs[-1]), 'structure': str(result),
        'robot_profile': str(profile), 'mode': 'area', 'operation_width_m': .5,
        'backend': 'reference', 'start': [1., 1., 0.]}))
    context = load_request(request)
    assert context.work_area.area == pytest.approx(5.) and len(context.rows) == 1
    inputs[-1].write_text(inputs[-1].read_text() + '\n# map changed\n')
    with pytest.raises(CoverageError) as error:
        load_request(request)
    assert error.value.code == 'STALE'


def test_nonmap_zero_transform_requires_explicit_review_and_rotates_all_geometry(inputs):
    _, topology, rows, aisles, _ = inputs
    candidate = frozen_proposal_document(topology, rows, aisles)
    with pytest.raises(ValueError, match='explicit reviewed map transform'):
        structure_document(candidate)
    with pytest.raises(ValueError, match='explicit manual review'):
        structure_document(candidate, map_from_reference=[0., 0., 0.])
    doc = structure_document(candidate, map_from_reference=[10., 20., np.pi / 2],
                             transform_review_confirmed=True, headland_ids=['HEADLAND'])
    assert np.asarray(doc['features'][1]['geometry']['coordinates'])[0] == pytest.approx([9., 20.])
    assert doc['source']['original_frame'] == 'test_map'
    assert candidate['source']['map_frame'] == 'test_map'


def test_stale_corridor_and_unreviewed_aisle_never_freeze(inputs, tmp_path):
    package, topology, rows, aisles, map_yaml = inputs
    bad = replace(aisles[0], centerline_xy=((0., 1.1), (5., 1.1)))
    with pytest.raises(ValueError, match='stale'):
        freeze_and_export_coverage(topology, rows, [bad], tmp_path / 'bad' / 'annotation' / 'topology.yaml',
            map_yaml, tmp_path / 'bad' / 'coverage.geojson', package=package,
            map_from_reference=[0., 0., 0.], transform_review_confirmed=True)
    assert not (tmp_path / 'bad').exists()
    candidate = frozen_proposal_document(topology, rows, [replace(aisles[0], diagnostics={})])
    with pytest.raises(ValueError, match='review traversable corridor'):
        structure_document(candidate, map_from_reference=[0., 0., 0.], transform_review_confirmed=True)


def test_immutable_export_and_annotation_tamper_rejected(inputs, tmp_path):
    output = export_fixture(inputs, tmp_path)
    annotation = output.parent / 'annotation' / 'topology.yaml'
    original = output.read_bytes()
    with pytest.raises(FileExistsError):
        export_coverage_structure(annotation, inputs[-1], output)
    assert output.read_bytes() == original
    annotation.write_text(annotation.read_text() + '\n# changed\n')
    with pytest.raises(ValueError, match='hash/provenance'):
        export_coverage_structure(annotation, inputs[-1], tmp_path / 'tampered.geojson',
                                  map_from_reference=[0., 0., 0.], transform_review_confirmed=True)


def test_manual_reference_aisle_can_be_reviewed_without_invented_crop_ids(inputs, tmp_path):
    package, topology, _, aisles, map_yaml = inputs
    manual = replace(aisles[0], aisle_kind='REFERENCE', left_row_auto_id='', right_row_auto_id='')
    output = tmp_path / 'manual' / 'coverage.geojson'
    freeze_and_export_coverage(topology, [], [manual], output.parent / 'annotation' / 'topology.yaml',
        map_yaml, output, package=package, map_from_reference=[0., 0., 0.], transform_review_confirmed=True)
    frozen = yaml.safe_load((output.parent / 'annotation' / 'topology.yaml').read_text())
    assert not frozen['rows'] and len(load_structure(output)[1]) == 1


@pytest.mark.parametrize('changed_input', ['annotation', 'map_yaml', 'map_image'])
def test_input_changed_during_actual_export_is_rejected_before_publication(inputs, tmp_path, monkeypatch, changed_input):
    from agt_greenhouse_annotation import coverage_export
    output = export_fixture(inputs, tmp_path)
    annotation = output.parent / 'annotation' / 'topology.yaml'
    target = tmp_path / 'new-coverage.geojson'
    real_validation = coverage_export._validate_document

    def mutate_after_real_geometry_validation(document):
        real_validation(document)
        if changed_input == 'annotation':
            annotation.write_text(annotation.read_text() + '\n# concurrent source edit\n')
        elif changed_input == 'map_yaml':
            inputs[-1].write_text(inputs[-1].read_text() + '\n# concurrent map edit\n')
        else:
            image_path = tmp_path / 'map.pgm'
            image_path.write_bytes(image_path.read_bytes() + b'\n')

    monkeypatch.setattr(coverage_export, '_validate_document', mutate_after_real_geometry_validation)
    with pytest.raises(CoverageError) as error:
        export_coverage_structure(annotation, inputs[-1], target,
            map_from_reference=[0., 0., 0.], transform_review_confirmed=True)
    assert error.value.code == 'STALE'
    assert not target.exists()


def test_outputs_cannot_enter_source_or_frozen_directories_or_follow_symlinks(inputs, tmp_path):
    output = export_fixture(inputs, tmp_path)
    annotation = output.parent / 'annotation' / 'topology.yaml'
    for target in (inputs[0].path / 'coverage.geojson', annotation.parent / 'another.geojson',
                   annotation.parent / 'nested' / 'coverage.geojson'):
        with pytest.raises(ValueError, match='outside'):
            export_coverage_structure(annotation, inputs[-1], target,
                map_from_reference=[0., 0., 0.], transform_review_confirmed=True)
        assert not target.exists()
    symlink = tmp_path / 'output-link.geojson'
    symlink.symlink_to(tmp_path / 'missing-target.geojson')
    with pytest.raises(FileExistsError, match='symlink'):
        export_coverage_structure(annotation, inputs[-1], symlink,
            map_from_reference=[0., 0., 0.], transform_review_confirmed=True)
    assert symlink.is_symlink() and not symlink.exists()


def test_actual_reference_planner_requires_reviewed_headlands_for_end_footprints(inputs, tmp_path):
    from agt_coverage_authoring.planner import plan_coverage
    output = export_fixture(inputs, tmp_path)
    annotation = output.parent / 'annotation' / 'topology.yaml'
    profile = Path(__file__).resolve().parents[3] / 'planning' / 'agt_coverage_authoring' / 'examples' / 'greenhouse_ackermann.yaml'
    request = tmp_path / 'row-request.yaml'
    parameters = {'map_yaml': str(inputs[-1]), 'structure': str(output),
                  'robot_profile': str(profile), 'mode': 'rows', 'backend': 'reference',
                  'start': [0., 1., 0.], 'end': [5., 1., 0.]}
    request.write_text(yaml.safe_dump(parameters))
    incomplete = plan_coverage(load_request(request))
    assert not incomplete.complete
    assert incomplete.report['metrics']['target_rows'] == 1
    assert any(item.get('code') == 'AISLE_TOO_NARROW' for item in incomplete.missing)

    complete_structure = export_coverage_structure(annotation, inputs[-1], tmp_path / 'with-headlands.geojson',
        map_from_reference=[0., 0., 0.], transform_review_confirmed=True,
        headland_ids=['ENTRY_HEADLAND', 'HEADLAND'])
    parameters['structure'] = str(complete_structure)
    request.write_text(yaml.safe_dump(parameters))
    complete = plan_coverage(load_request(request))
    assert complete.complete, complete.report
    assert complete.report['metrics']['covered_rows'] == 1
    assert complete.report['validation']['geometric_valid']
    assert complete.report['validation']['dynamic_status'] == 'UNKNOWN'


def test_headless_topology_and_coverage_imports_do_not_require_qt():
    code = """
import sys
class RejectQt:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('PyQt5', 'PySide', 'matplotlib')):
            raise ImportError('GUI import is forbidden in this headless test')
sys.meta_path.insert(0, RejectQt())
import agt_greenhouse_annotation.topology
import agt_greenhouse_annotation.coverage_export
"""
    subprocess.run([sys.executable, '-c', code], check=True, capture_output=True, text=True)
