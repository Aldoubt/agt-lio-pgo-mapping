from __future__ import annotations

import csv
from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from agt_map_localization_benchmark.study_workflow import (
    add_project_assets,
    add_manual_query,
    create_manual_query_set,
    create_project,
    create_study,
    create_structure_query_set,
    clone_query_set_as_draft,
    delete_draft_query,
    export_study,
    freeze_query_set,
    set_query_enabled,
    validate_project,
    validate_query_set,
    validate_study,
)
import agt_map_localization_benchmark.study_workflow as workflow
from agt_mapping_artifacts.frontend_package import write_frontend_map_package
from agt_mapping_artifacts.keyframe_blocks import build_keyframe_blocks
from agt_mapping_artifacts.frontend_package import sha256
from agt_greenhouse_annotation.topology import load_map_package, new_topology, save_topology


def _source(root: Path) -> Path:
    records = []
    cloud = np.asarray([[0.0, 0.0, 0.0, 1.0], [0.1, 0.0, 0.1, 2.0]], dtype='<f4')
    for index in range(12):
        records.append({
            'stamp_sec': 100 + index, 'stamp_nanosec': 0,
            'position': [float(index), 0.0, 0.0],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0], 'points_xyzi': cloud.copy(),
        })
    write_frontend_map_package(root / 'source', records, {
        'mapping_backend': {'id': 'fast_livo2_lio', 'project': 'fixture', 'mode': 'lio_only',
                            'source_commit': 'fixture', 'config_sha256': 'a' * 64,
                            'loop_closure': False, 'gps_factor': False,
                            'external_global_correction': False},
        'source': {'kind': 'unit_test'},
        'frames': {'map': 'map', 'body': 'body'},
        'reference': {'source': 'mapping_frontend_odometry', 'same_session': True,
                      'pgo_applied': False, 'optimized': False,
                      'absolute_ground_truth': False},
    })
    return root / 'source'


def _frozen_topology(source: Path, root: Path) -> Path:
    package = load_map_package(source)
    topology = new_topology(package)
    topology['rows'] = [{
        'id': 'row_manual_1', 'centerline': [[2.0, 0.0], [9.0, 0.0]],
        'nominal_width_m': 0.8, 'direction': 'forward', 'confidence': 'confirmed',
    }]
    topology['annotation']['status'] = 'frozen'
    topology['annotation']['manual_review_confirmed'] = True
    path = root / 'annotation' / 'greenhouse_topology.yaml'
    save_topology(topology, path, package)
    return path


@pytest.fixture
def assets(tmp_path: Path):
    root = tmp_path / 'inputs'
    root.mkdir()
    source = _source(root)
    topology = _frozen_topology(source, root)
    blocks = root / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=3, stride=3,
                          row_annotation_sha256=sha256(topology))
    project_root = tmp_path / 'project'
    project_root.mkdir()
    return source, blocks, topology, project_root


def test_manual_query_set_snaps_to_observation_and_freeze_is_immutable(assets):
    source, blocks, topology, project = assets
    draft_path = project / 'draft.yaml'
    draft = create_manual_query_set(
        query_set_id='manual_sparse', map_package=source, block_dir=blocks,
        topology_path=topology,
        locations=[{'query_id': 'Q001', 'scene': 'MIDDLE', 'xy': [4.2, 0.2]}],
        output=draft_path)
    query = draft['queries'][0]
    assert query['resolved_keyframe_id'] == 4
    assert query['snap_distance_m'] == pytest.approx(0.2828427)
    assert query['requested_location']['x_m'] == pytest.approx(4.2)
    frozen_path = project / 'frozen' / 'manual_sparse_r2.yaml'
    frozen = freeze_query_set(draft_path, frozen_path)
    assert frozen['status'] == 'FROZEN'
    assert frozen['revision'] == 2
    assert validate_query_set(frozen_path)['valid']
    with pytest.raises(FileExistsError):
        freeze_query_set(draft_path, frozen_path)


def test_reviewed_row_sampling_and_exact_block_annotation_binding(assets):
    source, blocks, topology, project = assets
    result = create_structure_query_set(
        query_set_id='rows', map_package=source, block_dir=blocks,
        topology_path=topology, output=project / 'rows.yaml')
    queries = result['queries']
    assert [query['scene'] for query in queries] == ['ENTRY', 'MIDDLE', 'MIDDLE', 'MIDDLE', 'EXIT']
    assert [query['row_id'] for query in queries] == ['row_manual_1'] * 5
    assert queries[0]['resolved_keyframe_id'] == 2
    assert queries[-1]['resolved_keyframe_id'] == 9
    assert validate_query_set(project / 'rows.yaml')['valid']


def test_study_project_round_trip_and_staleness_detection(assets):
    source, blocks, topology, project_root = assets
    draft = project_root / 'draft.yaml'
    create_manual_query_set(query_set_id='queries', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'ENTRY', 'xy': [4.0, 0.0]}],
                            output=draft)
    frozen = project_root / 'queries' / 'r2.yaml'
    freeze_query_set(draft, frozen)
    study_dir = project_root / 'studies' / 'study_001'
    study = create_study(study_id='study_001', query_set_path=frozen,
                         output_dir=study_dir, frames=(1, 3, 5), candidate_top_k=5)
    assert study['job']['total'] == 3
    assert validate_study(study_dir / 'study.yaml')['state'] == 'CURRENT'
    project = project_root / 'project.yaml'
    create_project(project_path=project, map_package=source, block_dir=blocks,
                   topology_path=topology)
    add_project_assets(project, query_set_paths=[frozen], study_paths=[study_dir / 'study.yaml'])
    assert validate_project(project)['state'] == 'CURRENT'

    # Source mutation is detected through existing manifest/checksum validators.
    with (source / 'metadata.yaml').open('a', encoding='utf-8') as stream:
        stream.write('\n# changed after project save\n')
    assert validate_project(project)['state'] == 'STALE'


def test_tampered_query_set_hash_is_rejected(assets):
    source, blocks, topology, project = assets
    path = project / 'draft.yaml'
    create_manual_query_set(query_set_id='tamper', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'OTHER', 'xy': [4.0, 0.0]}],
                            output=path)
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    value['queries'][0]['resolved_keyframe_id'] = 7
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    check = validate_query_set(path)
    assert not check['valid']
    assert any('content_sha256 mismatch' in error for error in check['errors'])


def test_draft_query_edits_are_revision_scoped_and_annotation_changes_go_stale(assets):
    source, blocks, topology, project = assets
    draft_path = project / 'queries' / 'draft.yaml'
    draft_path.parent.mkdir(parents=True)
    create_manual_query_set(query_set_id='edit_flow', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'ENTRY', 'xy': [3.0, 0.0]},
                                       {'query_id': 'Q002', 'scene': 'EXIT', 'xy': [8.0, 0.0]}],
                            output=draft_path)
    set_query_enabled(draft_path, 'Q002', False)
    delete_draft_query(draft_path, 'Q002')
    frozen_path = project / 'queries' / 'frozen_r2.yaml'
    freeze_query_set(draft_path, frozen_path)
    with pytest.raises(ValueError, match='immutable'):
        add_manual_query(frozen_path, xy=(5.0, 0.0))

    next_draft = project / 'queries' / 'draft_r3.yaml'
    clone_query_set_as_draft(frozen_path, next_draft)
    add_manual_query(next_draft, xy=(6.0, 0.0), query_id='Q003')
    assert validate_query_set(next_draft)['valid']

    topology.write_text(topology.read_text(encoding='utf-8') + '\n# changed revision\n', encoding='utf-8')
    stale = validate_query_set(frozen_path)
    assert stale['state'] == 'STALE'
    assert any('annotation is STALE' in error for error in stale['errors'])


def test_manual_query_set_may_pin_unreviewed_annotation_without_claiming_row_identity(assets):
    source, _bound_blocks, topology, project = assets
    unbound_blocks = project / 'blocks_unbound'
    build_keyframe_blocks(source, unbound_blocks, block_keyframe_count=3, stride=3)
    value = yaml.safe_load(topology.read_text(encoding='utf-8'))
    value['annotation']['status'] = 'draft'
    value['annotation']['manual_review_confirmed'] = False
    draft_annotation = project / 'annotation_draft.yaml'
    draft_annotation.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    draft_path = project / 'manual_with_draft_context.yaml'
    create_manual_query_set(query_set_id='manual_draft_context', map_package=source,
                            block_dir=unbound_blocks, topology_path=draft_annotation,
                            locations=[{'query_id': 'Q001', 'scene': 'OTHER', 'xy': [4.0, 0.0]}],
                            output=draft_path)
    frozen = project / 'manual_with_draft_context_frozen.yaml'
    freeze_query_set(draft_path, frozen)
    checked = validate_query_set(frozen)
    assert checked['valid']
    assert yaml.safe_load(frozen.read_text(encoding='utf-8'))['queries'][0]['row_id'] == 'UNKNOWN'

    value['annotation']['notes'] = 'operator revised this draft'
    draft_annotation.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    stale = validate_query_set(frozen)
    assert stale['state'] == 'STALE'
    assert any('annotation is STALE' in error for error in stale['errors'])


def test_study_export_preserves_candidate_fields_added_by_evidence_producer(tmp_path, monkeypatch):
    study_dir = tmp_path / 'study'
    evidence_dir = study_dir / 'evidence' / 'query_1'
    evidence_dir.mkdir(parents=True)
    (evidence_dir / 'manifest.yaml').write_text(yaml.safe_dump({
        'evidence': {'candidate_ambiguity': {'candidates': [{
            'rank': 1, 'candidate_keyframe': 305, 'gicp_converged': True,
            'candidate_patch_id': 'patch_305', 'descriptor_ring_distance': 0.42,
            'coarse_pose': {'x': 1.0, 'y': 2.0}, 'bbs_attempted': True,
        }]}},
    }, sort_keys=False), encoding='utf-8')
    study_path = study_dir / 'study.yaml'
    study_path.write_text(yaml.safe_dump({'results': [{
        'query_id': 'KF590', 'query_accumulation_frames': 1,
        'evidence_ref': {'kind': 'filesystem', 'path': 'evidence/query_1'},
    }]}, sort_keys=False), encoding='utf-8')
    monkeypatch.setattr(workflow, 'validate_study', lambda _path: {'valid': True, 'state': 'CURRENT'})

    output_dir = tmp_path / 'report'
    export_study(study_path, output_dir)

    with (output_dir / 'candidates.csv').open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        row = next(reader)
        assert {'candidate_patch_id', 'descriptor_ring_distance', 'coarse_pose', 'bbs_attempted'} <= set(reader.fieldnames)
    assert row['query_id'] == 'KF590'
    assert row['frames'] == '1'
    assert row['candidate_patch_id'] == 'patch_305'
    assert row['descriptor_ring_distance'] == '0.42'
    assert row['coarse_pose'] == '{"x": 1.0, "y": 2.0}'
    assert row['bbs_attempted'] == 'True'


def test_batch_expands_frames_and_summarizes_existing_evidence(assets, monkeypatch):
    source, blocks, topology, project_root = assets
    draft = project_root / 'draft.yaml'
    create_manual_query_set(query_set_id='batch', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'MIDDLE', 'xy': [4.0, 0.0]}],
                            output=draft)
    frozen = project_root / 'queries' / 'r2.yaml'
    freeze_query_set(draft, frozen)
    study_dir = project_root / 'study'
    create_study(study_id='study', query_set_path=frozen, output_dir=study_dir,
                 frames=(1, 3, 5), candidate_top_k=5)
    invocations = []

    def fake_run_query(args):
        invocations.append(args.query_accumulation_frames)
        evidence = study_dir / 'fake_evidence' / str(args.query_accumulation_frames)
        evidence.mkdir(parents=True)
        classification = 'CORRECT' if args.query_accumulation_frames == 1 else 'FALSE_ACCEPT'
        record = {'evidence': {
            'empirical_global_result': {'classification': classification, 'converged': True,
                                        'reference_error': {'xy_m': 0.2}},
            'candidate_ambiguity': {'candidates': [
                {'rank': 1, 'classification': classification, 'gicp_converged': True,
                 'gicp_fitness': 0.05, 'row_id': 'row_manual_2'}]},
        }}
        (evidence / 'manifest.yaml').write_text(yaml.safe_dump(record), encoding='utf-8')
        return evidence

    monkeypatch.setattr(workflow, 'run_query', fake_run_query)
    monkeypatch.setattr(workflow, 'verify_evidence_bundle', lambda *_args, **_kwargs: {'revision_state': 'CURRENT'})
    result = workflow.run_study(study_dir / 'study.yaml', ros_install=project_root)
    assert invocations == [1, 3, 5]
    assert result['status'] == 'SUCCEEDED'
    summary = yaml.safe_load((study_dir / 'summary.yaml').read_text(encoding='utf-8'))
    assert summary['CORRECT'] == 1
    assert summary['FALSE_ACCEPT'] == 2
    assert summary['false_accept_rate'] == pytest.approx(2 / 3)
    assert summary['correct_at_k']['1']['value'] == pytest.approx(1 / 3)
    assert (study_dir / 'summary.csv').is_file()


def test_study_rejects_source_or_block_binding_changed_from_frozen_query_set(assets):
    source, blocks, topology, project_root = assets
    draft = project_root / 'draft.yaml'
    create_manual_query_set(query_set_id='binding', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'MIDDLE', 'xy': [4.0, 0.0]}],
                            output=draft)
    frozen = project_root / 'queries' / 'r2.yaml'
    freeze_query_set(draft, frozen)
    study_dir = project_root / 'binding-study'
    study = create_study(study_id='binding-study', query_set_path=frozen,
                         output_dir=study_dir, frames=(1,), candidate_top_k=3)
    study['source_ref']['identity'] = 'frontend_mapping_map_package:wrong_map'
    workflow._refresh_digest(study)
    (study_dir / 'study.yaml').write_text(yaml.safe_dump(study, sort_keys=False), encoding='utf-8')
    check = validate_study(study_dir / 'study.yaml', verify_evidence=False)
    assert check['state'] == 'STALE'
    assert any('exact frozen Query Set binding' in reason for reason in check['stale_reasons'])


def test_batch_failure_is_partial_and_cancellation_records_not_run(assets, monkeypatch):
    source, blocks, topology, project_root = assets
    draft = project_root / 'draft.yaml'
    create_manual_query_set(query_set_id='cancel', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'ENTRY', 'xy': [4.0, 0.0]}],
                            output=draft)
    frozen = project_root / 'queries' / 'r2.yaml'
    freeze_query_set(draft, frozen)
    study_dir = project_root / 'cancel-study'
    create_study(study_id='cancel-study', query_set_path=frozen, output_dir=study_dir,
                 frames=(1, 3, 5), candidate_top_k=5)
    calls = []

    def fake_run_query(args):
        calls.append(args.query_accumulation_frames)
        if len(calls) == 1:
            signal.raise_signal(signal.SIGTERM)
            evidence = study_dir / 'cancel_evidence'
            evidence.mkdir()
            (evidence / 'manifest.yaml').write_text(yaml.safe_dump({'evidence': {
                'empirical_global_result': {'classification': 'REJECTED', 'converged': False},
                'candidate_ambiguity': {'candidates': []}}}), encoding='utf-8')
            return evidence
        raise AssertionError('cancellation should stop before the next query')

    import signal
    monkeypatch.setattr(workflow, 'run_query', fake_run_query)
    monkeypatch.setattr(workflow, 'verify_evidence_bundle', lambda *_args, **_kwargs: {'revision_state': 'CURRENT'})
    result = workflow.run_study(study_dir / 'study.yaml', ros_install=project_root)
    assert calls == [1]
    assert result['status'] == 'CANCELED'
    assert result['job']['cancel_requested'] is True
    assert [row['classification'] for row in result['results']] == ['REJECTED', 'NOT_RUN', 'NOT_RUN']


def test_project_marks_missing_study_evidence_stale(assets, monkeypatch):
    source, blocks, topology, project_root = assets
    draft = project_root / 'draft.yaml'
    create_manual_query_set(query_set_id='persist', map_package=source, block_dir=blocks,
                            topology_path=topology,
                            locations=[{'query_id': 'Q001', 'scene': 'MIDDLE', 'xy': [4.0, 0.0]}],
                            output=draft)
    frozen = project_root / 'queries' / 'r2.yaml'
    freeze_query_set(draft, frozen)
    study_dir = project_root / 'persistent-study'
    create_study(study_id='persistent-study', query_set_path=frozen,
                 output_dir=study_dir, frames=(1,), candidate_top_k=2)

    evidence_dir = study_dir / 'evidence' / 'Q001' / 'f1' / 'evidence_revision'
    evidence_dir.mkdir(parents=True)
    (evidence_dir / 'manifest.yaml').write_text(yaml.safe_dump({'evidence': {
        'empirical_global_result': {'classification': 'FALSE_ACCEPT', 'converged': True},
        'candidate_ambiguity': {'candidates': [{'rank': 1, 'classification': 'FALSE_ACCEPT'}]}}}),
        encoding='utf-8')
    monkeypatch.setattr(workflow, 'run_query', lambda _args: evidence_dir)
    monkeypatch.setattr(workflow, 'verify_evidence_bundle',
                        lambda *_args, **_kwargs: {'revision_state': 'CURRENT'})
    result = workflow.run_study(study_dir / 'study.yaml', ros_install=project_root)
    assert result['status'] == 'SUCCEEDED'

    project_path = project_root / 'persistent_project.yaml'
    create_project(project_path=project_path, map_package=source, block_dir=blocks,
                   topology_path=topology, query_set_paths=[frozen],
                   study_paths=[study_dir / 'study.yaml'])
    assert validate_project(project_path)['state'] == 'CURRENT'
    shutil.rmtree(evidence_dir)
    assert validate_project(project_path)['state'] == 'STALE'
