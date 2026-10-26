import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_mapping_artifacts.frontend_package import sha256, write_frontend_map_package
from agt_mapping_artifacts.keyframe_blocks import build_keyframe_blocks
from agt_mapping_artifacts.relocalization_evidence import (
    publish_evidence_bundle, validate_evidence_record, verify_evidence_bundle,
)


def record():
    source_digest, pose_digest = 'a' * 64, 'b' * 64
    algorithm_config = {
        'candidate_top_k': 10,
        'query_accumulation_frames': 1,
        'block_keyframe_count': 11,
        'registration': 'existing_global_topk_gicp',
    }
    config_digest = __import__('hashlib').sha256(
        json.dumps(algorithm_config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    ).hexdigest()
    return {
        'schema_version': 1, 'asset_type': 'agt.relocalization_evidence/v1',
        'evidence_id': 'evidence-fixture-001', 'created_utc': '2026-10-06T00:00:00Z',
        'source': {'identity': 'frontend_mapping_map_package:fast_livo2_lio',
                   'manifest_sha256': source_digest, 'checksums_sha256': 'c' * 64,
                   'pose_revision': pose_digest},
        'block_set': {'block_set_id': 'blockset-fixture', 'revision': 1,
                      'parent_source_digest': source_digest,
                      'manifest_sha256': 'd' * 64, 'index_sha256': 'e' * 64,
                      'checksums_sha256': 'f' * 64, 'builder_config_digest': '1' * 64,
                      'block_keyframe_count': 11, 'query_excluded_block_ids': ['block_0000'],
                      'query_excluded_keyframes': [3, 4, 5]},
        'query': {'query_id': 'keyframe-0004-f1', 'keyframe': 4, 'timestamp': 104.0,
                  'window_keyframes': [4], 'query_accumulation_frames': 1,
                  'window_sha256': '2' * 64, 'query_cloud_sha256': '3' * 64,
                  'row_id': 'UNKNOWN', 'along_row_s_m': None,
                  'click_xy_m': None, 'snap_distance_m': None},
        'algorithm': {'name': 'existing_global_topk_gicp', 'candidate_top_k': 10,
                      'config': algorithm_config, 'config_digest': config_digest,
                      'binary_sha256': {'candidate_localizer': '5' * 64},
                      'threads': 8, 'time_budget_s': 18.0},
        'reference': {'type': 'FRONTEND_SAME_SESSION_REFERENCE', 'same_session': True,
                      'independent_ground_truth': False, 'absolute_ground_truth': False,
                      'acceptance_semantics': 'reference_relative_diagnostic'},
        'analysis_artifacts': {'query_cloud': 'analysis/query.pcd'},
        'evidence': {
            'geometry_observability': {'status': 'NO_DATA', 'metrics': {}},
            'candidate_ambiguity': {'status': 'UNKNOWN', 'metrics': {}},
            'local_convergence_basin': {'status': 'CENSORED', 'metrics': {}},
            'empirical_global_result': {'classification': 'REFERENCE_UNKNOWN',
                                        'converged': None, 'reference_relative_only': True},
        },
    }


def make_assets(root: Path):
    records = [{
        'stamp_sec': i + 1, 'stamp_nanosec': 0,
        'position': np.asarray([i * 0.6, 0.0, 0.0]),
        'quaternion_xyzw': np.asarray([0.0, 0.0, 0.0, 1.0]),
        'points_xyzi': np.asarray([[0.0, 0.0, 0.0, float(i)]], dtype='<f4'),
    } for i in range(8)]
    source = root / 'source'
    write_frontend_map_package(source, records, {
        'mapping_backend': {'id': 'fast_livo2_lio', 'project': 'fixture', 'mode': 'lio_only',
                            'source_commit': 'fixture', 'config_sha256': 'a' * 64,
                            'loop_closure': False, 'gps_factor': False,
                            'external_global_correction': False},
        'source': {'kind': 'test_fixture'}, 'frames': {'map': 'map', 'body': 'body'},
        'reference': {'source': 'mapping_frontend_odometry', 'same_session': True,
                      'pgo_applied': False, 'optimized': False, 'absolute_ground_truth': False},
    })
    blocks = root / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=2)
    return source, blocks


def bind_record_to_assets(value, source: Path, blocks: Path):
    source_meta = yaml.safe_load((source / 'metadata.yaml').read_text(encoding='utf-8'))
    block_manifest = yaml.safe_load((blocks / 'manifest.yaml').read_text(encoding='utf-8'))
    value['source'].update({
        'identity': f"frontend_mapping_map_package:{source_meta['mapping_backend']['id']}",
        'path_at_run': str(source),
        'manifest_sha256': sha256(source / 'manifest.yaml'),
        'checksums_sha256': sha256(source / 'checksums.sha256'),
        'pose_revision': sha256(source / 'poses_timed.txt'),
    })
    value['block_set'].update({
        'block_set_id': block_manifest['block_set_id'],
        'path_at_run': str(blocks),
        'parent_source_digest': block_manifest['parent_source_digest'],
        'manifest_sha256': sha256(blocks / 'manifest.yaml'),
        'index_sha256': sha256(blocks / 'block_index.json'),
        'checksums_sha256': sha256(blocks / 'checksums.sha256'),
        'builder_config_digest': block_manifest['builder_config_digest'],
        'block_keyframe_count': block_manifest['builder_config']['block_keyframe_count'],
    })
    value['algorithm']['config']['block_keyframe_count'] = block_manifest['builder_config']['block_keyframe_count']
    value['algorithm']['config_digest'] = hashlib.sha256(
        json.dumps(value['algorithm']['config'], sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    ).hexdigest()
    return value


def test_valid_fixture_bundle_roundtrips_and_is_immutable(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    source = tmp_path / 'trace.json'
    source.write_text(json.dumps({'ranked_candidates': []}), encoding='utf-8')
    query_cloud = tmp_path / 'query.pcd'
    query_cloud.write_bytes(b'fixture query cloud bytes\n')
    published = tmp_path / 'evidence'
    value = bind_record_to_assets(record(), source_package, block_dir)
    value['query']['query_cloud_sha256'] = sha256(query_cloud)
    result = publish_evidence_bundle(
        published, value, {'raw/trace.json': source, 'analysis/query.pcd': query_cloud},
        source_package=source_package, block_dir=block_dir)
    assert result['status'] == 'PASS'
    checked = verify_evidence_bundle(published)
    assert checked['evidence_id'] == 'evidence-fixture-001'
    assert checked['revision_state'] == 'UNCHECKED'
    with pytest.raises(FileExistsError):
        publish_evidence_bundle(published, record(), {}, source_package=source_package,
                                block_dir=block_dir)


def test_publisher_refuses_to_write_inside_source_package(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    value = bind_record_to_assets(record(), source_package, block_dir)
    with pytest.raises(ValueError, match='outside protected asset'):
        publish_evidence_bundle(source_package / 'derived_evidence', value, {},
                                source_package=source_package, block_dir=block_dir)
    assert not (source_package / 'derived_evidence').exists()


def test_publisher_requires_the_declared_query_cloud(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    value = bind_record_to_assets(record(), source_package, block_dir)
    with pytest.raises(ValueError, match='analysis artifact query_cloud.*not included'):
        publish_evidence_bundle(tmp_path / 'evidence', value, {},
                                source_package=source_package, block_dir=block_dir)
    assert not (tmp_path / 'evidence').exists()


@pytest.mark.parametrize(('key', 'value'), [
    ('raw_trace', '../../outside.json'),
    ('summary', '/tmp/outside.json'),
    ('assets_manifest', 'analysis\\..\\outside.json'),
    ('display_only', {'candidate': '../outside.pcd'}),
])
def test_analysis_artifact_paths_must_be_bundle_relative(key, value):
    record_value = record()
    record_value['analysis_artifacts'][key] = value
    with pytest.raises(ValueError, match='safe relative POSIX bundle path'):
        validate_evidence_record(record_value)


def test_publisher_requires_every_declared_analysis_artifact(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    value = bind_record_to_assets(record(), source_package, block_dir)
    value['analysis_artifacts']['raw_trace'] = 'analysis/missing-trace.json'
    query_cloud = tmp_path / 'query.pcd'
    query_cloud.write_bytes(b'query cloud')
    value['query']['query_cloud_sha256'] = sha256(query_cloud)
    with pytest.raises(ValueError, match='analysis artifact raw_trace'):
        publish_evidence_bundle(tmp_path / 'evidence', value,
                                {'analysis/query.pcd': query_cloud},
                                source_package=source_package, block_dir=block_dir)
    assert not (tmp_path / 'evidence').exists()


def test_json_schemas_accept_emitted_records_and_reject_missing_types(tmp_path):
    jsonschema = pytest.importorskip('jsonschema')
    repo = Path(__file__).resolve().parents[3]
    block_schema = json.loads((repo / 'docs/contracts/schemas/agt_keyframe_block_set_v1.schema.json').read_text())
    evidence_schema = json.loads((repo / 'docs/contracts/schemas/agt_relocalization_evidence_v1.schema.json').read_text())
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    block_manifest = yaml.safe_load((block_dir / 'manifest.yaml').read_text(encoding='utf-8'))
    evidence_record = bind_record_to_assets(record(), source_package, block_dir)
    jsonschema.validate(block_manifest, block_schema)
    jsonschema.validate(evidence_record, evidence_schema)

    invalid_evidence = json.loads(json.dumps(evidence_record))
    del invalid_evidence['algorithm']['config']
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid_evidence, evidence_schema)
    invalid_evidence = json.loads(json.dumps(evidence_record))
    invalid_evidence['query']['timestamp'] = 'not-a-number'
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid_evidence, evidence_schema)
    invalid_evidence = json.loads(json.dumps(evidence_record))
    invalid_evidence['analysis_artifacts']['raw_trace'] = '../../outside.json'
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid_evidence, evidence_schema)
    invalid_block = json.loads(json.dumps(block_manifest))
    del invalid_block['builder_config']['algorithm']
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid_block, block_schema)


@pytest.mark.parametrize('mutate', [
    lambda x: x.update(schema_version=2),
    lambda x: x['source'].update(manifest_sha256='not-a-hash'),
    lambda x: x['block_set'].update(parent_source_digest='9' * 64),
    lambda x: x['query'].update(query_accumulation_frames=3),
    lambda x: x['evidence'].update(candidate_ambiguity={'status': 'MAYBE'}),
    lambda x: x['evidence'].update(empirical_global_result={'classification': 'SUCCESS'}),
])
def test_invalid_stale_or_unsupported_contract_is_rejected(mutate):
    value = record()
    mutate(value)
    with pytest.raises(ValueError):
        validate_evidence_record(value)


def test_same_session_correct_requires_reference_relative_semantics():
    value = record()
    value['evidence']['empirical_global_result'] = {
        'classification': 'CORRECT', 'converged': True,
        'reference_error': {'xy_m': 0.1, 'translation_3d_m': 0.1, 'yaw_deg': 0.2},
    }
    with pytest.raises(ValueError, match='reference-relative'):
        validate_evidence_record(value)
    value['evidence']['empirical_global_result']['reference_relative_only'] = True
    validate_evidence_record(value)


def test_contract_rejects_missing_required_nonfinite_and_inconsistent_values():
    value = record()
    del value['source']['identity']
    with pytest.raises(ValueError, match='source missing required'):
        validate_evidence_record(value)

    value = record()
    value['evidence']['candidate_ambiguity'] = {'status': 'KNOWN', 'metrics': {'score': float('nan')}}
    with pytest.raises(ValueError, match='nonfinite'):
        validate_evidence_record(value)

    for classification in ('CORRECT', 'FALSE_ACCEPT'):
        value = record()
        value['evidence']['empirical_global_result'] = {
            'classification': classification, 'converged': False,
            'reference_relative_only': True, 'reference_error': {'xy_m': 0.1},
        }
        with pytest.raises(ValueError, match='requires a converged'):
            validate_evidence_record(value)

    value = record()
    value['evidence']['empirical_global_result'] = {
        'classification': 'CORRECT', 'converged': True, 'reference_relative_only': True,
    }
    with pytest.raises(ValueError, match='translation error'):
        validate_evidence_record(value)


def test_contract_rejects_mixed_row_annotation_revisions():
    value = record()
    value['block_set']['row_annotation_sha256'] = 'a' * 64
    value['query']['row_annotation_sha256'] = 'b' * 64
    with pytest.raises(ValueError, match='different row annotation revisions'):
        validate_evidence_record(value)


def test_annotation_revision_is_part_of_evidence_freshness(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    topology = tmp_path / 'topology.yaml'
    topology.write_text('revision: one\n', encoding='utf-8')
    value = bind_record_to_assets(record(), source_package, block_dir)
    value['query']['row_annotation_sha256'] = hashlib.sha256(topology.read_bytes()).hexdigest()
    query_cloud = tmp_path / 'query.pcd'
    query_cloud.write_bytes(b'annotation fixture query cloud\n')
    value['query']['query_cloud_sha256'] = sha256(query_cloud)
    evidence = tmp_path / 'evidence'
    publish_evidence_bundle(evidence, value, {'analysis/query.pcd': query_cloud},
                            source_package=source_package, block_dir=block_dir,
                            topology_path=topology)
    unchecked = verify_evidence_bundle(evidence, source_package=source_package, block_dir=block_dir)
    assert unchecked['revision_state'] == 'UNCHECKED'
    assert verify_evidence_bundle(evidence, source_package=source_package, block_dir=block_dir,
                                  topology_path=topology)['revision_state'] == 'CURRENT'
    topology.write_text('revision: two\n', encoding='utf-8')
    result = verify_evidence_bundle(evidence, source_package=source_package, block_dir=block_dir,
                                    topology_path=topology)
    assert result['revision_state'] == 'STALE'
    assert 'row annotation revision changed' in result['stale_reasons']


def test_tampered_bundle_is_rejected(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    trace = tmp_path / 'trace.json'
    trace.write_text('{"ok": true}', encoding='utf-8')
    query_cloud = tmp_path / 'query.pcd'
    query_cloud.write_bytes(b'original query cloud\n')
    destination = tmp_path / 'bundle'
    value = bind_record_to_assets(record(), source_package, block_dir)
    value['query']['query_cloud_sha256'] = sha256(query_cloud)
    publish_evidence_bundle(destination, value,
                            {'raw/trace.json': trace, 'analysis/query.pcd': query_cloud},
                            source_package=source_package, block_dir=block_dir)
    (destination / 'raw/trace.json').write_text('{"ok": false}', encoding='utf-8')
    with pytest.raises(ValueError, match='checksum mismatch'):
        verify_evidence_bundle(destination)


def test_query_cloud_digest_mismatch_is_rejected_even_with_refreshed_file_checksums(tmp_path):
    source_package, block_dir = make_assets(tmp_path / 'inputs')
    query_cloud = tmp_path / 'query.pcd'
    query_cloud.write_bytes(b'original query cloud\n')
    value = bind_record_to_assets(record(), source_package, block_dir)
    value['query']['query_cloud_sha256'] = sha256(query_cloud)
    destination = tmp_path / 'bundle'
    publish_evidence_bundle(destination, value, {'analysis/query.pcd': query_cloud},
                            source_package=source_package, block_dir=block_dir)
    (destination / 'analysis/query.pcd').write_bytes(b'different query cloud\n')
    entries = []
    for path in sorted(destination.rglob('*')):
        if path.is_file() and path.name != 'checksums.sha256':
            entries.append(f'{sha256(path)}  {path.relative_to(destination).as_posix()}')
    (destination / 'checksums.sha256').write_text('\n'.join(entries) + '\n', encoding='ascii')
    with pytest.raises(ValueError, match='query cloud digest'):
        verify_evidence_bundle(destination)
