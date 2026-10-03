"""Contracts for source reuse, command metadata and input immutability.

The native invocation is mocked: these tests cannot exercise retrieval or BBS.
Actual native ordering/result parity belongs to the native instrumentation suite.
"""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_map_localization_benchmark import topk_trace_replay as replay
from agt_map_localization_benchmark.backends import GLOBAL_SETTINGS
from agt_map_localization_benchmark.pcd import sha256_file, write_pcd


@pytest.fixture
def source(tmp_path):
    package = tmp_path / 'map_package'
    package.mkdir()
    (package / 'patches').mkdir()
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]], dtype='<f4')
    write_pcd(package / 'map.pcd', points)
    rows = []
    for index in range(12):
        write_pcd(package / 'patches' / f'{index}.pcd', points)
        rows.append(f'{index}.pcd {1000+index} {index} 0 0 1 0 0 0\n')
    (package / 'poses_timed.txt').write_text(''.join(rows), encoding='ascii')
    (package / 'manifest.yaml').write_text('format_version: 1\n', encoding='utf-8')
    (package / 'metadata.yaml').write_text(yaml.safe_dump({
        'mapping_backend': {'id': 'point_lio'},
        'reference_type': 'FRONTEND_SAME_SESSION_REFERENCE',
        'reference': {'source': 'mapping_frontend_odometry', 'pgo_applied': False,
                      'optimized': False, 'absolute_ground_truth': False, 'same_session': True},
    }), encoding='utf-8')
    indexed = sorted(p for p in package.rglob('*') if p.is_file())
    (package / 'checksums.sha256').write_text(''.join(
        f'{sha256_file(p)}  {p.relative_to(package).as_posix()}\n' for p in indexed), encoding='ascii')
    scenes = tmp_path / 'scenes.yaml'
    scenes.write_text(yaml.safe_dump({'schema_version': 1, 'scenes': [
        {'id': 'MIDDLE_01', 'type': 'row_middle', 'keyframe': 3},
        {'id': 'HEADLAND_01', 'type': 'headland', 'keyframe': 8},
    ]}), encoding='utf-8')
    benchmark = tmp_path / 'benchmark'
    assets = benchmark / 'global' / 'assets'
    queries = benchmark / 'global' / 'queries'
    assets.mkdir(parents=True)
    queries.mkdir()
    for filename in ('polar_context.db', 'polar_context.yaml', 'global_map_downsampled.pcd'):
        (assets / filename).write_bytes(b'frozen descriptor/BBS fixture\n')
    target = benchmark / 'global' / 'global_target_map.pcd'
    write_pcd(target, points)
    for scene in ('MIDDLE_01', 'HEADLAND_01'):
        for count in (1, 3, 5):
            write_pcd(queries / f'{scene}_f{count}.pcd', points)
    asset_manifest = {
        'target': {'sha256': sha256_file(target), 'patch_indices': [0, 11]},
        'heldout_query_indices': list(range(1, 11)),
        'asset_files_sha256': {p.name: sha256_file(p) for p in assets.iterdir()},
    }
    (benchmark / 'global' / 'assets_manifest.json').write_text(json.dumps(asset_manifest), encoding='utf-8')
    settings = GLOBAL_SETTINGS | {'local_map_radius_xy_m': 37.0}
    manifest = {'status': 'COMPLETED', 'map_package': str(package),
                'scene_yaml_sha256': sha256_file(scenes),
                'map_checksums_sha256': sha256_file(package / 'checksums.sha256'),
                'parameters': {'frames': [1, 3, 5], 'global_native': settings}}
    (benchmark / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    binary = tmp_path / 'candidate_bbs_gicp_localizer'
    binary.write_bytes(b'not run: native invocation is mocked\n')
    binary.chmod(0o755)
    return {'map_package': package, 'scenes': scenes, 'benchmark_assets_root': benchmark,
            'native_localizer_path': binary, 'output': tmp_path / 'new_run', 'backend_id': 'point_lio'}


def native_mock(command, *, timeout):
    assert timeout == 30.0
    get = lambda flag: command[command.index(flag)+1]
    query = {name: get('--query-' + name.replace('_', '-'))
             for name in ('scene_id', 'scene_type', 'backend_id')}
    query.update(frames=int(get('--query-frames')), keyframe=int(get('--query-keyframe')),
                 timestamp=float(get('--query-timestamp')))
    pose = {'x': float(query['keyframe']), 'y': 0., 'z': 0., 'qx': 0., 'qy': 0., 'qz': 0., 'qw': 1.}
    trace = {'schema_version': 1, 'query': query,
             'descriptor': {'database_size': 2, 'prefilter': int(get('--descriptor-prefilter')),
                            'candidate_top_k': int(get('--candidate-top-k'))},
             'ranked_candidates': [
                 {'rank': 1, 'patch': '0.pcd', 'keyframe': 0,
                  'descriptor': {'ring_distance': .2, 'sector_similarity': .9},
                  'map_pose': {'x': 0., 'y': 0., 'z': 0., 'yaw_deg': 0.},
                  'bbs': {'attempted': True, 'valid': True, 'coarse_pose': pose},
                  'gicp': {'attempted': True, 'converged': True, 'final_pose': pose}},
             ], 'selected': {'candidate_rank': 1, 'reason': 'best_valid_bbs_score'}}
    Path(get('--trace-candidates-json')).write_text(json.dumps(trace), encoding='utf-8')
    return {'backend_success': True, 'backend_exit_code': 0, 'backend_reason': None,
            'pose': pose, 'wall_ms_external': 1.}


def test_defaults_are_explicit_paper_mode_not_runtime_changes(source):
    args = []
    for name, value in source.items():
        args += ['--' + name.replace('_', '-'), str(value)]
    parsed = replay.make_parser().parse_args(args)
    assert parsed.candidate_top_k == 10
    assert parsed.frames == (1, 3, 5)
    assert parsed.scene_ids == ('MIDDLE_01', 'HEADLAND_01')
    assert GLOBAL_SETTINGS['candidate_top_k'] == 4


def test_mocked_replay_preserves_raw_trace_selection_and_freezes_full_inputs(source, monkeypatch):
    monkeypatch.setattr(replay, 'invoke', native_mock)
    result = replay.run_replay(**source)
    assert result['status'] == 'COMPLETE'
    assert result['query_count'] == result['trace_available_count'] == 6
    assert result['immutable_inputs_verified'] is True
    assert result['native_success_count'] == result['offline_reference_nominal_success_count'] == 6
    assert result['global_settings']['local_map_radius_xy_m'] == 37.0
    before = json.loads((source['output'] / 'input_hashes_before.json').read_text())
    after = json.loads((source['output'] / 'input_hashes_after.json').read_text())
    assert before == after
    assert 'patches/11.pcd' in before['map_package_indexed_files_sha256']
    for record in result['queries']:
        raw = json.loads(Path(record['native_trace']).read_text())
        enriched = json.loads(Path(record['enriched_trace']).read_text())
        assert raw['ranked_candidates'] == enriched['ranked_candidates']
        assert raw['selected'] == enriched['selected']
        assert 'reference_pose' not in raw['query']
        assert enriched['query']['reference_pose']['x'] == record['keyframe']
        assert enriched['query']['scene_type'] in ('ROW_MIDDLE', 'HEADLAND')
        assert enriched['acceptance']['owner'] == 'offline_reference_tolerance'
        assert enriched['acceptance']['runtime_acceptance_observed'] is False
        assert enriched['provenance']['descriptor_database_sha256'] == before['asset_files_sha256']['polar_context.db']
        assert enriched['provenance']['map_metadata_sha256'] == sha256_file(source['map_package'] / 'metadata.yaml')
        invocation = json.loads(Path(record['invocation']).read_text())
        argv = invocation['argv']
        assert argv[argv.index('--candidate-top-k')+1] == '10'
        assert argv[argv.index('--local-map-radius-xy')+1] == '37.0'
        assert argv[argv.index('--query-backend-id')+1] == 'point_lio'


@pytest.mark.parametrize('overrides, message', [
    ({'scene_ids': ('MIDDLE_01', 'MIDDLE_01')}, 'unique'),
    ({'scene_ids': ('ABSENT',)}, 'absent'),
    ({'frames': (1, 2)}, 'frames'),
    ({'candidate_top_k': 0}, 'positive'),
    ({'candidate_top_k': 41}, 'prefilter'),
    ({'backend_id': 'wrong_backend'}, 'differs from map backend'),
])
def test_invalid_replay_selection_fails_before_invocation_or_output(source, monkeypatch, overrides, message):
    def forbidden(*args, **kwargs):
        pytest.fail('invalid inputs must never invoke native')
    monkeypatch.setattr(replay, 'invoke', forbidden)
    with pytest.raises(ValueError, match=message):
        replay.run_replay(**(source | overrides))
    assert not source['output'].exists()


def test_existing_output_is_never_overwritten(source):
    source['output'].mkdir()
    sentinel = source['output'] / 'user_note.txt'
    sentinel.write_text('preserve existing data')
    with pytest.raises(ValueError, match='already exists'):
        replay.run_replay(**source)
    assert sentinel.read_text() == 'preserve existing data'


def test_changed_asset_bytes_rejected_against_prior_manifest(source):
    db = source['benchmark_assets_root'] / 'global' / 'assets' / 'polar_context.db'
    db.write_bytes(b'changed database')
    with pytest.raises(ValueError, match='asset set or hashes'):
        replay.run_replay(**source)
    assert not source['output'].exists()


def test_query_window_must_be_excluded_from_descriptor_and_target(source):
    path = source['benchmark_assets_root'] / 'global' / 'assets_manifest.json'
    manifest = json.loads(path.read_text())
    manifest['heldout_query_indices'].remove(3)
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='not excluded'):
        replay.run_replay(**source)
    assert not source['output'].exists()


def test_input_mutation_during_run_is_reported_and_preserved(source, monkeypatch):
    def mutate(command, *, timeout):
        result = native_mock(command, timeout=timeout)
        path = Path(command[command.index('--scan')+1])
        path.write_bytes(path.read_bytes() + b'changed after snapshot\n')
        return result
    monkeypatch.setattr(replay, 'invoke', mutate)
    with pytest.raises(ValueError, match='input bytes changed'):
        replay.run_replay(**source)
    manifest = json.loads((source['output'] / 'manifest.json').read_text())
    assert manifest['status'] == 'FAILED'
    assert manifest['immutable_inputs_verified'] is False
    assert manifest['trace_available_count'] == 6
    assert len(list((source['output'] / 'native_traces').glob('*.json'))) == 6


def test_missing_native_trace_is_unavailable_not_fabricated(source, monkeypatch):
    monkeypatch.setattr(replay, 'invoke', lambda *args, **kwargs: {
        'backend_success': False, 'backend_exit_code': 124, 'backend_reason': 'external timeout', 'pose': None})
    result = replay.run_replay(**source)
    assert result['status'] == 'COMPLETE_WITH_TRACE_ERRORS'
    assert result['trace_available_count'] == 0
    assert result['native_success_count'] == 0
    assert result['immutable_inputs_verified'] is True
    assert not list((source['output'] / 'traces').glob('*.json'))
    assert len(result['trace_errors']) == 6


def test_trace_metadata_mismatch_is_not_hidden_by_enrichment(source, monkeypatch):
    def wrong_metadata(command, *, timeout):
        result = native_mock(command, timeout=timeout)
        path = Path(command[command.index('--trace-candidates-json')+1])
        trace = json.loads(path.read_text())
        trace['query']['keyframe'] += 1
        path.write_text(json.dumps(trace))
        return result
    monkeypatch.setattr(replay, 'invoke', wrong_metadata)
    result = replay.run_replay(**source)
    assert result['status'] == 'COMPLETE_WITH_TRACE_ERRORS'
    assert result['trace_available_count'] == 0
    assert all('keyframe differs' in error for error in result['trace_errors'])
