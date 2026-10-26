"""Meaningful opt-in native parity tests; fixture paths are supplied by the audit runner."""
import json
import os
from pathlib import Path
import subprocess

import pytest

from agt_map_localization_benchmark.backends import GLOBAL_SETTINGS, global_command, enrich_candidate_trace
from agt_map_localization_benchmark.greenhouse import parser


def test_trace_cli_default_and_global_only_opt_in():
    required = ['--map-package', '/data/map', '--scenes', '/data/scenes.yaml', '--run-id', 'new_map']
    ordinary = parser().parse_args(required)
    assert ordinary.trace_candidates is False and ordinary.candidate_top_k is None
    assert ordinary.skip_coarse_seed_local is False
    traced = parser().parse_args(required + ['--trace-candidates', '--candidate-top-k', '10',
                                            '--skip-coarse-seed-local'])
    assert traced.trace_candidates and traced.candidate_top_k == 10 and traced.skip_coarse_seed_local


def test_default_command_and_opt_in_trace(tmp_path):
    ordinary = global_command(Path('native'), Path('map'), Path('scan'), Path('assets'))
    assert '--trace-candidates-json' not in ordinary
    assert ordinary[ordinary.index('--candidate-top-k') + 1] == '4'
    changed = global_command(Path('native'), Path('map'), Path('scan'), Path('assets'),
                             settings={'candidate_top_k': 10}, trace_candidates_json=tmp_path/'trace.json',
                             query_metadata={'frames': 3, 'scene_id': 'ROW_A', 'irrelevant': 'ignored'})
    assert changed[changed.index('--candidate-top-k') + 1] == '10'
    assert '--query-frames' in changed and '--query-irrelevant' not in changed
    assert GLOBAL_SETTINGS['candidate_top_k'] == 4


def test_offline_enrichment_has_no_runtime_acceptance(tmp_path):
    path = tmp_path/'trace.json'
    path.write_text(json.dumps({'schema_version': 1, 'query': {}, 'ranked_candidates': [],
                               'selected': {'native_success': False}}))
    enrich_candidate_trace(path, query={'reference_pose': {'x': 1}}, nominal_success=False,
                           provenance={'reference_type': 'SAME_SESSION'})
    trace = json.loads(path.read_text())
    assert trace['acceptance']['owner'] == 'offline_reference_tolerance'
    assert trace['acceptance']['runtime_acceptance_observed'] is False
    assert trace['selected']['native_success'] is False


@pytest.fixture
def native_fixture():
    names = ['AGT_TOPK_BASELINE_BINARY', 'AGT_TOPK_TRACE_BINARY', 'AGT_TOPK_FIXTURE_ROOT']
    if not all(os.environ.get(n) for n in names):
        pytest.skip('isolated native baseline/build and immutable greenhouse fixtures required')
    return tuple(Path(os.environ[n]) for n in names)


def run_json(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=90)
    payload = next(json.loads(line) for line in reversed(result.stdout.splitlines()) if line.startswith('{'))
    return result.returncode, payload


@pytest.mark.parametrize('query_name', ['MIDDLE_01_f1', 'HEADLAND_01_f1', 'MIDDLE_01_f3'])
def test_baseline_off_on_selection_pose_parity(native_fixture, tmp_path, query_name):
    baseline, instrumented, root = native_fixture
    settings = {'threads': 1, 'timeout_sec': 60., 'per_candidate_timeout_sec': 20., 'candidate_top_k': 2}
    def command(binary, trace=None):
        return global_command(binary, root/'global_target_map.pcd', root/'queries'/f'{query_name}.pcd',
                              root/'assets', settings=settings, trace_candidates_json=trace)
    old = run_json(command(baseline))
    off = run_json(command(instrumented))
    trace_path = tmp_path/'trace.json'
    on = run_json(command(instrumented, trace_path))
    # Timing is observational and naturally differs; every algorithmic result field is compared.
    for result in (old, off, on):
        result[1].pop('bbs_elapsed_ms', None)
    assert old == off == on
    trace = json.loads(trace_path.read_text())
    candidates = trace['ranked_candidates']
    assert trace['schema_version'] == 1 and len(candidates) == 40
    assert [c['rank'] for c in candidates] == list(range(1, 41))
    for a, b in zip(candidates, candidates[1:]):
        sim_a, sim_b = a['descriptor']['sector_similarity'], b['descriptor']['sector_similarity']
        assert sim_a >= sim_b - 1e-9
        if abs(sim_a - sim_b) <= 1e-9:
            assert a['descriptor']['ring_distance'] <= b['descriptor']['ring_distance']
    assert all(not c['bbs']['attempted'] and c['bbs']['valid'] is None for c in candidates[2:])
    attempted_gicp = [c for c in candidates if c['gicp']['attempted']]
    if on[0] == 0:
        assert len(attempted_gicp) == 1
        winner = candidates[trace['selected']['candidate_rank']-1]
        assert winner['patch'] == on[1]['candidate_patch']
        assert winner['bbs']['valid'] is True
        assert winner['gicp']['converged'] is True
    assert trace['acceptance']['nominal_success'] is None


def test_native_failure_retains_strict_trace(native_fixture, tmp_path):
    _, instrumented, _ = native_fixture
    path = tmp_path/'failure.json'
    status, result = run_json([str(instrumented), '--map', '/missing', '--scan', '/missing',
                              '--assets-dir', '/missing', '--trace-candidates-json', str(path)])
    assert status != 0 and result['success'] is False
    trace = json.loads(path.read_text())
    assert trace['schema_version'] == 1
    assert trace['ranked_candidates'] == []
    assert trace['selected']['native_success'] is False
    assert 'error' in trace


def test_trace_io_error_does_not_change_failure(native_fixture, tmp_path):
    _, instrumented, _ = native_fixture
    blocker = tmp_path/'file'
    blocker.write_text('file blocks output directory')
    common = [str(instrumented), '--map', '/missing', '--scan', '/missing', '--assets-dir', '/missing']
    assert run_json(common) == run_json(common + ['--trace-candidates-json', str(blocker/'trace.json')])
