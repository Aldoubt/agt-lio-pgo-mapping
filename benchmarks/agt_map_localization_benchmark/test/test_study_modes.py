"""Compatibility contracts for explicitly separated offline research studies."""
from argparse import Namespace
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from agt_map_localization_benchmark import cli
from agt_map_localization_benchmark.backends import GLOBAL_SETTINGS, LOCAL_SETTINGS
from agt_map_localization_benchmark.coverage_sampling import (
    COVERAGE_CELL_M, COVERAGE_FRACTIONS, MIN_POINTS_PER_CELL, MIN_VOXELS_PER_CELL,
)
from agt_map_localization_benchmark.dataset import point_keys
from agt_map_localization_benchmark.metrics import SUCCESS_RULES
from agt_map_localization_benchmark.pcd import sha256_file
from agt_map_localization_benchmark.selection import CONTROL_SEED, SWEEP_QUANTILES, make_candidates


def test_existing_cli_defaults_to_phase3b_and_accepts_portable_explicit_paths(tmp_path):
    required = ['--profile', 'smoke', '--run-id', 'new_run']
    old_command = cli.parser().parse_args(required)
    assert old_command.study == 'phase3b'
    assert old_command.output_root is None
    explicit = cli.parser().parse_args(required + [
        '--study', 'phase3c', '--output-root', str(tmp_path / 'experiments'),
        '--navigation-source', str(tmp_path / 'navigation-source'),
        '--ros-install', str(tmp_path / 'isolated-install'),
    ])
    assert explicit.study == 'phase3c'
    assert explicit.output_root == tmp_path / 'experiments'
    assert explicit.navigation_source == tmp_path / 'navigation-source'
    assert explicit.ros_install == tmp_path / 'isolated-install'


@pytest.fixture
def smoke_record(tmp_path):
    smoke_dir = tmp_path / 'smoke'
    smoke_dir.mkdir()
    locks = ({'map': 'immutable-map'}, {'cli.py': 'benchmark-bytes'},
             {'tracker.cpp': 'algorithm-bytes'}, {'tracker': 'binary-bytes'},
             {'small_gicp': 'linked-library-bytes'})
    manifest = {
        # Original Phase 3B records had no study discriminator or coverage settings.
        'schema_version': 2, 'status': 'COMPLETED', 'profile': 'smoke',
        'source_sha256_before': locks[0], 'source_sha256_after': locks[0],
        'benchmark_source_sha256_after': locks[1],
        'algorithm_source_sha256': locks[2],
        'native_executable_sha256': locks[3],
        'linked_registration_dependencies_after': locks[4],
        'parameters': {
            'local_native': LOCAL_SETTINGS, 'global_native': GLOBAL_SETTINGS,
            'success_criteria_exploratory': SUCCESS_RULES,
            'qt_quantile_sweep_predeclared': list(SWEEP_QUANTILES),
            'control_seed': CONTROL_SEED,
        },
        'synthetic': {'local_cases': 15}, 'real_trial_count': 228,
    }
    return smoke_dir, locks, manifest


def _write_smoke(smoke_dir, manifest):
    path = smoke_dir / 'manifest.json'
    path.write_text(json.dumps(manifest), encoding='utf-8')
    return path


def _phase3c_manifest(manifest, *, explicit_study=True):
    result = deepcopy(manifest)
    result['schema_version'] = 3
    if explicit_study:
        result['study'] = 'phase3c'
    result['parameters'].update({
        'coverage_cell_m': COVERAGE_CELL_M,
        'coverage_fractions_predeclared': list(COVERAGE_FRACTIONS),
        'coverage_min_points_per_cell': MIN_POINTS_PER_CELL,
        'coverage_min_voxels_per_cell': MIN_VOXELS_PER_CELL,
    })
    return result


def test_legacy_phase3b_smoke_without_new_fields_remains_accepted(smoke_record):
    smoke_dir, locks, manifest = smoke_record
    path = _write_smoke(smoke_dir, manifest)
    # Also retain callers that construct the former argparse.Namespace directly.
    args = Namespace(profile='standard', smoke_run=smoke_dir)
    accepted = cli._check_standard_predecessor(args, *locks)
    assert accepted['manifest_sha256'] == sha256_file(path)
    args.study = 'phase3b'
    assert cli._check_standard_predecessor(args, *locks) == accepted


@pytest.mark.parametrize('explicit_study', [False, True])
def test_phase3c_smoke_accepts_original_and_explicit_study_discriminators(smoke_record, explicit_study):
    smoke_dir, locks, manifest = smoke_record
    path = _write_smoke(smoke_dir, _phase3c_manifest(manifest, explicit_study=explicit_study))
    args = Namespace(profile='standard', smoke_run=smoke_dir, study='phase3c')
    assert cli._check_standard_predecessor(args, *locks)['manifest_sha256'] == sha256_file(path)


@pytest.mark.parametrize('source_study,target_study', [('phase3b', 'phase3c'), ('phase3c', 'phase3b')])
def test_smoke_from_other_study_cannot_authorize_standard(smoke_record, source_study, target_study):
    smoke_dir, locks, manifest = smoke_record
    if source_study == 'phase3c':
        manifest = _phase3c_manifest(manifest, explicit_study=False)
    _write_smoke(smoke_dir, manifest)
    args = Namespace(profile='standard', smoke_run=smoke_dir, study=target_study)
    with pytest.raises(ValueError):
        cli._check_standard_predecessor(args, *locks)


def test_smoke_cannot_relabel_phase3c_schema_as_phase3b(smoke_record):
    smoke_dir, locks, manifest = smoke_record
    manifest = _phase3c_manifest(manifest)
    manifest['study'] = 'phase3b'
    _write_smoke(smoke_dir, manifest)
    with pytest.raises(ValueError, match='study/schema mismatch'):
        cli._check_standard_predecessor(
            Namespace(profile='standard', smoke_run=smoke_dir, study='phase3b'), *locks)


@pytest.mark.parametrize('study', ['phase3b', 'phase3c'])
def test_compatibility_never_bypasses_same_code_smoke_lock(smoke_record, study):
    smoke_dir, locks, manifest = smoke_record
    if study == 'phase3c':
        manifest = _phase3c_manifest(manifest)
    manifest['benchmark_source_sha256_after'] = {'cli.py': 'old-code'}
    _write_smoke(smoke_dir, manifest)
    args = Namespace(profile='standard', smoke_run=smoke_dir, study=study)
    with pytest.raises(ValueError):
        cli._check_standard_predecessor(args, *locks)


@pytest.mark.parametrize('setting', [
    'coverage_cell_m', 'coverage_fractions_predeclared',
    'coverage_min_points_per_cell', 'coverage_min_voxels_per_cell',
])
def test_phase3c_requires_its_complete_sampling_protocol(smoke_record, setting):
    smoke_dir, locks, manifest = smoke_record
    manifest = _phase3c_manifest(manifest)
    del manifest['parameters'][setting]
    _write_smoke(smoke_dir, manifest)
    args = Namespace(profile='standard', smoke_run=smoke_dir, study='phase3c')
    with pytest.raises(ValueError):
        cli._check_standard_predecessor(args, *locks)


def _candidate_source():
    rng = np.random.default_rng(22)
    coords = np.concatenate([
        np.tile([i * .4 + .05, .05, .05], (64, 1)) + rng.uniform(-.02, .02, (64, 3))
        for i in range(32)
    ]).astype('f4')
    keys = np.unique(point_keys(coords, .2))
    geom = np.zeros(len(keys), dtype=[('translation_valid', 'u4'), ('translation_q', 'f4')])
    geom['translation_valid'] = 1
    geom['translation_q'] = np.linspace(.01, .9, len(keys))
    source = SimpleNamespace(
        voxel_size=.2, keys=keys, geom=geom, rev=None,
        v1_stable=np.ones(len(keys), dtype=bool),
        raw_map_subset=lambda _indices: (coords, np.zeros(len(coords), dtype='f4')),
    )
    return source


def test_phase3c_is_opt_in_and_keeps_legacy_candidate_bytes_unchanged(tmp_path):
    source = _candidate_source()
    # The Phase 3B sampler did not need normal_valid for coverage-field diagnostics.
    baseline, _ = make_candidates(source, (0, 1), tmp_path / 'phase3b')
    legacy = {candidate.name: candidate for candidate in baseline}
    expected_legacy = {'A_RAW', 'B_AUTO_STABLE'} | {
        f'{prefix}_q{q}' for q in (25, 50, 75)
        for prefix in ('D_QT', 'CONTROL_RANDOM', 'CONTROL_VOXEL')
    }
    assert set(legacy) == expected_legacy
    original = source.geom
    source.geom = np.zeros(len(original), dtype=original.dtype.descr + [('normal_valid', 'u4')])
    for field in original.dtype.names:
        source.geom[field] = original[field]
    source.geom['normal_valid'] = 1
    extended, _ = make_candidates(source, (0, 1), tmp_path / 'phase3c', study='phase3c')
    extended = {candidate.name: candidate for candidate in extended}
    expected_extra = {
        f'{prefix}_f{f}' for f in (25, 50, 75)
        for prefix in ('D_COVERAGE_QT', 'CONTROL_CELL_RANDOM', 'CONTROL_CELL_UNIFORM')
    }
    assert set(extended) == expected_legacy | expected_extra
    for name, candidate in legacy.items():
        assert np.array_equal(candidate.indices, extended[name].indices)
        assert sha256_file(candidate.path) == sha256_file(extended[name].path)


def test_configured_output_is_portable_and_cannot_overlap_sources(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    cli._validate_output_root(tmp_path / 'experiments' / 'phase3c', [source])
    for output in (source, source / 'results', tmp_path):
        with pytest.raises(ValueError):
            cli._validate_output_root(output, [source])
    alias = tmp_path / 'source-alias'
    alias.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError):
        cli._validate_output_root(alias / 'results', [source])
    assert list(source.iterdir()) == []


def test_algorithm_fingerprints_use_configured_checkout_and_track_content(tmp_path):
    checkout = tmp_path / 'navigation-checkout'
    relative_files = [
        'agt_global_relocalization_native/src/map_gicp_tracker.cpp',
        'agt_global_relocalization_native/src/candidate_bbs_gicp_localizer.cpp',
        'agt_global_relocalization_native/src/build_relocalization_assets.cpp',
        'agt_global_relocalization_native/src/build_relocalization_candidates.cpp',
        'agt_global_relocalization_native/include/agt_global_relocalization_native/polar_context.hpp',
        'agt_global_relocalization/config/global_relocalization.yaml',
    ]
    paths = []
    for relative in relative_files:
        path = checkout / 'navigation' / 'localization' / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative, encoding='utf-8')
        paths.append(path)
    before = cli._algorithm_source_hashes(checkout)
    assert before == {str(path): sha256_file(path) for path in paths}
    paths[0].write_text('changed algorithm implementation', encoding='utf-8')
    after = cli._algorithm_source_hashes(checkout)
    assert after[str(paths[0])] != before[str(paths[0])]
    assert {key for key in before if before[key] != after[key]} == {str(paths[0])}


def test_postprocess_accepts_configured_sources_and_blocks_writes_inside_old_run(tmp_path, monkeypatch):
    from agt_map_localization_benchmark import reanalyse
    source_run = tmp_path / 'old-run'
    source_run.mkdir()
    argv = ['--source-run', str(source_run), '--output-dir', str(source_run / 'new-output'),
            '--navigation-source', str(tmp_path / 'navigation'),
            '--dependency-source-root', str(tmp_path / 'dependencies')]
    args = reanalyse.parser().parse_args(argv)
    assert args.navigation_source == tmp_path / 'navigation'
    assert args.dependency_source_root == tmp_path / 'dependencies'
    def no_data_reads(*_args):
        pytest.fail('output overlap must be rejected before reading or creating source data')
    monkeypatch.setattr(reanalyse, 'PgoEvidence', no_data_reads)
    with pytest.raises(SystemExit, match='disjoint'):
        reanalyse.main(argv)
    assert list(source_run.iterdir()) == []


@pytest.mark.parametrize('study,schema', [('phase3b', 2), ('phase3c', 3)])
def test_cli_orchestration_keeps_protocol_manifest_and_reports_separate(tmp_path, monkeypatch, study, schema):
    """Exercise the real CLI orchestration; native execution is explicitly stubbed."""
    inputs = {name: tmp_path / name for name in ('map', 'confidence', 'geometry', 'reviewed')}
    for path in inputs.values():
        path.mkdir()
    data = SimpleNamespace(
        pgo=inputs['map'], confidence=inputs['confidence'], geometry=inputs['geometry'],
        reviewed=inputs['reviewed'], rev=object(), validate_sources=lambda: {'map': 'locked'})
    native = SimpleNamespace(local_ready=True, global_ready=False, sha256={'native': 'locked'})
    monkeypatch.setattr(cli, 'PgoEvidence', lambda *_args: data)
    monkeypatch.setattr(cli, 'NativePrograms', SimpleNamespace(discover=lambda _path: native))
    monkeypatch.setattr(cli, 'dependency_fingerprints', lambda *_args: {'library': 'locked'})
    monkeypatch.setattr(cli, '_algorithm_source_hashes', lambda _source: {'algorithm': 'locked'})
    monkeypatch.setattr(cli, '_benchmark_source_hashes', lambda: {'code': 'locked'})
    monkeypatch.setattr(cli, '_source_version', lambda _path: {'commit': 'fixture'})
    monkeypatch.setattr(cli, 'run_synthetic', lambda *_args: {'local_cases': 15})
    observed = []
    def run_real(_run, _data, _native, _plan, manifest, **_kwargs):
        observed.append(manifest['study'])
        return [{'fixture': 'no native registration'}], {}, []
    monkeypatch.setattr(cli, '_run_real', run_real)
    monkeypatch.setattr(cli, 'write_outputs', lambda *_args: None)
    def phase3c_report(run, *_args):
        (run / 'coverage_field.csv').write_text('fixture only\n')
        return {'semantics': 'observed map support; not navigation or task coverage'}
    monkeypatch.setattr(cli, 'write_phase3c_outputs', phase3c_report)
    out = tmp_path / 'portable-experiments'
    argv = ['--profile', 'smoke', '--run-id', 'run', '--output-root', str(out),
            '--map-package', str(inputs['map']), '--confidence-source', str(inputs['confidence']),
            '--geometry-source', str(inputs['geometry']), '--reviewed-source', str(inputs['reviewed']),
            '--ros-install', str(tmp_path / 'native-install'),
            '--navigation-source', str(tmp_path / 'navigation')]
    if study == 'phase3c':
        argv += ['--study', study]
    assert cli.main(argv) == 0
    manifest = json.loads((out / 'run/manifest.json').read_text())
    assert manifest['schema_version'] == schema and manifest['study'] == study
    assert manifest['status'] == 'COMPLETED' and observed == [study]
    assert manifest['offline_only'] and not manifest['publish_production_map']
    assert ('coverage_cell_m' in manifest['parameters']) == (study == 'phase3c')
    assert ('phase3c_analysis' in manifest) == (study == 'phase3c')
    assert (out / 'run/coverage_field.csv').exists() == (study == 'phase3c')
    with pytest.raises(SystemExit, match='refusing overwrite'):
        cli.main(argv)
    assert observed == [study]
    # STANDARD must be able to share its output parent with the frozen SMOKE.
    standard = list(argv)
    standard[standard.index('smoke')] = 'standard'
    standard[standard.index('run')] = 'standard'
    standard += ['--smoke-run', str(out / 'run')]
    assert cli.main(standard) == 0
    standard_manifest = json.loads((out / 'standard/manifest.json').read_text())
    assert standard_manifest['smoke_predecessor']['manifest_sha256'] == sha256_file(out / 'run/manifest.json')
    assert standard_manifest['study'] == study and observed == [study, study]
