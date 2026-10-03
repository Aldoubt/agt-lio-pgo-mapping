import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from agt_map_localization_benchmark.backends import (local_command, global_command,
                                                      parse_backend)
from agt_map_localization_benchmark.cli import _profile_plan, safe_run_id
from agt_map_localization_benchmark.dataset import (PgoEvidence, Pose, fixed_split,
                                                     key_array, lookup_sorted_keys,
                                                     point_keys, stable_flags)
from agt_map_localization_benchmark.geometry import alignment_and_recovery, query_geometry, spectrum
from agt_map_localization_benchmark.metrics import (SUCCESS_RULES, association, classify_failure,
                                                     initial_pose, measure_pose, perturbations,
                                                     pose_record)
from agt_map_localization_benchmark.pcd import read_pcd, write_pcd, sha256_file
from agt_map_localization_benchmark.selection import make_candidates, voxel_round_robin, spatial_coverage
from agt_map_localization_benchmark.synthetic import scenes, synthetic_geometry


def test_binary_pcd_roundtrip_and_trailer_rejection(tmp_path):
    path = tmp_path / 'source.pcd'
    coords = np.array([[0, 1, 2], [3, 4, 5]], dtype='f4')
    write_pcd(path, coords, np.array([.4, .5], dtype='f4'))
    src = sha256_file(path)
    records = read_pcd(path, ['x', 'y', 'z', 'intensity'])
    assert np.array_equal(np.column_stack((records['x'], records['y'], records['z'])), coords)
    assert src == sha256_file(path)
    with path.open('ab') as f:
        f.write(b'\x00' * 4)
    assert len(read_pcd(path)) == 2
    with path.open('ab') as f:
        f.write(b'NOT_ZERO')
    with pytest.raises(ValueError, match='nonzero trailer'):
        read_pcd(path)
    with pytest.raises(FileExistsError):
        write_pcd(path, coords)


def test_float32_voxel_index_and_sorted_match():
    positions = np.array([[-.0001, 0, .2], [0., 0., .4], [1., 1., 1.]], dtype='f4')
    keys = point_keys(positions, .2)
    assert keys[0]['x'] == -1
    assert keys[0]['z'] == 1
    source = key_array([[-1, 0, 1], [0, 0, 2], [5, 5, 5]])
    pos, found = lookup_sorted_keys(source, keys)
    assert found.tolist() == [True, True, True]
    query = key_array([[100, 0, 0], [-2, 0, 0]])
    _, found = lookup_sorted_keys(source, query)
    assert not found.any()
    evidence = np.zeros(4, dtype=[('final_confidence', '<f4'), ('override_mode', '<u4')])
    evidence['final_confidence'] = [.6, .9, .9, .9]
    evidence['override_mode'] = [0, 2, 3, 1]
    assert stable_flags(evidence, .6).tolist() == [True, False, False, True]


def test_disjoint_split_and_self_query_label():
    poses = tuple(Pose(i, f'{i}.pcd', i, np.array([i * .5, 0., 0.]),
                       np.array([0., 0., 0., 1.])) for i in range(700))
    t1 = fixed_split(poses, (350,), 'tier1')
    t0 = fixed_split(poses, (350,), 'tier0')
    assert set(range(348, 353)).isdisjoint(t1.map_indices)
    assert set(range(348, 353)).issubset(t0.map_indices)
    assert set(t1.map_indices).issubset(t0.map_indices)
    assert 350 in t1.all_query_indices
    assert all(i % 3 != 2 for i in t1.map_indices)


def test_three_five_frame_aggregation_uses_reference_body_and_full_rpy(tmp_path):
    pgo = tmp_path / 'pgo'
    patches = pgo / 'patches'
    patches.mkdir(parents=True)
    positions = np.array([0., 0., 1.], dtype='f8')
    angles = [0, 15, 25, 30, 40]
    poses = []
    for i, angle in enumerate(angles):
        rot = Rotation.from_euler('xyz', [10., -5., angle], degrees=True)
        t = np.array([i * .5, i * .25, 2.], dtype='f8')
        # World point is fixed; body-frame source differs for each pose.
        body = (positions - t) @ rot.as_matrix()
        write_pcd(patches / f'{i}.pcd', np.repeat(body[None, :], 10, axis=0))
        poses.append(Pose(i, f'{i}.pcd', float(i), t, rot.as_quat()))
    data = object.__new__(PgoEvidence)
    data.pgo, data.poses = pgo, tuple(poses)
    for frames in (1, 3, 5):
        result = data.query_body(2, frames)
        expected = (positions - poses[2].t) @ poses[2].rotation
        assert len(result) == frames * 10
        assert np.max(np.linalg.norm(result - expected, axis=1)) < 1e-5


def test_controls_equal_size_and_coverage_deterministic(tmp_path):
    rng = np.random.default_rng(33)
    # 32 ordered V1 voxels, 64 raw points per voxel. Unique Qt per voxel.
    coords = np.concatenate([
        np.tile([i * .4, 0., 0.], (64, 1)) + rng.uniform(-.035, .035, (64, 3))
        for i in range(32)]).astype('f4')
    coords[:, 1:] = .04  # keep each cluster in one 0.2m voxel
    keys = point_keys(coords, .2)
    uniq = np.unique(keys)
    v1 = np.zeros(len(uniq), dtype=[('final_confidence', 'f4'), ('override_mode', 'u4')])
    v1['final_confidence'] = .9
    geom = np.zeros(len(uniq), dtype=[('translation_valid', 'u4'), ('translation_q', 'f4')])
    geom['translation_valid'] = 1
    geom['translation_q'] = np.linspace(.01, .9, len(uniq))
    class Dummy:
        voxel_size = .2
        keys = uniq
        v1_stable = np.ones(len(uniq), dtype=bool)
        reviewed_stable = np.ones(len(uniq), dtype=bool)
        rev = v1
        def raw_map_subset(self, _indices):
            return coords, np.zeros(len(coords), dtype='f4')
    dummy = Dummy()
    dummy.geom = geom
    cands, meta = make_candidates(dummy, (0, 1), tmp_path / 'first')
    counts = {c.name: len(c.indices) for c in cands}
    for q in (25, 50, 75):
        assert counts[f'D_QT_q{q}'] == counts[f'CONTROL_RANDOM_q{q}']
        assert counts[f'D_QT_q{q}'] == counts[f'CONTROL_VOXEL_q{q}']
    chosen = voxel_round_robin(np.arange(len(coords)), coords, 35, 2026)
    assert np.array_equal(chosen, voxel_round_robin(np.arange(len(coords)), coords, 35, 2026))
    assert len(np.unique(chosen)) == 35
    assert 0. <= cands[0].coverage['fraction_of_raw_xy_1m_cells'] <= 1.
    assert 0. <= cands[0].coverage['fraction_of_raw_xyz_1m_cells'] <= 1.
    assert cands[0].coverage['occupied_xyz_1m_cells'] >= cands[0].coverage['occupied_xy_1m_cells']
    assert meta['meta']['eligible_unique_auto_stable_voxels'] > 12


def test_query_conditioned_rotation_changes_with_body_origin():
    # Same immutable normals, different query-origin p-t cross n.
    rng = np.random.default_rng(4)
    coords = rng.uniform(-2, 2, size=(220, 3)).astype('f4')
    lookup = np.arange(len(coords))
    keys = np.arange(len(coords))
    geom = np.zeros(len(coords), dtype=[('normal_valid', 'u4'), ('normal_x', 'f4'),
                                        ('normal_y', 'f4'), ('normal_z', 'f4'),
                                        ('translation_valid', 'u4'), ('translation_q', 'f4'),
                                        ('translation_weak_x', 'f4'), ('translation_weak_y', 'f4'),
                                        ('translation_weak_z', 'f4'),
                                        ('rotation_valid', 'u4'), ('rotation_q', 'f4'),
                                        ('rotation_weak_x', 'f4'), ('rotation_weak_y', 'f4'),
                                        ('rotation_weak_z', 'f4')])
    normals = rng.normal(size=(len(coords), 3))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    for j, axis in enumerate('xyz'):
        geom['normal_' + axis] = normals[:, j]
    geom['normal_valid'] = geom['translation_valid'] = geom['rotation_valid'] = 1
    geom['translation_q'] = geom['rotation_q'] = .4
    geom['translation_weak_x'] = 1.
    geom['rotation_weak_y'] = 1.
    class D:
        geom_meta = {'parameters': {'min_valid_normals': 6,
                                  'epsilon': {'translation': 1e-6, 'rotation_m2': 1e-6}}}
    datum = D()
    datum.geom = geom
    one = query_geometry(datum, coords, lookup, np.ones(len(coords), dtype=bool),
                         keys, np.zeros(3))
    two = query_geometry(datum, coords, lookup, np.ones(len(coords), dtype=bool),
                         keys, np.array([1., 0., 0.]))
    assert one['qr_mapping_view_median'] == two['qr_mapping_view_median'] == pytest.approx(.4)
    assert one['qr_query_conditioned']['q'] != two['qr_query_conditioned']['q']
    assert one['qt_query_local']['q'] == pytest.approx(two['qt_query_local']['q'])
    assert len(one['qr_query_conditioned']['weak_xyz_map']) == 3


def test_native_commands_share_params_and_unavailable_metrics_are_null():
    binary = Path('/tmp/map_gicp_tracker')
    query = Path('/tmp/query.pcd')
    a = local_command(binary, Path('/tmp/A.pcd'), query, np.zeros(3), np.array([0, 0, 0, 1]))
    b = local_command(binary, Path('/tmp/B.pcd'), query, np.zeros(3), np.array([0, 0, 0, 1]))
    assert [(x, y) for x, y in zip(a, b) if x != y] == [('/tmp/A.pcd', '/tmp/B.pcd')]
    assert '--constraint-mode' in a and a[-1] == 'full_se3'
    assert '--bbs-query-frame-mode' in global_command(Path('/tmp/g'), Path('/tmp/m'), query, Path('/tmp/a'))
    success = parse_backend(json.dumps({'success': True, 'x': 0, 'y': 0, 'z': 0,
                                        'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1,
                                        'fitness': .1, 'overlap': .95,
                                        'ambiguity_valid': True,
                                        'ambiguity_margin': .15,
                                        'ambiguity_second_bbs_score': .70,
                                        'candidate_attempted_count': 4,
                                        'candidate_valid_count': 4}), '', 0, 11)
    assert success['pose']['x'] == 0
    assert success['inliers_native'] is None and success['iterations_native'] is None
    assert success['converged'] is True
    assert success['ambiguity_valid_native'] is True
    assert success['ambiguity_margin_native'] == pytest.approx(.15)
    assert success['ambiguity_second_bbs_score_native'] == pytest.approx(.70)
    assert success['candidate_attempted_count_native'] == 4
    assert success['candidate_valid_count_native'] == 4
    fail = parse_backend('{"success":false,"message":"GICP did not converge"}', '', 1, 13)
    assert fail['converged'] is False and fail['pose'] is None


def test_q_spearman_and_bins_group_repeated_starts_not_probabilities():
    rows = []
    for i in range(5):
        for j in range(3):
            rows.append({'tier': 'tier1', 'algorithm': 'LOCAL', 'candidate': f'cand{i}',
                         'center': 350, 'frames': 1, 'nominal_success': j <= i // 2,
                         'geometry': {'qt_mapping_view_median': i / 5.}})
    result = association(rows, 'qt_mapping_view_median')
    assert result['groups'] == 5  # not the 15 repeated perturbation rows
    assert result['spearman_q_vs_empirical_nominal_fraction'] is not None
    assert sum(b['trials'] for b in result['reliability_bins']) == 15
    assert 'NOT calibrated' in result['note']


def test_predeclared_thresholds_perturbations_and_synthetic_geometry():
    scenarios = perturbations()
    assert len(scenarios) == 20
    assert {p['dyaw_deg'] for p in scenarios} >= {0, 5, 10, 20, 45}
    assert {abs(s) for p in scenarios for s in p['dxyz_m']} >= {0, .25, .5, 1, 2}
    assert _profile_plan('standard')['tier1']['scenario_names'] == [p['name'] for p in scenarios]
    with pytest.raises(ValueError):
        safe_run_id('../old')
    assert len(scenes()) == 5
    for points, normals in scenes().values():
        g = synthetic_geometry(points, normals)
        assert g['qt_analytic']['valid']
        assert g['normal_voxels'] > 1000
    ref = pose_record(np.zeros(3), np.array([0, 0, 0, 1]))
    native = parse_backend(json.dumps({'success': True, **ref}), '', 0, 1)
    assert all(measure_pose(ref, native)[n + '_success'] for n in SUCCESS_RULES)
    assert not any(measure_pose(ref, {'pose': None, 'backend_success': False})[n + '_success']
                   for n in SUCCESS_RULES)


def test_requested_thresholds_use_3d_translation_and_heading_not_extra_z_gate():
    assert [(SUCCESS_RULES[k]['translation_3d_m'], SUCCESS_RULES[k]['yaw_deg'])
            for k in ('strict', 'nominal', 'loose')] == [(0.2, 2.), (.5, 5.), (1., 10.)]
    ref = pose_record(np.zeros(3), np.array([0, 0, 0, 1]))
    for z, yaw, expected in ((.19, 1.5, (True, True, True)),
                             (.4, 4., (False, True, True)),
                             (.9, 9., (False, False, True)),
                             (1.1, 1., (False, False, False))):
        pose = pose_record(np.array([0, 0, z]),
                           Rotation.from_euler('z', yaw, degrees=True).as_quat())
        native = parse_backend(json.dumps({'success': True, **pose}), '', 0, 1.)
        actual = measure_pose(ref, native)
        assert tuple(actual[key + '_success'] for key in ('strict', 'nominal', 'loose')) == expected
        assert actual['so3_error_deg'] == pytest.approx(yaw, abs=1e-6)
        assert actual['yaw_error_deg'] == pytest.approx(yaw, abs=1e-6)
    # Pitch and roll do not turn the heading criterion into a 6D metric.
    tilted = Rotation.from_euler('xyz', [13, -17, 27], degrees=True)
    result = Rotation.from_euler('xyz', [13, -17, 30], degrees=True)
    reference = pose_record(np.zeros(3), tilted.as_quat())
    native = parse_backend(json.dumps({'success': True,
                                       **pose_record(np.zeros(3), result.as_quat())}), '', 0, 1)
    assert measure_pose(reference, native)['yaw_error_deg'] == pytest.approx(3., abs=1e-5)


def test_failure_taxonomy_only_reports_available_native_information():
    reference = pose_record(np.zeros(3), np.array([0, 0, 0, 1]))
    def code(msg, *, success=False, pose=None, algo='LOCAL', exit_code=1, ref=reference):
        native = parse_backend(json.dumps({'success': success, 'message': msg,
                                           **(pose or {})}), '', exit_code, 10)
        row = {'reference_pose': ref, 'algorithm': algo, 'native': native}
        row.update(measure_pose(reference, native))
        return classify_failure(row)
    assert code('GICP did not converge') == 'NO_CONVERGENCE'
    assert code('local map has fewer than 1000 points') == 'INSUFFICIENT_MAP_POINTS'
    assert code('query scan too sparse after downsample') == 'INSUFFICIENT_QUERY_POINTS'
    assert code('maximum iteration limit reached') == 'MAX_ITERATIONS'
    assert code('whatever', exit_code=124) == 'TIMEOUT'
    assert code('whatever', ref=None) == 'REFERENCE_UNAVAILABLE'
    assert code('opaque backend failure') == 'UNKNOWN_FAILURE'
    assert code('success with no pose', success=True, exit_code=0) == 'INVALID_RESULT'
    wrong = pose_record(np.array([2., 0., 0.]), np.array([0., 0., 0., 1.]))
    assert code('', success=True, pose=wrong, exit_code=0) == 'WRONG_BASIN'
    assert code('', success=True, pose=wrong, algo='GLOBAL', exit_code=0) == 'FALSE_RELOCALIZATION'
    assert code('', success=True, pose=reference, exit_code=0) is None


def test_mapping_consensus_axial_sign_invariance_and_rotation_recovery():
    from agt_map_localization_benchmark.geometry import mapping_weak_consensus
    sidecar = np.zeros(24, dtype=[('translation_valid', 'u4'), ('translation_weak_x', 'f4'),
                                    ('translation_weak_y', 'f4'), ('translation_weak_z', 'f4'),
                                    ('rotation_valid', 'u4'), ('rotation_weak_x', 'f4'),
                                    ('rotation_weak_y', 'f4'), ('rotation_weak_z', 'f4')])
    sidecar['translation_valid'] = sidecar['rotation_valid'] = 1
    sidecar['translation_weak_x'] = np.where(np.arange(24) % 2, -1, 1)
    sidecar['rotation_weak_z'] = np.where(np.arange(24) % 2, -1, 1)
    mapping_t = mapping_weak_consensus(sidecar, np.arange(24), 'translation', 6)
    mapping_r = mapping_weak_consensus(sidecar, np.arange(24), 'rotation', 6)
    assert mapping_t['valid'] and mapping_r['valid']
    assert mapping_t['axial_coherence'] == pytest.approx(1.)
    geom = {'qt_query_local': {'weak_xyz_map': [1., 0., 0.]},
            'qr_query_conditioned': {'weak_xyz_map': [0., 0., 1.]},
            'translation_mapping_weak_consensus': mapping_t,
            'rotation_mapping_weak_consensus': mapping_r}
    original = np.array([0., 0., 0., 1.])
    diag = alignment_and_recovery(np.zeros(3), np.array([1., 0., 0.]), np.zeros(3),
                                  geom, 45., reference_xyzw=original, result_xyzw=original)
    assert diag['translation_mapping_weak_recovery_m'] == pytest.approx(1.)
    assert diag['translation_query_weak_recovery_m'] == pytest.approx(1.)
    assert diag['rotation_mapping_weak_recovery_deg'] == pytest.approx(45.)
    assert diag['rotation_query_weak_recovery_deg'] == pytest.approx(45.)
    assert diag['rotation_query_weak_alignment_abs_cos'] == pytest.approx(1.)
    assert alignment_and_recovery(np.zeros(3), np.zeros(3), None, geom, 0.)[
        'rotation_mapping_weak_recovery_deg'] is None


def test_native_dependency_hashes_and_prior_asset_verification(tmp_path):
    from agt_map_localization_benchmark.backends import dependency_fingerprints
    from agt_map_localization_benchmark.reanalyse import _source_file_hashes
    install = tmp_path / 'install'
    lib = tmp_path / '.agt_native/lib'
    lib.mkdir(parents=True)
    (lib / 'libsmall_gicp.so').write_bytes(b'GICP-TEST')
    (lib / 'libcpu_bbs3d.so').write_bytes(b'BBS-TEST')
    deps = dependency_fingerprints(install, tmp_path / 'src')
    assert deps['small_gicp']['linked_library_sha256'] == sha256_file(lib / 'libsmall_gicp.so')
    assert deps['3d_bbs_cpu']['linked_library_sha256'] == sha256_file(lib / 'libcpu_bbs3d.so')
    assert deps['small_gicp']['vendored_git_commit'] is None
    run = tmp_path / 'original'
    asset_dir = run / 'real/tier1/assets/A_RAW/assets'
    asset_dir.mkdir(parents=True)
    db = asset_dir / 'polar_context.db'
    voxel = asset_dir / 'voxelmaps_coords/0.pcd'
    voxel.parent.mkdir()
    db.write_bytes(b'descriptor')
    voxel.write_bytes(b'coarse-bbs-data')
    old = {'candidates': {}, 'global_assets': {'tier1/A_RAW': {
        'status': 'READY', 'assets_dir': str(asset_dir),
        'descriptor_database_sha256': sha256_file(db),
        'asset_files_sha256': {'polar_context.db': sha256_file(db),
                               'voxelmaps_coords/0.pcd': sha256_file(voxel)},
    }}}
    verified = _source_file_hashes(run, old)
    assert len(verified) == 2
    voxel.write_bytes(b'MUTATED')
    with pytest.raises(ValueError, match='BBS/descriptor asset changed'):
        _source_file_hashes(run, old)


def test_standard_requires_completed_same_code_native_and_linked_libraries_smoke(tmp_path):
    from argparse import Namespace
    from agt_map_localization_benchmark.cli import _check_standard_predecessor
    from agt_map_localization_benchmark.backends import LOCAL_SETTINGS, GLOBAL_SETTINGS
    from agt_map_localization_benchmark.selection import SWEEP_QUANTILES, CONTROL_SEED
    smoke = tmp_path / 'smoke'
    smoke.mkdir()
    args = Namespace(profile='standard', smoke_run=smoke)
    locks = ({'data': 'source-bytes'}, {'bench': 'source-bytes'},
             {'native': 'source-bytes'}, {'gicp': 'native-bytes'},
             {'small_gicp': 'library-bytes'})
    manifest = {
        'status': 'COMPLETED', 'profile': 'smoke',
        'source_sha256_before': locks[0], 'source_sha256_after': locks[0],
        'benchmark_source_sha256_after': locks[1],
        'algorithm_source_sha256': locks[2],
        'native_executable_sha256': locks[3],
        'linked_registration_dependencies_after': locks[4],
        'parameters': {'local_native': LOCAL_SETTINGS, 'global_native': GLOBAL_SETTINGS,
                       'success_criteria_exploratory': SUCCESS_RULES,
                       'qt_quantile_sweep_predeclared': list(SWEEP_QUANTILES),
                       'control_seed': CONTROL_SEED},
        'synthetic': {'local_cases': 15}, 'real_trial_count': 228,
    }
    file = smoke / 'manifest.json'
    file.write_text(json.dumps(manifest))
    assert _check_standard_predecessor(args, *locks)['manifest_sha256'] == sha256_file(file)
    for name, corrupted in (('benchmark_source_sha256_after', {'bench': 'modified'}),
                            ('native_executable_sha256', {'gicp': 'modified'}),
                            ('linked_registration_dependencies_after', {'small_gicp': 'modified'})):
        altered = dict(manifest)
        altered[name] = corrupted
        file.write_text(json.dumps(altered))
        with pytest.raises(ValueError, match='code/algorithms/settings differ'):
            _check_standard_predecessor(args, *locks)
