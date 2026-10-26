from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

import agt_map_localization_benchmark.relocalization_mvp as mvp
from agt_map_localization_benchmark.backends import NativePrograms, PROGRAMS
from agt_map_localization_benchmark.greenhouse import MapPackage
from agt_map_localization_benchmark.pcd import sha256_file, write_pcd
from agt_mapping_artifacts.frontend_package import write_frontend_map_package
from agt_mapping_artifacts.keyframe_blocks import build_keyframe_blocks
from agt_mapping_artifacts.relocalization_evidence import verify_evidence_bundle


def _source_package(root: Path) -> Path:
    records = []
    for index in range(8):
        records.append({
            'stamp_sec': 100 + index,
            'stamp_nanosec': 0,
            'position': [float(index), 0.0, 0.0],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            'points_xyzi': np.asarray([
                [0.0, 0.0, 0.0, 1.0],
                [0.1, 0.0, 0.0, 2.0],
                [0.0, 0.1, 0.0, 3.0],
            ], dtype='<f4'),
        })
    provenance = {
        'mapping_backend': {
            'id': 'fast_livo2_lio', 'project': 'fixture', 'mode': 'lio_only',
            'source_commit': 'fixture', 'config_sha256': 'a' * 64,
            'loop_closure': False, 'gps_factor': False,
            'external_global_correction': False,
        },
        'source': {'kind': 'integration_fixture'},
        'frames': {'map': 'map', 'body': 'body'},
        'reference': {
            'source': 'mapping_frontend_odometry', 'same_session': True,
            'pgo_applied': False, 'optimized': False,
            'absolute_ground_truth': False,
        },
    }
    package = root / 'source'
    write_frontend_map_package(package, records, provenance)
    return package


def test_small_fixture_source_to_blocks_query_evidence_and_reload(tmp_path: Path, monkeypatch):
    source = _source_package(tmp_path / 'fixture')
    blocks = tmp_path / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=3, stride=3)
    dataset = MapPackage(source)

    native_root = tmp_path / 'native'
    native_root.mkdir()
    native_paths = {}
    native_hashes = {}
    native_available = {}
    for name in PROGRAMS:
        path = native_root / name
        path.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
        path.chmod(0o755)
        native_paths[name] = path
        native_hashes[name] = sha256_file(path)
        native_available[name] = True
    fake_native = NativePrograms(native_paths, native_hashes, native_available)
    monkeypatch.setattr(
        mvp.NativePrograms, 'discover',
        classmethod(lambda _cls, _prefix: fake_native),
    )
    monkeypatch.setattr(mvp, 'supports_candidate_trace', lambda _path: True)
    monkeypatch.setattr(mvp, 'dependency_fingerprints', lambda *_args: {'fixture': {'sha256': 'b' * 64}})

    def fake_run_global(_dataset, config, _native, run, frames, _local_targets, **kwargs):
        scene = config.scenes[0]
        global_root = run / 'global'
        (global_root / 'queries').mkdir(parents=True)
        (global_root / 'traces').mkdir()
        (global_root / 'assets').mkdir()
        query_path = global_root / 'queries' / f'{scene.scene_id}_f{frames[0]}.pcd'
        write_pcd(query_path, dataset.query_body(scene.keyframe, frames[0]))
        (global_root / 'global.csv').write_text('query,success\nfixture,false\n', encoding='utf-8')
        (global_root / 'assets_manifest.json').write_text(
            json.dumps({'fixture': True}) + '\n', encoding='utf-8')
        trace = {
            'ranked_candidates': [
                {
                    'rank': 1, 'keyframe': 0, 'patch': '0.pcd',
                    'descriptor': {'sector_similarity': 0.91, 'ring_distance': 0.1},
                    'bbs': {'attempted': True, 'valid': True, 'score': 0.8},
                    'gicp': {'attempted': True, 'converged': True, 'fitness': 0.4},
                },
                {
                    'rank': 2, 'keyframe': 1, 'patch': '1.pcd',
                    'descriptor': {'sector_similarity': 0.88, 'ring_distance': 0.2},
                    'bbs': {'attempted': True, 'valid': True, 'score': 0.7},
                    'gicp': {'attempted': False, 'converged': None, 'fitness': None},
                },
            ],
            'selected': {'candidate_rank': 1},
        }
        trace_path = global_root / 'traces' / f'{scene.scene_id}_f{frames[0]}.json'
        trace_path.write_text(json.dumps(trace) + '\n', encoding='utf-8')
        return [{
            'global_failure_code': None,
            'global_backend_success': True,
            'final_nominal_success': False,
            'global_final_pose': {
                'x': 3.0, 'y': 0.0, 'z': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
            },
            'candidate_keyframe': 0,
            'final_xy_error_m': 1.0,
            'final_translation_3d_error_m': 1.0,
            'final_yaw_error_deg': 0.0,
            'global_wall_ms': 12.0,
            'ambiguity_valid': True,
            'ambiguity_margin': 0.03,
            'ambiguity_second_bbs_score': 0.7,
            'score_margin_normalized': 0.04,
        }]

    monkeypatch.setattr(mvp, 'run_global', fake_run_global)
    args = mvp._parser().parse_args([
        '--map-package', str(source), '--block-dir', str(blocks),
        '--output-root', str(tmp_path / 'evidence'), '--query-keyframe', '4',
        '--query-accumulation-frames', '1', '--candidate-top-k', '2',
        '--ros-install', str(tmp_path / 'ros_install'),
    ])
    evidence_dir = mvp.run_query(args)

    verified = verify_evidence_bundle(evidence_dir, source_package=source, block_dir=blocks)
    record = yaml.safe_load((evidence_dir / 'manifest.yaml').read_text(encoding='utf-8'))
    job = json.loads((evidence_dir / 'analysis' / 'analysis_job.json').read_text(encoding='utf-8'))
    assert verified['status'] == 'PASS'
    assert verified['revision_state'] == 'CURRENT'
    assert record['query']['keyframe'] == 4
    assert record['evidence']['empirical_global_result']['classification'] == 'FALSE_ACCEPT'
    assert record['evidence']['candidate_ambiguity']['status'] == 'KNOWN'
    assert record['query']['row_id'] == 'UNKNOWN'
    assert job['query_excluded_block_ids']
    assert {3, 4, 5}.issubset(set(job['query_block_excluded_keyframes']))
    assert job['display_files_are_not_analysis_inputs'] is True
    trace_reference = record['analysis_artifacts']['raw_trace']
    assert trace_reference.startswith('analysis/')
    assert (evidence_dir / trace_reference).is_file()
