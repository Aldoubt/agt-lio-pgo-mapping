from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from agt_greenhouse_annotation.point_cloud import NavigationPointCloud
from agt_greenhouse_annotation.structure_artifacts import (
    AisleProposal,
    GlobalRowProposal,
    GreenhouseStructureConfig,
    analyze_greenhouse_structure,
    decision_for_proposal,
    freeze_accepted_proposals,
    load_proposal_artifact,
    make_boundary_aisle_proposal,
    merge_row_proposals,
    proposal_aisles_for_rows,
    save_proposal_revision,
    split_row_proposal,
)
from agt_greenhouse_annotation.structure_artifacts import _build_global_rows, _build_local_rows
from agt_greenhouse_annotation.navigation_structure import RowModel
from agt_greenhouse_annotation.navigation_corridor import CorridorRefinementConfig
from agt_greenhouse_annotation.topology import label_pose, load_topology


def _synthetic_cloud():
    xs = np.arange(.05, 8.0, .10)
    ys = np.arange(.05, 4.2, .10)
    rows = []
    for y in ys:
        for x in xs:
            rows.extend([(x, y, 0.0, 1.0)] * 3)
            if any(abs(y - center) < .11 for center in (.8, 1.8, 2.8, 3.8)):
                rows.extend([(x, y, .5, 2.0)] * 3)
    values = np.asarray(rows, dtype=np.float64)
    return NavigationPointCloud(values[:, :3])


def _manual_row(auto_id, y):
    return GlobalRowProposal(auto_id, (1.0, 0.0), y,
                             ((0.0, y), (8.0, y)), .22, .8, .9,
                             {'fixture': True})


def test_global_local_chains_repeat_deterministically_and_preserve_layered_terrain(tmp_path):
    cloud = _synthetic_cloud()
    first = analyze_greenhouse_structure(cloud, 'sha256:fixture', mode='COMPARE')
    second = analyze_greenhouse_structure(cloud, 'sha256:fixture', mode='COMPARE')
    assert first.analysis_hash == second.analysis_hash
    assert first.diagnostics['global_local_are_independent'] is True
    assert first.diagnostics['compare_fusion'] is False
    assert len(first.global_rows) >= 3
    assert len(first.local_rows) >= 3
    assert first.terrain.ridge_evidence.shape == first.navigation.occupancy.shape
    assert first.terrain.depression_evidence.shape == first.navigation.occupancy.shape
    assert first.terrain.step_evidence.shape == first.navigation.occupancy.shape
    assert first.terrain.ground_valid.dtype == bool
    assert not np.shares_memory(first.terrain.ridge_evidence, first.terrain.depression_evidence)

    # Display density is a view-only operation; the detector receives the full cloud.
    from agt_greenhouse_annotation.review_3d import deterministic_display_sample
    preview_a = deterministic_display_sample(cloud.xyz(), 40)
    preview_b = deterministic_display_sample(cloud.xyz(), 90)
    assert len(preview_a) == 40 and len(preview_b) == 90
    assert first.analysis_hash == analyze_greenhouse_structure(
        cloud, 'sha256:fixture', mode='COMPARE'
    ).analysis_hash

    one = save_proposal_revision(first, tmp_path, {
        'backend_id': 'point_lio', 'backend_commit': 'fixture-backend-commit',
        'rosbag_metadata_sha256': 'fixture-rosbag-hash',
        'mapping_repository_commit': 'fixture-mapping-commit',
    })
    two = save_proposal_revision(second, tmp_path)
    assert one != two
    assert load_proposal_artifact(one)['analysis_hash'] == load_proposal_artifact(two)['analysis_hash']
    assert load_proposal_artifact(one)['revision'] == 1
    assert load_proposal_artifact(two)['revision'] == 2
    provenance = load_proposal_artifact(one)['provenance']
    assert provenance['backend_id'] == 'point_lio'
    assert provenance['legacy_source_commit'] == '42fff086e41109c74966b79d84cb46c6bbfb37dc'
    assert provenance['annotation_revision'] == 1


def test_row_review_actions_and_revision_artifact_are_explicit(tmp_path):
    first, second = _manual_row('AUTO-G001', 1.0), _manual_row('AUTO-G002', 2.0)
    merged = merge_row_proposals([first, second], 'AUTO-M001')
    assert merged.auto_id == 'AUTO-M001' and merged.decision == 'pending'
    left, right = split_row_proposal(merged, first_auto_id='AUTO-S001A', second_auto_id='AUTO-S001B')
    assert left.centerline_xy[1] == right.centerline_xy[0]
    accepted = decision_for_proposal(first, 'accepted', physical_row_id='R01', direction='forward')
    assert accepted.auto_id == 'AUTO-G001'
    assert accepted.physical_row_id == 'R01'
    assert accepted.confirmed_direction == 'forward'
    with pytest.raises(ValueError, match='manually assigned'):
        decision_for_proposal(first, 'accepted', physical_row_id=None, direction='forward')


def test_only_adjacent_accepted_rows_freeze_interior_aisles_and_topk_consumes_schema_v1(tmp_path):
    rows = [_manual_row('AUTO-G001', 1.0), _manual_row('AUTO-G002', 2.0), _manual_row('AUTO-G003', 3.0)]
    aisles = proposal_aisles_for_rows(
        rows, GreenhouseStructureConfig(corridor=CorridorRefinementConfig(aisle_minimum_width_m=.30))
    )
    assert [aisle.auto_id for aisle in aisles] == ['AUTO-A001', 'AUTO-A002']
    with pytest.raises(ValueError, match='both adjacent'):
        freeze_accepted_proposals(
            {'schema_version': 1, 'source': {}, 'assignment': {},
             'annotation': {'annotator': 'manual', 'absolute_ground_truth': False}},
            [decision_for_proposal(rows[0], 'accepted', physical_row_id='R01', direction='forward'),
             decision_for_proposal(rows[2], 'accepted', physical_row_id='R03', direction='reverse')],
            [replace(aisles[0], decision='accepted', physical_aisle_id='A01')],
            tmp_path / 'invalid.yaml')

    reviewed = [
        decision_for_proposal(rows[0], 'accepted', physical_row_id='R01', direction='forward'),
        decision_for_proposal(rows[1], 'accepted', physical_row_id='R02', direction='reverse'),
    ]
    aisle = replace(aisles[0], decision='accepted', physical_aisle_id='A01')
    base = {
        'schema_version': 1,
        'source': {'backend_id': 'point_lio', 'map_frame': 'camera_init'},
        'annotation': {'status': 'draft', 'manual_review_confirmed': False,
                       'annotator': 'manual', 'absolute_ground_truth': False},
        'assignment': {'max_row_assignment_distance_m': .5},
        'rows': [], 'headlands': [], 'scenes': [], 'backend_transforms': {},
    }
    target = tmp_path / 'frozen.yaml'
    frozen = freeze_accepted_proposals(base, reviewed, [aisle], target)
    loaded = load_topology(target)
    assert loaded['schema_version'] == 1
    assert [row['confidence'] for row in loaded['rows']] == ['confirmed', 'confirmed']
    assert loaded['rows'][0]['proposal_source']['auto_id'] == 'AUTO-G001'
    assert loaded['aisles'][0]['id'] == 'A01'
    label = label_pose(frozen, 2.0, 1.1, 0.0)
    assert label['physical_row_id'] == 'R01'
    assert label['along_row_s_m'] == pytest.approx(2.0)
    assert label['lateral_d_m'] == pytest.approx(.1)
    assert label['relative_heading_deg'] == pytest.approx(0.0)
    with pytest.raises(FileExistsError, match='immutable'):
        freeze_accepted_proposals(base, reviewed, [aisle], target)

    from agt_map_localization_benchmark.topk_ambiguity_analysis import analyze_trace
    candidates = []
    for rank, (x, y) in enumerate(((2.0, 2.0), (2.1, 1.1), (4.0, 1.1)), start=1):
        candidates.append({
            'rank': rank, 'patch': f'{rank}.pcd', 'keyframe': rank,
            'descriptor': {'ring_distance': .1 * rank, 'sector_similarity': 1.0 - .1 * rank,
                           'sector_shift': 0, 'yaw_seed_deg': 0},
            'map_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw_deg': 0.0},
            'bbs': {'attempted': False, 'valid': None, 'timed_out': False,
                    'coarse_pose': None, 'score': None, 'elapsed_ms': None},
            'gicp': {'attempted': False, 'converged': None, 'fitness': None,
                     'overlap': None, 'final_pose': None},
        })
    trace = {
        'schema_version': 1,
        'query': {'scan': 'query.pcd', 'frames': 1, 'timestamp': 10.0, 'scene_id': 'SYNTH',
                  'scene_type': 'ROW_MIDDLE', 'keyframe': 0, 'backend_id': 'point_lio',
                  'reference_pose': {'x': 2.0, 'y': 1.0, 'z': 0.0, 'yaw_deg': 0.0}},
        'descriptor': {'database_size': 3, 'prefilter': 3, 'candidate_top_k': 3,
                       'params': {}, 'score_semantics': {'ring_distance': 'lower_is_better',
                                                        'sector_similarity': 'higher_is_better'}},
        'ranked_candidates': candidates,
        'selected': {'candidate_rank': 1, 'reason': 'synthetic fixture'},
    }
    rows_at_k, candidate_labels = analyze_trace(trace, frozen)
    assert candidate_labels[0]['candidate_row'] == 'R02'
    assert candidate_labels[1]['candidate_row'] == 'R01'
    assert rows_at_k[0]['physical_row_recall'] is False
    assert rows_at_k[1]['physical_row_recall'] is True


def test_boundary_aisle_requires_explicit_anchor_and_narrow_aisle_is_rejected(tmp_path):
    row = _manual_row('AUTO-G001', 1.0)
    boundary = make_boundary_aisle_proposal(row, (0.0, 0.0), auto_id='AUTO-AB001')
    assert boundary.aisle_kind == 'BOUNDARY'
    assert boundary.boundary_anchor_xy == (0.0, 0.0)
    narrow = _manual_row('AUTO-G002', 1.35)
    aisle = proposal_aisles_for_rows([row, narrow], GreenhouseStructureConfig())[0]
    assert aisle.geometric_width_m < aisle.minimum_width_m
    bad = replace(aisle, decision='accepted', physical_aisle_id='A01')
    with pytest.raises(ValueError, match='narrower'):
        freeze_accepted_proposals(
            {'schema_version': 1, 'source': {}, 'assignment': {},
             'annotation': {'annotator': 'manual', 'absolute_ground_truth': False}},
            [decision_for_proposal(row, 'accepted', physical_row_id='R01', direction='forward'),
             decision_for_proposal(narrow, 'accepted', physical_row_id='R02', direction='forward')],
            [bad], tmp_path / 'narrow.yaml')


def test_global_proposal_endpoints_follow_row_support_instead_of_map_outliers():
    point_count = np.ones((20, 100), dtype=np.int32)
    regularized = np.zeros_like(point_count, dtype=bool)
    regularized[10, 30:70] = True
    nav = SimpleNamespace(origin_x_m=0.0, origin_y_m=0.0, width=100, height=20,
                          resolution_m=.1, point_count=point_count)
    row_model = RowModel(np.array([1.0, 0.0]), 0.0, (1.05,), .22, (.8,))
    result = SimpleNamespace(row_model=row_model, row_regularized_obstacle=regularized,
                             row_support=np.ones_like(point_count, dtype=np.float64))

    rows = _build_global_rows(result, nav)
    assert len(rows) == 1
    x0, x1 = rows[0].centerline_xy[0][0], rows[0].centerline_xy[1][0]
    assert x0 == pytest.approx(3.05)
    assert x1 == pytest.approx(6.95)


def test_aisle_proposal_reports_side_clearance_and_rejects_missing_overlap():
    first = _manual_row('AUTO-G001', 1.0)
    second = replace(_manual_row('AUTO-G002', 2.6), centerline_xy=((9.0, 2.6), (10.0, 2.6)))
    aisle = proposal_aisles_for_rows([first, second], GreenhouseStructureConfig())[0]
    assert aisle.geometric_width_m == pytest.approx(1.6 - .44 - .24)
    assert aisle.diagnostics['side_clearance_reserved_m'] == pytest.approx(.24)
    assert aisle.diagnostics['longitudinal_overlap_m'] == 0.0
    assert aisle.diagnostics['status'] == 'REJECTED_NO_LONGITUDINAL_OVERLAP'
    assert aisle.centerline_xy == ()


def test_local_track_proposal_preserves_curved_observation_polyline():
    observations = [
        SimpleNamespace(u_center_m=1.0, v_center_m=2.0, window_index=0),
        SimpleNamespace(u_center_m=2.0, v_center_m=2.4, window_index=1),
        SimpleNamespace(u_center_m=3.0, v_center_m=2.1, window_index=2),
    ]
    track = SimpleNamespace(row_id=7, representative_v_m=2.2, longitudinal_span_m=2.0,
                            mean_support=.8, observations=observations)
    result = SimpleNamespace(tracks=[track], config=SimpleNamespace(row_structural_half_width_m=.2))
    navigation = SimpleNamespace(width=40, height=40, origin_x_m=0.0, origin_y_m=0.0,
                                 resolution_m=.1, point_count=np.ones((40, 40), dtype=np.int32))

    proposals = _build_local_rows(result, np.array([1.0, 0.0]), navigation)
    assert len(proposals) == 1
    assert len(proposals[0].centerline_xy) == 3
    assert proposals[0].centerline_xy[1][1] == pytest.approx(2.4)


def test_smoke_reference_topology_must_match_map_hash(tmp_path):
    from agt_greenhouse_annotation.workbench import _reference_topology_summary

    map_path = tmp_path / 'map_package'
    map_path.mkdir()
    topology_path = tmp_path / 'greenhouse_topology.yaml'
    topology_path.write_text(yaml.safe_dump({
        'schema_version': 1,
        'source': {'map_package': str(map_path), 'map_package_manifest_sha256': 'fixture-sha'},
        'annotation': {'status': 'draft', 'manual_review_confirmed': False},
        'rows': [], 'headlands': [], 'scenes': [],
    }), encoding='utf-8')
    package = SimpleNamespace(path=map_path.resolve(), manifest_sha256='fixture-sha')

    summary = _reference_topology_summary(topology_path, package)
    assert summary['map_package_manifest_sha256'] == 'fixture-sha'
    assert (summary['physical_row_count'], summary['headland_count'], summary['scene_count']) == (0, 0, 0)
    package = SimpleNamespace(path=map_path.resolve(), manifest_sha256='different-sha')
    with pytest.raises(ValueError, match='manifest SHA-256'):
        _reference_topology_summary(topology_path, package)


def test_wall_exclusion_removes_rows_in_both_sources_preserving_obstacles():
    cloud = _synthetic_cloud()
    base_config = GreenhouseStructureConfig()
    baseline = analyze_greenhouse_structure(cloud, 'synthetic', config=base_config, row_direction_xy=(1., 0.))
    polygon = ((-.1, -.1), (8.1, -.1), (8.1, 1.15), (-.1, 1.15))
    config = replace(base_config, row_exclusion_polygons_xy=(polygon,))
    masked = analyze_greenhouse_structure(cloud, 'synthetic', config=config, row_direction_xy=(1., 0.))
    assert any(row.lateral_v_m < 1.15 for row in baseline.global_rows)
    assert any(row.lateral_v_m < 1.15 for row in baseline.local_rows)
    assert masked.global_rows and masked.local_rows
    assert all(row.lateral_v_m > 1.15 for row in masked.global_rows)
    assert all(row.lateral_v_m > 1.15 for row in masked.local_rows)
    np.testing.assert_array_equal(masked.navigation.occupancy, baseline.navigation.occupancy)
    np.testing.assert_array_equal(masked.navigation.obstacle_count, baseline.navigation.obstacle_count)
    assert masked.diagnostics['row_excluded_cell_count'] > 0
    assert masked.analysis_hash != baseline.analysis_hash
    roundtrip = GreenhouseStructureConfig.from_mapping(config.to_dict())
    assert roundtrip.row_exclusion_polygons_xy == config.row_exclusion_polygons_xy
