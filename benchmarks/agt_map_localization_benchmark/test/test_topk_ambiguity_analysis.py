"""Independent examples for retrieval truth, unknown labels, and censoring."""
from copy import deepcopy
import csv
import json
import math
import hashlib
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_map_localization_benchmark.topk_ambiguity_analysis import (
    analyze_trace, canonical_pose, paired_effect, row_entropy, run_analysis,
    scene_bootstrap, scene_effects, score_margin, validate_trace,
)


def topology():
    return {
        'schema_version': 1,
        'source': {'backend_id': 'point_lio', 'map_frame': 'map'},
        'annotation': {'status': 'frozen', 'manual_review_confirmed': True},
        'assignment': {'max_row_assignment_distance_m': .5},
        'rows': [
            {'id': 'R01', 'centerline': [[0, 0], [12, 0]], 'nominal_width_m': 1., 'confidence': 'confirmed'},
            {'id': 'R02', 'centerline': [[0, 3], [12, 3]], 'nominal_width_m': 1., 'confidence': 'confirmed'},
        ],
        'headlands': [{'id': 'H01', 'polygon': [[-3, -1], [-1, -1], [-1, 4], [-3, 4]], 'confidence': 'confirmed'}],
        'backend_transforms': {},
    }


def pose(x, y=0, yaw=0):
    return {'x': x, 'y': y, 'z': 0., 'yaw_deg': yaw}


def trace():
    candidates = []
    # Rank 1: wrong physical row; rank 2: correct row but 6 m longitudinal error;
    # rank 3: correct row and 0.5 m error. Remaining entries are unlabelled.
    positions = [(2, 3), (8, 0), (2.5, 0)] + [(30+i, 20) for i in range(7)]
    for rank, (x, y) in enumerate(positions, 1):
        candidates.append({
            'rank': rank, 'patch': f'{rank}.pcd', 'keyframe': rank,
            'descriptor': {'ring_distance': .1*rank, 'sector_similarity': 1.-.05*rank,
                           'sector_shift': 0, 'yaw_seed_deg': 0},
            'map_pose': pose(x, y),
            'bbs': {'attempted': False, 'valid': None, 'timed_out': False,
                    'coarse_pose': None, 'score': None, 'elapsed_ms': None},
            'gicp': {'attempted': False, 'converged': None, 'fitness': None,
                     'overlap': None, 'final_pose': None},
        })
    return {'schema_version': 1,
            'query': {'scan': 'query.pcd', 'frames': 1, 'timestamp': 100., 'scene_id': 'M01',
                      'scene_type': 'ROW_MIDDLE', 'keyframe': 0, 'backend_id': 'point_lio',
                      'reference_pose': pose(2)},
            'descriptor': {'database_size': 10, 'prefilter': 40, 'candidate_top_k': 10,
                           'params': {}, 'score_semantics': {'ring_distance': 'lower_is_better',
                                                            'sector_similarity': 'higher_is_better'}},
            'ranked_candidates': candidates, 'selected': {'candidate_rank': 1, 'reason': 'native result'}}


def test_schema_rejects_ordering_selected_rank_and_score_semantics():
    validate_trace(trace())
    for mutation in ('ordering', 'selected', 'semantics', 'nonfinite'):
        bad = trace()
        if mutation == 'ordering': bad['ranked_candidates'][0]['rank'] = 2
        elif mutation == 'selected': bad['selected']['candidate_rank'] = 11
        elif mutation == 'semantics': bad['descriptor']['score_semantics']['sector_similarity'] = 'lower_is_better'
        else: bad['ranked_candidates'][0]['descriptor']['sector_similarity'] = float('nan')
        with pytest.raises(ValueError): validate_trace(bad)


def test_candidate_row_mapping_recall_wrong_row_and_longitudinal_truth():
    rows, candidates = analyze_trace(trace(), topology())
    at1, at3, at5, at10 = rows
    assert [c['candidate_row'] for c in candidates[:3]] == ['R02', 'R01', 'R01']
    assert candidates[1]['delta_s_m'] == pytest.approx(6.)
    assert candidates[2]['delta_s_m'] == pytest.approx(.5)
    assert at1['physical_row_recall'] is False
    assert at1['top1_wrong_row'] is True
    assert at3['physical_row_recall'] is True
    assert at3['pose_region_recall_1m'] is True
    assert at3['wrong_row_fraction'] == pytest.approx(1/3)
    assert at10['wrong_row_fraction'] is None
    assert at10['wrong_row_fraction_known_rows'] == pytest.approx(1/3)
    assert at10['wrong_row_fraction_lower_bound'] == pytest.approx(.1)
    assert at10['wrong_row_fraction_upper_bound'] == pytest.approx(.8)
    assert at10['wrong_row_unknown_count'] == 7
    assert at3['longitudinal_median_m'] == pytest.approx(3.25)
    assert at3['longitudinal_p95_m'] == pytest.approx(5.725)
    for suffix in ('1m', '2m', '5m'):
        assert at3[f'longitudinal_fraction_gt_{suffix}'] == pytest.approx(.5)
    assert at3['s_spread_m'] == pytest.approx(5.5)
    assert at3['known_candidate_row_count'] == 2


def test_unknown_candidates_cannot_be_declared_wrong_or_known_recall_miss():
    data = trace(); data['ranked_candidates'][0]['map_pose'] = pose(30, 20)
    rows, _ = analyze_trace(data, topology())
    assert rows[0]['physical_row_recall'] is None
    assert rows[0]['wrong_row_fraction'] is None
    assert rows[0]['top1_wrong_row'] is None
    data['query']['reference_pose'] = pose(30, 20)
    rows, _ = analyze_trace(data, topology())
    assert all(r['physical_row_recall'] is None for r in rows)
    assert all(r['pose_region_recall_2m'] is None for r in rows)


def test_headland_recall_is_region_metric_and_never_fabricates_row():
    data = trace(); data['query']['scene_type'] = 'HEADLAND'; data['query']['reference_pose'] = pose(-2, 0)
    data['ranked_candidates'][0]['map_pose'] = pose(-2, 1)
    rows, _ = analyze_trace(data, topology())
    assert rows[0]['query_row'] == 'UNKNOWN'
    assert rows[0]['physical_row_recall'] is None
    assert rows[0]['wrong_row_fraction'] is None
    assert rows[0]['headland_region_recall'] is True
    assert rows[0]['xy_region_recall_2m'] is True
    middle_rows, _ = analyze_trace(trace(), topology())
    assert middle_rows[0]['headland_region_recall'] is None


def test_entropy_uses_only_known_physical_rows_with_disclosed_coverage():
    rows, _ = analyze_trace(trace(), topology())
    expected = -(1/3)*math.log(1/3)-(2/3)*math.log(2/3)
    assert rows[-1]['row_entropy'] == pytest.approx(expected)
    assert rows[-1]['effective_row_count'] == pytest.approx(math.exp(expected))
    assert rows[-1]['entropy_known_weight_fraction'] == pytest.approx(.3)
    candidates = [
        {'label': {'physical_row_id': row, 'label_confidence': 'confirmed'}, 'descriptor': {'sector_similarity': score}}
        for row, score in [('R01', 1.), ('R02', 0.)]]
    with pytest.raises(ValueError, match='fixed positive tau'): row_entropy(candidates, 'softmax')
    weighted = row_entropy(candidates, 'softmax', 1.)
    p = math.e/(math.e+1)
    assert weighted['row_entropy'] == pytest.approx(-p*math.log(p)-(1-p)*math.log(1-p))


def test_score_margin_never_confuses_similarity_and_ring_distance():
    assert score_margin(.8, .6) == pytest.approx(.25)
    assert score_margin(.2, .3, higher_is_better=False) == pytest.approx(.5)
    assert score_margin(None, .3) is None
    rows, _ = analyze_trace(trace(), topology())
    assert rows[0]['score_margin_m12'] == pytest.approx((.95-.90)/.95)


def test_unattempted_stage_is_censored_not_failure_and_runtime_policy_unknown():
    data = trace()
    data['ranked_candidates'][0]['bbs'].update(attempted=True, valid=True, coarse_pose=pose(20, 0))
    data['ranked_candidates'][1]['bbs'].update(attempted=True, valid=False, timed_out=True)
    data['ranked_candidates'][2]['bbs'].update(attempted=True, valid=True, coarse_pose=pose(2.2))
    data['ranked_candidates'][2]['gicp'].update(attempted=True, converged=True, final_pose=pose(2.1))
    data['acceptance'] = {'owner': 'offline_reference_tolerance', 'nominal_success': True}
    rows, _ = analyze_trace(data, topology())
    assert rows[0]['bbs_basin_hit'] is False
    assert rows[1]['bbs_basin_hit'] is True
    assert rows[1]['bbs_observed_count'] == 2
    assert rows[1]['bbs_censored_count'] == 1
    assert rows[0]['gicp_final_success'] is None
    assert rows[1]['gicp_final_success'] is True
    assert rows[1]['gicp_observed_count'] == 1
    assert rows[1]['gicp_censored_count'] == 2
    assert rows[1]['offline_reference_nominal_success'] is True
    assert rows[1]['final_accepted_success'] is None


def test_coarse_and_gicp_pose_tolerances_are_independent_fixed_proxies():
    data = trace()
    candidate = data['ranked_candidates'][0]
    candidate['bbs'].update(attempted=True, valid=True, coarse_pose=pose(2.9, yaw=9))
    candidate['gicp'].update(attempted=True, converged=True, final_pose=pose(2.6, yaw=4))
    rows, _ = analyze_trace(data, topology())
    assert rows[0]['bbs_basin_hit'] is True
    assert rows[0]['gicp_final_success'] is False
    candidate['gicp']['final_pose'] = pose(2.1, yaw=6)
    assert analyze_trace(data, topology())[0][0]['gicp_final_success'] is False


def test_backend_transform_required_even_when_frames_share_name():
    data = trace(); data['query']['backend_id'] = 'fast_livo2_lio'
    assert analyze_trace(data, topology())[0][0]['physical_row_recall'] is None
    topo = topology()
    topo['backend_transforms']['fast_livo2_lio'] = {'matrix': np.eye(4).tolist()}
    assert analyze_trace(data, topo)[0][0]['physical_row_recall'] is False
    matrix = np.eye(4); matrix[:2, :2] = [[0, -1], [1, 0]]; matrix[0, 3] = 10
    topo['backend_transforms']['fast_livo2_lio']['matrix'] = matrix.tolist()
    actual = canonical_pose(topo, 'fast_livo2_lio', pose(2, 3, 20))
    assert actual['x'] == pytest.approx(7.)
    assert actual['y'] == pytest.approx(2.)
    assert actual['yaw_deg'] == pytest.approx(110.)


def test_keyframe_csv_is_consistency_check_not_stale_label_override():
    rows, _ = analyze_trace(trace(), topology(), labels={
        ('point_lio','1'): {'physical_row_id':'R02','zone_type':'ROW','headland_id':'UNKNOWN',
                           'along_row_s_m':2.,'lateral_d_m':0.,'relative_heading_deg':0.,
                           'label_confidence':'confirmed'}})
    assert rows[0]['top1_wrong_row'] is True
    with pytest.raises(ValueError, match='stale/inconsistent'):
        analyze_trace(trace(), topology(), labels={
            ('point_lio','1'): {'physical_row_id':'R01','zone_type':'ROW','headland_id':'UNKNOWN',
                               'along_row_s_m':2.,'label_confidence':'confirmed'}})


def test_scene_bootstrap_uses_scenes_and_not_candidate_or_repeat_count():
    rows = [{'scene_id': 'one', 'metric': 0.}]*100 + [{'scene_id': 'two', 'metric': 1.}]
    lo, hi = scene_bootstrap(rows, 'metric', 1000, np.random.default_rng(7))
    assert (lo, hi) == pytest.approx((0., 1.))
    assert scene_bootstrap(rows[:100], 'metric', 1000, np.random.default_rng(7)) == (None, None)
    paired=[]
    for scene_id in ('s1', 's2'):
        for frames, metric in [(1, 0.), (5, 1.)]:
            paired.append(dict(scene_id=scene_id,scene_type='ROW_MIDDLE',backend_id='point_lio',k=10,frames=frames,metric=metric))
    effect=paired_effect(paired,['metric'],'frames',1000,7)[0]
    assert effect['paired_scene_count'] == 2
    assert effect['mean_difference'] == pytest.approx(1.)
    assert effect['ci95_lower'] == pytest.approx(1.)
    contrasts=[dict(r,scene_type='HEADLAND',scene_id='h'+r['scene_id'],metric=2.) for r in paired]
    differences=scene_effects(paired+contrasts,['metric'],1000,7)
    assert differences[0]['mean_difference'] == pytest.approx(2.)


def test_pipeline_outputs_and_provenance_with_draft_smoke(tmp_path: Path):
    trace_path=tmp_path/'trace.json';trace_path.write_text(json.dumps(trace()))
    topology_path=tmp_path/'topology.yaml';topology_path.write_text(yaml.safe_dump(topology()))
    before=topology_path.read_bytes()
    with pytest.raises(ValueError,match='topology/provenance validation failed'):
        run_analysis([trace_path],topology_path,tmp_path/'invalid_production',bootstrap_samples=100,figures=False)
    manifest=run_analysis([trace_path],topology_path,tmp_path/'analysis',bootstrap_samples=100,figures=False,allow_draft=True)
    assert manifest['query_count']==1 and manifest['candidate_count']==10
    assert topology_path.read_bytes()==before
    expected={'table_topk_recall_by_scene.csv','table_wrong_row_by_scene.csv',
              'table_longitudinal_ambiguity.csv','table_candidate_entropy.csv','table_score_margin.csv',
              'table_bbs_gicp_stage_success.csv','table_multiframe_topk.csv','table_cross_backend_topk.csv'}
    assert expected <= {p.name for p in (tmp_path/'analysis'/'tables').glob('*.csv')}
    assert len(manifest['topology']['sha256'])==64 and manifest['topology']['read_only'] is True
    draft=topology();draft['annotation'].update(status='draft',manual_review_confirmed=False)
    topology_path.write_text(yaml.safe_dump(draft))
    with pytest.raises(ValueError,match='frozen, manually confirmed'):
        run_analysis([trace_path],topology_path,tmp_path/'draft',figures=False)
    run_analysis([trace_path],topology_path,tmp_path/'draft',figures=False,allow_draft=True)
    with (tmp_path/'draft'/'tables'/'query_metrics.csv').open() as handle:
        rows=list(csv.DictReader(handle))
    assert rows[0]['physical_row_recall']==''
    assert rows[0]['wrong_row_fraction']==''
    assert rows[0]['row_entropy']==''
    assert rows[0]['score_margin_m12']!=''
    assert rows[0]['mean_radius_m']!=''


def test_frozen_production_verifies_real_manifest_and_rejects_byte_tampering(tmp_path: Path):
    from agt_greenhouse_annotation.topology import load_map_package, new_topology, save_topology
    package=tmp_path/'map_package';package.mkdir();(package/'patches').mkdir()
    (package/'map.pcd').write_text('synthetic provenance-test fixture, not physical data\n')
    positions=[(2.,0.)]+[(c['map_pose']['x'],c['map_pose']['y']) for c in trace()['ranked_candidates']]
    lines=[]
    for index,(x,y) in enumerate(positions):
        (package/'patches'/f'{index}.pcd').write_text('synthetic patch\n')
        lines.append(f'{index}.pcd {100.+index} {x} {y} 0 1 0 0 0\n')
    (package/'poses_timed.txt').write_text(''.join(lines))
    (package/'metadata.yaml').write_text(yaml.safe_dump({'frames':{'map':'map'},
        'mapping_backend':{'id':'point_lio','source_commit':'SYNTHETIC_TEST_ONLY'},'source':{'rosbag':None}}))
    checksums={str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in package.rglob('*') if p.is_file()}
    (package/'manifest.yaml').write_text(yaml.safe_dump({'files':checksums}))
    loaded=load_map_package(package)
    frozen=new_topology(loaded)
    frozen['rows']=topology()['rows'];frozen['headlands']=topology()['headlands']
    for row in frozen['rows']:row['direction']='bidirectional'
    frozen['annotation'].update(status='frozen',manual_review_confirmed=True)
    topology_path=tmp_path/'greenhouse_topology.yaml'
    save_topology(frozen,topology_path,loaded)
    trace_path=tmp_path/'trace.json';trace_path.write_text(json.dumps(trace()))
    result=run_analysis([trace_path],topology_path,tmp_path/'valid',figures=False)
    assert result['topology_validation']['valid'] is True
    assert len(result['analysis_source_provenance']['analysis_source_sha256'])==64
    changed=yaml.safe_load(topology_path.read_text());changed['rows'][0]['notes']='changed after freeze'
    topology_path.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='annotation file hash/provenance mismatch'):
        run_analysis([trace_path],topology_path,tmp_path/'tampered',figures=False)


def test_all_seven_paper_figures_can_render(tmp_path: Path):
    pytest.importorskip('matplotlib')
    data=trace();headland=deepcopy(data)
    headland['query'].update(scene_id='H01',scene_type='HEADLAND',reference_pose=pose(-2,0))
    trace_path=tmp_path/'traces.json';trace_path.write_text(json.dumps([data,headland]))
    topology_path=tmp_path/'topology.yaml';topology_path.write_text(yaml.safe_dump(topology()))
    run_analysis([trace_path],topology_path,tmp_path/'analysis',bootstrap_samples=10,allow_draft=True)
    figures=list((tmp_path/'analysis'/'figures').glob('*.png'))
    assert len(figures)==7
    assert all(p.stat().st_size > 1000 for p in figures)
