"""Predeclared offline perturbations, error definitions, descriptive statistics."""
from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr

# Predeclared 3D-translation / heading-yaw criteria, NOT product safety gates.
# Z is already included in 3D translation; an additional Z cutoff would silently
# tighten the requested thresholds and is therefore deliberately not applied.
SUCCESS_RULES = {
    'strict': {'translation_3d_m': 0.20, 'yaw_deg': 2.0},
    'nominal': {'translation_3d_m': 0.50, 'yaw_deg': 5.0},
    'loose': {'translation_3d_m': 1.00, 'yaw_deg': 10.0},
}
FAILURE_CODES = (
    'NO_CONVERGENCE', 'WRONG_BASIN', 'INSUFFICIENT_MAP_POINTS',
    'INSUFFICIENT_QUERY_POINTS', 'MAX_ITERATIONS', 'INVALID_RESULT', 'TIMEOUT',
    'REFERENCE_UNAVAILABLE', 'UNKNOWN_FAILURE', 'FALSE_RELOCALIZATION',
)


def perturbations() -> tuple[dict, ...]:
    """Fixed map-frame XYZ translations, Z yaw, and combined starts."""
    configs = [{'name': 'zero', 'dxyz_m': (0.0, 0.0, 0.0), 'dyaw_deg': 0.0}]
    for axis in range(3):
        for value in (0.25, 0.5, 1.0, 2.0):
            shift = [0.0] * 3
            shift[axis] = value
            configs.append({'name': f'{"xyz"[axis]}_{value:g}m',
                            'dxyz_m': tuple(shift), 'dyaw_deg': 0.0})
    for angle in (5.0, 10.0, 20.0, 45.0):
        configs.append({'name': f'yaw_{angle:g}deg',
                        'dxyz_m': (0.0, 0.0, 0.0), 'dyaw_deg': angle})
    configs += [
        {'name': 'x_1m_yaw_20deg', 'dxyz_m': (1.0, 0.0, 0.0), 'dyaw_deg': 20.0},
        {'name': 'y_2m_yaw_45deg', 'dxyz_m': (0.0, 2.0, 0.0), 'dyaw_deg': 45.0},
        {'name': 'z_1m_yaw_10deg', 'dxyz_m': (0.0, 0.0, 1.0), 'dyaw_deg': 10.0},
    ]
    return tuple(configs)


def initial_pose(t: np.ndarray, xyzw: np.ndarray, perturb: dict) -> tuple[np.ndarray, np.ndarray]:
    new_t = np.asarray(t, dtype='f8') + np.asarray(perturb['dxyz_m'], dtype='f8')
    yaw = Rotation.from_euler('z', perturb['dyaw_deg'], degrees=True)
    new_q = (yaw * Rotation.from_quat(xyzw)).as_quat()
    return new_t, new_q


def pose_record(t: np.ndarray, xyzw: np.ndarray) -> dict:
    return {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
            'qx': float(xyzw[0]), 'qy': float(xyzw[1]),
            'qz': float(xyzw[2]), 'qw': float(xyzw[3])}


def measure_pose(reference: dict, native: dict) -> dict:
    """No pose/error is inferred when the native API did not return a pose."""
    pose = native.get('pose')
    out = {'translation_3d_error_m': None, 'xy_error_m': None, 'z_error_m': None,
           'yaw_error_deg': None, 'so3_error_deg': None,
           'error_xyz_map_m': None}
    out.update({f'{rule}_success': False for rule in SUCCESS_RULES})
    if pose is None:
        return out
    ref_t = np.array([reference[k] for k in ('x', 'y', 'z')], dtype='f8')
    res_t = np.array([pose[k] for k in ('x', 'y', 'z')], dtype='f8')
    delta = res_t - ref_t
    ref_q = Rotation.from_quat([reference[k] for k in ('qx', 'qy', 'qz', 'qw')])
    res_q = Rotation.from_quat([pose[k] for k in ('qx', 'qy', 'qz', 'qw')])
    d_rotation = res_q * ref_q.inv()
    # Compare each body's heading in the map plane; yaw of R_result*R_ref^-1
    # is NOT generally the heading difference when roll/pitch are nonzero.
    def heading(rotation: Rotation) -> float:
        forward = rotation.apply([1., 0., 0.])
        return math.atan2(float(forward[1]), float(forward[0]))
    yaw_delta = heading(res_q) - heading(ref_q)
    out.update({'translation_3d_error_m': float(np.linalg.norm(delta)),
                'xy_error_m': float(np.linalg.norm(delta[:2])),
                'z_error_m': abs(float(delta[2])),
                'yaw_error_deg': abs(math.degrees(math.atan2(math.sin(yaw_delta),
                                                            math.cos(yaw_delta)))),
                'so3_error_deg': math.degrees(d_rotation.magnitude()),
                'error_xyz_map_m': [float(v) for v in delta]})
    for name, rule in SUCCESS_RULES.items():
        out[f'{name}_success'] = bool(native['backend_success']
            and out['translation_3d_error_m'] <= rule['translation_3d_m']
            and out['yaw_error_deg'] <= rule['yaw_deg'])
    return out


def classify_failure(row: dict) -> str | None:
    """Offline reference-aware case outcome, never inferred from native fitness.

    A native iteration-limit failure is only MAX_ITERATIONS if the backend
    *says so*: its current CLI does not expose an iteration count.  GLOBAL
    FALSE_RELOCALIZATION means the returned GLOBAL pose is wrong relative to
    the declared same-session reference; it is not a negative-session
    false-positive rate.
    """
    if not row.get('reference_pose'):
        return 'REFERENCE_UNAVAILABLE'
    native = row['native']
    reason = str(native.get('backend_reason') or '').lower()
    if native.get('backend_exit_code') == 124 or 'timeout' in reason or 'timed out' in reason:
        return 'TIMEOUT'
    if native.get('backend_success'):
        if native.get('pose') is None or row.get('translation_3d_error_m') is None:
            return 'INVALID_RESULT'
        if not row.get('loose_success'):
            return 'FALSE_RELOCALIZATION' if row['algorithm'] == 'GLOBAL' else 'WRONG_BASIN'
        return None
    if 'local map has fewer' in reason or 'map too sparse' in reason or 'target_points=0' in reason:
        return 'INSUFFICIENT_MAP_POINTS'
    if 'query scan too sparse' in reason or 'query too sparse' in reason or 'scan too sparse' in reason:
        return 'INSUFFICIENT_QUERY_POINTS'
    if 'max iteration' in reason or 'iteration limit' in reason:
        return 'MAX_ITERATIONS'
    if 'did not converge' in reason or 'found no valid pose' in reason:
        return 'NO_CONVERGENCE'
    if ('invalid' in reason or 'nonfinite' in reason or 'nan' in reason
            or 'no parseable backend json' in reason or 'quaternion' in reason
            or native.get('pose') is None and native.get('backend_exit_code') == 0):
        return 'INVALID_RESULT'
    return 'UNKNOWN_FAILURE'


def distribution(rows: list[dict], field: str) -> dict:
    valid = [float(row[field]) for row in rows
             if row.get(field) is not None and math.isfinite(float(row[field]))]
    return {'available': len(valid), 'median': float(np.median(valid)) if valid else None,
            'p95': float(np.quantile(valid, 0.95)) if valid else None,
            'worst': float(max(valid)) if valid else None}


def association(rows: list[dict], value_field: str) -> dict:
    """Group repeated perturbations BEFORE Qt/recovery exploratory association."""
    groups = defaultdict(list)
    for row in rows:
        if row.get('algorithm') != 'LOCAL':
            continue
        key = (row['tier'], row['candidate'], row['center'], row['frames'])
        groups[key].append(row)
    pairs = []
    for group in groups.values():
        g = group[0]['geometry']
        if value_field == 'qt_mapping_view_median':
            q = g.get('qt_mapping_view_median')
        elif value_field == 'qr_mapping_view_median':
            q = g.get('qr_mapping_view_median')
        elif value_field == 'qr_query_conditioned':
            q = g.get('qr_query_conditioned', {}).get('q')
        else:
            q = g.get('qt_query_local', {}).get('q')
        if q is not None and math.isfinite(float(q)):
            pairs.append((float(q), sum(bool(r['nominal_success']) for r in group), len(group)))
    if len(pairs) < 4 or len({v[0] for v in pairs}) < 2:
        return {'groups': len(pairs), 'spearman_q_vs_empirical_nominal_fraction': None,
                'reliability_bins': [], 'note': 'too few distinct candidate/query groups'}
    x = np.array([v[0] for v in pairs], dtype='f8')
    y = np.array([v[1] / v[2] for v in pairs], dtype='f8')
    corr = spearmanr(x, y).correlation if len(set(y)) > 1 else np.nan
    edge = np.unique(np.quantile(x, [0, 0.25, 0.5, 0.75, 1.0]))
    bins = []
    for i in range(len(edge) - 1):
        # Ordinary half-open bins, with inclusive last bin.
        if i == len(edge) - 2:
            group = [v for v in pairs if edge[i] <= v[0] <= edge[i + 1]]
        else:
            group = [v for v in pairs if edge[i] <= v[0] < edge[i + 1]]
        hits = sum(v[1] for v in group)
        trials = sum(v[2] for v in group)
        bins.append({'q_lower': float(edge[i]), 'q_upper': float(edge[i + 1]),
                     'candidate_query_groups_not_independent': len(group),
                     'nominal_hits': hits, 'trials': trials,
                     'empirical_success_fraction': hits / trials if trials else None})
    return {'groups': len(pairs),
            'spearman_q_vs_empirical_nominal_fraction': float(corr) if math.isfinite(corr) else None,
            'reliability_bins': bins,
            'note': 'shared session and nested maps: groups NOT independent, NOT calibrated P(success)'}


def _weak_direction_summary(group: list[dict]) -> dict:
    """Condition on actual nonzero starts; weak axes have arbitrary sign."""
    result = {}
    for axis in ('translation_query', 'translation_mapping',
                 'rotation_query', 'rotation_mapping'):
        a = f'{axis}_weak_alignment_abs_cos'
        recovery = f'{axis}_weak_recovery_{"deg" if axis.startswith("rotation_") else "m"}'
        bins = {}
        for label, predicate in (
            ('weak_aligned', lambda x: x >= .80),
            ('strong_aligned', lambda x: x <= .33),
            ('intermediate', lambda x: .33 < x < .80),
        ):
            subset = [row for row in group
                      if row['weak_alignment'].get(a) is not None
                      and predicate(row['weak_alignment'][a])]
            bins[label] = {
                'nominal_success': sum(bool(row['nominal_success']) for row in subset),
                'total': len(subset),
                'weak_projection_recovery': distribution([row['weak_alignment'] for row in subset], recovery),
            }
        result[axis] = {'bins': bins, 'axis_semantics':
                        'mapping-era per-voxel axial consensus' if axis.endswith('mapping') else
                        'query-origin candidate-map spectrum'}
    return result


def _density_control_comparison(tables: list[dict]) -> list[dict]:
    """Matched point count alone does not match occupied spatial footprint."""
    by_key = {(t['tier'], t['algorithm'], t['frames'], t['candidate']): t for t in tables}
    paired = []
    for item in tables:
        if item['algorithm'] != 'LOCAL' or not item['candidate'].startswith('D_QT_'):
            continue
        suffix = item['candidate'].removeprefix('D_QT_')
        for kind in ('CONTROL_RANDOM', 'CONTROL_VOXEL'):
            other = by_key.get((item['tier'], 'LOCAL', item['frames'], f'{kind}_{suffix}'))
            if other is None:
                continue
            d, c = item['density_and_coverage'], other['density_and_coverage']
            paired.append({
                'tier': item['tier'], 'frames': item['frames'], 'qt_candidate': item['candidate'],
                'control': other['candidate'], 'points_match': d['points'] == c['points'],
                'points_each': d['points'],
                'qt_xy_occupied': d['occupied_xy_1m_cells'],
                'control_xy_occupied': c['occupied_xy_1m_cells'],
                'qt_xyz_occupied': d['occupied_xyz_1m_cells'],
                'control_xyz_occupied': c['occupied_xyz_1m_cells'],
                'qt_nominal_success': item['success_counts']['nominal'],
                'control_nominal_success': other['success_counts']['nominal'],
                'qt_median_error_m': item['translation_3d_error_m']['median'],
                'control_median_error_m': other['translation_3d_error_m']['median'],
                'qt_median_wall_ms': item['wall_ms_external']['median'],
                'control_median_wall_ms': other['wall_ms_external']['median'],
                'note': 'observational matched-point-count comparison, NOT automatically selected winner',
            })
    return paired


def summarize(rows: list[dict], candidates: dict, manifest: dict) -> dict:
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['tier'], row['algorithm'], row['candidate'], row['frames'])].append(row)
    tables = []
    for (tier, algo, name, frames), group in sorted(grouped.items()):
        total = len(group)
        codes = {key: sum(r.get('failure_code') == key for r in group) for key in FAILURE_CODES}
        metrics = {
            'tier': tier, 'algorithm': algo, 'candidate': name, 'frames': frames,
            'success_counts': {criterion: {
                'success': sum(bool(r[f'{criterion}_success']) for r in group),
                'total': total} for criterion in SUCCESS_RULES},
            'native_backend_successes': sum(bool(r['native']['backend_success']) for r in group),
            'available_result_poses': sum(r['native']['pose'] is not None for r in group),
            'failure_codes': codes,
            'backend_failures_by_reason': dict(sorted(
                (reason, sum(r['native']['backend_reason'] == reason for r in group))
                for reason in set(r['native']['backend_reason'] for r in group)
                if reason is not None)),
            'backend_success_outside_loose': sum(bool(r['native']['backend_success'] and not r['loose_success'])
                                                 for r in group),
            'reference_label': group[0]['reference_label'],
            'translation_3d_error_m': distribution(group, 'translation_3d_error_m'),
            'xy_error_m': distribution(group, 'xy_error_m'),
            'z_error_m': distribution(group, 'z_error_m'),
            'yaw_error_deg': distribution(group, 'yaw_error_deg'),
            'so3_error_deg': distribution(group, 'so3_error_deg'),
            'wall_ms_external': distribution([r['native'] for r in group], 'wall_ms_external'),
            'fitness_native': distribution([r['native'] for r in group], 'fitness_native'),
            'density_and_coverage': candidates[tier][name]['coverage'],
            'weak_direction_analysis': _weak_direction_summary(group),
        }
        tables.append(metrics)
    assoc = {name: association(rows, name) for name in
             ('qt_mapping_view_median', 'qr_mapping_view_median',
              'qt_query_local', 'qr_query_conditioned')}
    counts = {code: sum(r.get('failure_code') == code for r in rows) for code in FAILURE_CODES}
    return {'rows': len(rows), 'tables': tables, 'associations': assoc,
            'failure_breakdown': counts,
            'insufficient_map_or_query_points': (counts['INSUFFICIENT_MAP_POINTS']
                                                + counts['INSUFFICIENT_QUERY_POINTS']),
            'density_control_comparisons': _density_control_comparison(tables),
            'parameters_frozen_pre_run': manifest['parameters'],
            'not_run': manifest['not_run'],
            'caveat': 'Q is directional richness, not probability or a safety limit. '
                      'Both real tiers use the same session PGO and full-session evidence labels. '
                      'Returned poses from native failures remain in error distributions with '
                      'backend_success=false; unavailable poses are excluded, never imputed.'}


def write_outputs(run: Path, rows: list[dict], candidates: dict, manifest: dict) -> dict:
    summary = summarize(rows, candidates, manifest)
    (run / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    fields = ['tier', 'algorithm', 'candidate', 'center', 'frames', 'scenario',
              'reference_label', 'leakage_label', 'failure_code', 'backend_success',
              'converged', 'backend_reason', 'strict_success', 'nominal_success',
              'loose_success', 'translation_3d_error_m', 'xy_error_m', 'z_error_m',
              'yaw_error_deg', 'so3_error_deg', 'wall_ms_external', 'fitness_native',
              'overlap_native', 'inliers_native', 'iterations_native',
              'qt_mapping_view_median', 'qr_mapping_view_median', 'qt_query_local',
              'qr_query_conditioned', 'translation_query_weak_alignment_abs_cos',
              'translation_mapping_weak_alignment_abs_cos',
              'rotation_query_weak_alignment_abs_cos',
              'rotation_mapping_weak_alignment_abs_cos',
              'translation_query_weak_recovery_m', 'translation_mapping_weak_recovery_m',
              'rotation_query_weak_recovery_deg', 'rotation_mapping_weak_recovery_deg',
              'reference_pose', 'initial_pose', 'result_pose', 'native_json',
              'geometry_json', 'weak_alignment_json']
    with (run / 'results.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        for row in rows:
            native = row['native']
            geom = row['geometry']
            writer.writerow({
                'tier': row['tier'], 'algorithm': row['algorithm'],
                'candidate': row['candidate'], 'center': row['center'],
                'frames': row['frames'], 'scenario': row['scenario'],
                'reference_label': row['reference_label'],
                'backend_success': native['backend_success'], 'converged': native['converged'],
                'backend_reason': native['backend_reason'],
                **{k: row.get(k) for k in fields if k in row and k not in ('native', 'geometry')},
                'wall_ms_external': native['wall_ms_external'],
                'fitness_native': native['fitness_native'], 'overlap_native': native['overlap_native'],
                'inliers_native': native['inliers_native'], 'iterations_native': native['iterations_native'],
                'qt_mapping_view_median': geom.get('qt_mapping_view_median'),
                'qr_mapping_view_median': geom.get('qr_mapping_view_median'),
                'qt_query_local': geom['qt_query_local']['q'],
                'qr_query_conditioned': geom['qr_query_conditioned']['q'],
                **{key: row['weak_alignment'].get(key) for key in fields
                   if '_weak_alignment_abs_cos' in key or '_weak_recovery_' in key},
                'reference_pose': json.dumps(row['reference_pose']),
                'initial_pose': json.dumps(row['initial_pose']),
                'result_pose': json.dumps(native['pose']),
                'native_json': json.dumps(native, allow_nan=False),
                'geometry_json': json.dumps(geom, allow_nan=False),
                'weak_alignment_json': json.dumps(row['weak_alignment'], allow_nan=False),
            })
    _write_report(run, summary)
    _write_plots(run, summary)
    return summary


def _write_report(run: Path, summary: dict) -> None:
    def fmt(x):
        return 'NA' if x is None else f'{x:.3f}'

    def stats(item):
        return '/'.join(fmt(item[k]) for k in ('median', 'p95', 'worst'))

    text = [
        '# Offline Localization A/B — descriptive results', '',
        '## Observed (offline only)', '',
        'These native trials did not start a ROS node, Nav2, robot controls, or map->odom. '
        'Reference poses are `optimized_PGO_pose`, **not absolute ground truth**.',
        'Tier0 is `SELF_QUERY/DATA_LEAKAGE_EXPECTED`. Tier1 excludes query patches '
        'but has pose-graph, full-session evidence-label and reference-centered oracle-ROI leakage.',
        '', f"Native real registrations: **{summary['rows']}**. Success is reported as N/total, "
        'not as an inferred probability. Strict = 0.2 m / 2°, nominal = 0.5 m / 5°, '
        'loose = 1 m / 10° (3D translation plus absolute heading yaw).', '',
        '| Tier | Algorithm | Map | Frames | Strict | Nominal | Loose | 3D error m median/P95/max | Yaw ° median/P95/max | SO(3) ° median/P95/max | Wall ms median/P95/max |',
        '|---|---|---|---:|---:|---:|---:|---|---|---|---|',
    ]
    for row in summary['tables']:
        def score(name):
            value = row['success_counts'][name]
            return f"{value['success']}/{value['total']}"
        text.append(f"| {row['tier']} | {row['algorithm']} | {row['candidate']} | {row['frames']} | "
                    f"{score('strict')} | {score('nominal')} | {score('loose')} | "
                    f"{stats(row['translation_3d_error_m'])} | {stats(row['yaw_error_deg'])} | "
                    f"{stats(row['so3_error_deg'])} | {stats(row['wall_ms_external'])} |")
    text += ['', '## Failure breakdown', '',
             'A native backend success outside loose is `WRONG_BASIN` (LOCAL) or '
             '`FALSE_RELOCALIZATION` (GLOBAL, relative to same-session PGO). '
             'It is **not** a measured independent negative-session false-positive rate. '
             'MAX_ITERATIONS is only counted if the native CLI states it explicitly; '
             'fitness or overlap never replace pose error.', '',
             '| Failure code | Cases |', '|---|---:|']
    text.extend(f'| {key} | {value} |' for key, value in summary['failure_breakdown'].items())
    text += ['', '## Qt sweep versus point-count controls', '',
             'Each paired control has exactly as many input points as D. XY and XYZ occupied '
             '1m cells expose remaining spatial-coverage confounding; no automatic winner is selected.',
             '', '| Tier | Frames | Qt map | Control | Points equal | XY cells Qt/control | XYZ cells Qt/control | Nominal Qt/control | Median error m Qt/control |',
             '|---|---:|---|---|---|---|---|---|---|']
    for item in summary['density_control_comparisons']:
        a, b = item['qt_nominal_success'], item['control_nominal_success']
        text.append(f"| {item['tier']} | {item['frames']} | {item['qt_candidate']} | {item['control']} | "
                    f"{item['points_match']} ({item['points_each']}) | "
                    f"{item['qt_xy_occupied']}/{item['control_xy_occupied']} | "
                    f"{item['qt_xyz_occupied']}/{item['control_xyz_occupied']} | "
                    f"{a['success']}/{a['total']} vs {b['success']}/{b['total']} | "
                    f"{fmt(item['qt_median_error_m'])}/{fmt(item['control_median_error_m'])} |")
    text += ['', '## Exploratory Q association (not calibrated)', '',
             'Spearman and empirical bins group repeated perturbations by tier/map/query center/frame. '
             'Those groups still share a session and nested maps; **they are not independent samples**. '
             'Qr_mapping_view is a median of mapping-era sidecar values; '
             'Qr_query_conditioned is recomputed around the query body origin from candidate-map '
             'points and frozen normals; they are distinct populations.', '',
             '| Evidence | Candidate/query groups | Spearman vs observed nominal fraction | Bins |',
             '|---|---:|---:|---|']
    for name, item in summary['associations'].items():
        bins = '; '.join(f"[{fmt(b['q_lower'])},{fmt(b['q_upper'])}]:"
                         f"{b['nominal_hits']}/{b['trials']}"
                         for b in item['reliability_bins']) or 'NOT_AVAILABLE'
        text.append(f"| {name} | {item['groups']} | "
                    f"{fmt(item['spearman_q_vs_empirical_nominal_fraction'])} | {bins} |")
    text += ['', '## Weak-axis perturbation / recovery', '',
             'Both mapping-era per-voxel **axial consensus** (invalid when diffuse) and '
             'candidate/query-origin spectrum are reported separately. '
             'Positive projected recovery is only a smaller signed-axis-independent error; '
             'zero starts and missing poses/axes yield null. Below are Tier1 LOCAL '
             '1-frame nonzero starts; each cell is nominal hits/N and median weak-axis recovery.',
             '', '| Map | Axis source / perturbation | Weak-aligned ≥0.80 | Strong-aligned ≤0.33 |',
             '|---|---|---|---|']
    for row in summary['tables']:
        if row['tier'] != 'tier1' or row['algorithm'] != 'LOCAL' or row['frames'] != 1:
            continue
        for source, result in row['weak_direction_analysis'].items():
            def bintext(label):
                bin = result['bins'][label]
                return (f"{bin['nominal_success']}/{bin['total']}; "
                        f"{fmt(bin['weak_projection_recovery']['median'])} "
                        f"{'°' if source.startswith('rotation_') else 'm'}")
            text.append(f"| {row['candidate']} | {source} | {bintext('weak_aligned')} | "
                        f"{bintext('strong_aligned')} |")
    text += ['', '## Analysis limits and hypotheses', '',
             '- Geometry Q measures directional richness, **not P(success)**; observed '
             'associations cannot establish causation or predict an unseen session.',
             '- A density-controlled difference may reflect spatial footprint, oracle ROI, '
             'single-session evidence leakage or native local-map crop, not only Qt.',
             '- Error distributions include *available* poses even when the native backend '
             'reported failure; success N/total still requires backend success.',
             '- Iterations and exact inliers remain null because the unchanged native JSON '
             'does not provide them; do not infer them from overlap.',
             '', '## NOT_RUN / unavailable', '']
    for key, reason in summary['not_run'].items():
        text.append(f'- {key}: {reason}')
    (run / 'report.md').write_text('\n'.join(text) + '\n', encoding='utf-8')


def _write_plots(run: Path, summary: dict) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        (run / 'plots_not_run.txt').write_text('matplotlib unavailable\n')
        return
    plots = run / 'plots'
    plots.mkdir(exist_ok=True)
    tiers = [r for r in summary['tables'] if r['tier'] == 'tier1' and r['algorithm'] == 'LOCAL'
             and r['frames'] == 1]
    if tiers:
        names = [r['candidate'] for r in tiers]
        rates = [r['success_counts']['nominal']['success'] / r['success_counts']['nominal']['total']
                 for r in tiers]
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.bar(range(len(names)), rates, color='#4281a4')
        ax.set_ylim(0, 1)
        ax.set_ylabel('Observed nominal fraction (NOT predicted probability)')
        ax.set_xticks(range(len(names)), names, rotation=65, ha='right')
        ax.set_title('Tier1 LOCAL — frame=1, split-labelled observational result')
        fig.tight_layout()
        fig.savefig(plots / 'tier1_local_nominal.png', dpi=130)
        plt.close(fig)
    bins = summary['associations']['qt_mapping_view_median']['reliability_bins']
    if bins:
        fig, ax = plt.subplots(figsize=(6, 4))
        for b in bins:
            if b['empirical_success_fraction'] is not None:
                center = (b['q_lower'] + b['q_upper']) / 2
                ax.scatter(center, b['empirical_success_fraction'],
                           s=35 + 8 * b['candidate_query_groups_not_independent'], color='#c8604b')
                ax.annotate(str(b['candidate_query_groups_not_independent']),
                            (center, b['empirical_success_fraction']))
        ax.set(xlabel='Median mapping-view Qt bin', ylabel='Observed nominal fraction', ylim=(0, 1),
               title='Empirical groups only; NOT calibrated probability')
        fig.tight_layout()
        fig.savefig(plots / 'qt_empirical_bins.png', dpi=130)
        plt.close(fig)
