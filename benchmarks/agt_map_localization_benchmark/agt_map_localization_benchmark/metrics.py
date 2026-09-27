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

# Analytical replay criteria, NOT calibrated field safety or product gates.
SUCCESS_RULES = {
    'strict': {'translation_3d_m': 0.25, 'z_m': 0.20, 'yaw_deg': 5.0},
    'nominal': {'translation_3d_m': 0.50, 'z_m': 0.40, 'yaw_deg': 10.0},
    'loose': {'translation_3d_m': 1.00, 'z_m': 0.70, 'yaw_deg': 20.0},
}


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
    yaw_delta = float(d_rotation.as_euler('zyx', degrees=True)[0])
    out.update({'translation_3d_error_m': float(np.linalg.norm(delta)),
                'xy_error_m': float(np.linalg.norm(delta[:2])),
                'z_error_m': abs(float(delta[2])),
                'yaw_error_deg': abs(((yaw_delta + 180) % 360) - 180),
                'so3_error_deg': math.degrees(d_rotation.magnitude()),
                'error_xyz_map_m': [float(v) for v in delta]})
    for name, rule in SUCCESS_RULES.items():
        out[f'{name}_success'] = bool(native['backend_success']
            and out['translation_3d_error_m'] <= rule['translation_3d_m']
            and out['z_error_m'] <= rule['z_m']
            and out['yaw_error_deg'] <= rule['yaw_deg'])
    return out


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
                     'independent_candidate_query_groups': len(group),
                     'nominal_hits': hits, 'trials': trials,
                     'empirical_success_fraction': hits / trials if trials else None})
    return {'groups': len(pairs),
            'spearman_q_vs_empirical_nominal_fraction': float(corr) if math.isfinite(corr) else None,
            'reliability_bins': bins,
            'note': 'descriptive repeated-session association, NOT calibrated P(success)'}


def summarize(rows: list[dict], candidates: dict, manifest: dict) -> dict:
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['tier'], row['algorithm'], row['candidate'], row['frames'])].append(row)
    tables = []
    for (tier, algo, name, frames), group in sorted(grouped.items()):
        total = len(group)
        metrics = {
            'tier': tier, 'algorithm': algo, 'candidate': name, 'frames': frames,
            'success_counts': {criterion: {
                'success': sum(bool(r[f'{criterion}_success']) for r in group),
                'total': total} for criterion in SUCCESS_RULES},
            'native_backend_successes': sum(bool(r['native']['backend_success']) for r in group),
            'backend_failures_by_reason': dict(sorted(
                (reason, sum(r['native']['backend_reason'] == reason for r in group))
                for reason in set(r['native']['backend_reason'] for r in group)
                if reason is not None)),
            'backend_success_outside_loose': sum(bool(r['native']['backend_success'] and not r['loose_success'])
                                                 for r in group),
            'reference_label': group[0]['reference_label'],
            'translation_3d_error_m': distribution(group, 'translation_3d_error_m'),
            'xy_error_m': distribution(group, 'xy_error_m'),
            'yaw_error_deg': distribution(group, 'yaw_error_deg'),
            'wall_ms_external': distribution([r['native'] for r in group], 'wall_ms_external'),
            'fitness_native': distribution([r['native'] for r in group], 'fitness_native'),
            'weak_axis_recovery_m': distribution([r['weak_alignment'] for r in group],
                                                  'weak_axis_recovery_m'),
            'density_and_coverage': candidates[tier][name]['coverage'],
        }
        for label, pred in (('strong_aligned', lambda x: x is not None and x >= 0.80),
                            ('weak_aligned', lambda x: x is not None and x <= 0.33)):
            subset = [r for r in group if pred(r['weak_alignment']['translation_perturb_weak_axis_abs_cos'])]
            metrics[label + '_perturbations'] = {
                'nominal_success': sum(r['nominal_success'] for r in subset),
                'total': len(subset),
                'weak_axis_recovery_m': distribution([r['weak_alignment'] for r in subset],
                                                      'weak_axis_recovery_m')}
        tables.append(metrics)
    assoc = {name: association(rows, name) for name in
             ('qt_mapping_view_median', 'qr_mapping_view_median',
              'qt_query_local', 'qr_query_conditioned')}
    return {'rows': len(rows), 'tables': tables, 'associations': assoc,
            'parameters_frozen_pre_run': manifest['parameters'],
            'not_run': manifest['not_run'],
            'caveat': 'Q is directional richness, not probability or a safety limit. '
                      'Both real tiers use the same session PGO and full-session evidence labels.'}


def write_outputs(run: Path, rows: list[dict], candidates: dict, manifest: dict) -> dict:
    summary = summarize(rows, candidates, manifest)
    (run / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    fields = ['tier', 'algorithm', 'candidate', 'center', 'frames', 'scenario',
              'reference_label', 'backend_success', 'converged', 'backend_reason',
              'strict_success', 'nominal_success', 'loose_success',
              'translation_3d_error_m', 'xy_error_m', 'z_error_m', 'yaw_error_deg',
              'so3_error_deg', 'wall_ms_external', 'fitness_native', 'overlap_native',
              'inliers_native', 'iterations_native', 'qt_mapping_view_median',
              'qr_mapping_view_median', 'qt_query_local', 'qr_query_conditioned',
              'weak_axis_recovery_m', 'perturb_alignment_abs_cos',
              'reference_pose', 'initial_pose', 'result_pose', 'native_json',
              'geometry_json']
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
                'weak_axis_recovery_m': row['weak_alignment']['weak_axis_recovery_m'],
                'perturb_alignment_abs_cos': row['weak_alignment']['translation_perturb_weak_axis_abs_cos'],
                'reference_pose': json.dumps(row['reference_pose']),
                'initial_pose': json.dumps(row['initial_pose']),
                'result_pose': json.dumps(native['pose']),
                'native_json': json.dumps(native, allow_nan=False),
                'geometry_json': json.dumps(geom, allow_nan=False),
            })
    _write_report(run, summary)
    _write_plots(run, summary)
    return summary


def _write_report(run: Path, summary: dict) -> None:
    text = [
        '# Offline Localization A/B — descriptive results', '',
        'These are **observed offline runs**, not an online field validation or a '
        'product-map threshold decision. PGO references are `optimized_PGO_pose`, not absolute GT.',
        '', f"Total native registrations: {summary['rows']}", '',
        '| Tier | Algorithm | Candidate | Frames | strict | nominal | loose | median 3D error | P95 3D error | median wall ms |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for row in summary['tables']:
        def score(name):
            v = row['success_counts'][name]
            return f"{v['success']}/{v['total']}"
        d = row['translation_3d_error_m']
        w = row['wall_ms_external']['median']
        text.append(f"| {row['tier']} | {row['algorithm']} | {row['candidate']} | {row['frames']} | "
                    f"{score('strict')} | {score('nominal')} | {score('loose')} | "
                    f"{d['median'] if d['median'] is not None else 'NA'} | "
                    f"{d['p95'] if d['p95'] is not None else 'NA'} | "
                    f"{w if w is not None else 'NA'} |")
    text += ['', '## Analysis limits', '',
             '- Tier0 contains its own query points (`SELF_QUERY/DATA_LEAKAGE_EXPECTED`).',
             '- Tier1 excludes query patches from every candidate map and descriptor database; '
             'optimized poses and V1/geometry labels still use the full single session '
             '(pose-graph **and evidence-label** leakage). The bounded map ROI is centered '
             'on the known PGO reference: offline oracle ROI bias, not no-seed full-map search.',
             '- Qr_mapping_view is the median of mapping-era per-voxel sidecar Qr. '
             'Qr_query_conditioned is a separate query-origin recomputation on map-subset points; '
             'they are not interchangeable.',
             '- Native metrics not exposed by the unchanged executable (iterations, exact inlier count) '
             'are null, not inferred from overlap.',
             '- Strict/nominal/loose are predeclared offline replay criteria; Q is not P(success).',
             '- GLOBAL success outside loose is backend-success-with-wrong-reference-pose, '
             'not a measured negative-session false relocation rate.',
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
                           s=35 + 8 * b['independent_candidate_query_groups'], color='#c8604b')
                ax.annotate(str(b['independent_candidate_query_groups']),
                            (center, b['empirical_success_fraction']))
        ax.set(xlabel='Median mapping-view Qt bin', ylabel='Observed nominal fraction', ylim=(0, 1),
               title='Empirical groups only; NOT calibrated probability')
        fig.tight_layout()
        fig.savefig(plots / 'qt_empirical_bins.png', dpi=130)
        plt.close(fig)
