"""Phase 3C descriptive analysis for coverage-preserving geometry experiments.

Everything here is offline, same-session and explicitly uncalibrated.
"""
from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.stats import spearmanr


FEATURE_NAMES = (
    'Qt_query', 'Qr_query', 'log_translation_condition', 'log_rotation_condition',
    'valid_normal_voxels', 'crop_map_points', 'crop_occupied_cells', 'crop_support_ratio',
)


def write_coverage_field(path: Path, rows: list[dict]) -> None:
    keys = sorted({key for row in rows for key in row})
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _groups(rows: list[dict]) -> list[dict]:
    grouped: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        if row.get('algorithm') != 'LOCAL':
            continue
        grouped[(row['tier'], row['candidate'], row['center'], row['frames'])].append(row)
    result = []
    for (tier, candidate, center, frames), group in grouped.items():
        first = group[0]
        q = first.get('quality_features') or {}
        support = first.get('local_support') or {}
        result.append({
            'tier': tier, 'candidate': candidate, 'center': center, 'frames': frames,
            'trials': len(group),
            'nominal_hits': sum(bool(row.get('nominal_success')) for row in group),
            'nominal_fraction': sum(bool(row.get('nominal_success')) for row in group) / len(group),
            'translation_error_median': _median(row.get('translation_3d_error_m') for row in group),
            'yaw_error_median': _median(row.get('yaw_error_deg') for row in group),
            'quality_features': q, 'local_support': support,
        })
    return result


def _median(values) -> float | None:
    valid = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return float(np.median(valid)) if valid else None


def _spearman(groups: list[dict], feature: str, target: str) -> dict:
    pairs = []
    for group in groups:
        x = group['quality_features'].get(feature)
        y = group.get(target)
        if x is not None and y is not None and math.isfinite(float(x)) and math.isfinite(float(y)):
            pairs.append((float(x), float(y)))
    if len(pairs) < 4 or len({x for x, _ in pairs}) < 2 or len({y for _, y in pairs}) < 2:
        return {'groups': len(pairs), 'rho': None}
    rho = spearmanr([x for x, _ in pairs], [y for _, y in pairs]).correlation
    return {'groups': len(pairs), 'rho': float(rho) if math.isfinite(float(rho)) else None}


def stratified_qr(groups: list[dict]) -> list[dict]:
    valid = [g for g in groups
             if g['quality_features'].get('crop_support_ratio') is not None
             and g['quality_features'].get('Qr_query') is not None]
    if len(valid) < 6:
        return []
    support = np.asarray([g['quality_features']['crop_support_ratio'] for g in valid], dtype='f8')
    edges = np.unique(np.quantile(support, [0., 1/3, 2/3, 1.]))
    result = []
    for i in range(len(edges) - 1):
        if i == len(edges) - 2:
            subset = [g for g in valid
                      if edges[i] <= g['quality_features']['crop_support_ratio'] <= edges[i + 1]]
        else:
            subset = [g for g in valid
                      if edges[i] <= g['quality_features']['crop_support_ratio'] < edges[i + 1]]
        assoc = _spearman(subset, 'Qr_query', 'nominal_fraction')
        result.append({'support_lower': float(edges[i]), 'support_upper': float(edges[i + 1]),
                       'groups': len(subset), 'qr_vs_nominal_rho': assoc['rho']})
    return result


def _feature_matrix(rows: list[dict], center: int) -> tuple[np.ndarray, np.ndarray]:
    values, labels = [], []
    for row in rows:
        if row.get('tier') != 'tier1' or row.get('algorithm') != 'LOCAL' or row.get('center') != center:
            continue
        features = row.get('quality_features') or {}
        vector = [features.get(name) for name in FEATURE_NAMES]
        if any(v is None or not math.isfinite(float(v)) for v in vector):
            continue
        values.append([float(v) for v in vector])
        labels.append(1.0 if row.get('nominal_success') else 0.0)
    return np.asarray(values, dtype='f8'), np.asarray(labels, dtype='f8')


def _logistic_fit(train_x: np.ndarray, train_y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    if len(train_x) < 20 or len(np.unique(train_y)) < 2:
        return None
    mean = train_x.mean(axis=0)
    scale = train_x.std(axis=0)
    scale[scale < 1e-9] = 1.0
    x = (train_x - mean) / scale
    design = np.column_stack((np.ones(len(x)), x))

    def loss(beta):
        z = np.clip(design @ beta, -30., 30.)
        # Ridge only on feature coefficients; keeps this diagnostic fit stable.
        return float(np.sum(np.logaddexp(0., z) - train_y * z) + 0.5 * 0.1 * np.dot(beta[1:], beta[1:]))

    fit = minimize(loss, np.zeros(design.shape[1]), method='L-BFGS-B')
    if not fit.success or not np.isfinite(fit.x).all():
        return None
    return fit.x, mean, scale


def leave_one_center_out(rows: list[dict]) -> dict:
    centers = sorted({int(row['center']) for row in rows
                      if row.get('tier') == 'tier1' and row.get('algorithm') == 'LOCAL'})
    output = {'status': 'EXPLORATORY', 'groups': centers,
              'note': 'grouped by query center; diagnostic score is NOT calibrated probability',
              'folds': []}
    if len(centers) != 2:
        output['note'] += '; expected exactly two current Tier1 centers, model NOT_RUN'
        return output
    for heldout in centers:
        train_center = next(c for c in centers if c != heldout)
        train_x, train_y = _feature_matrix(rows, train_center)
        test_x, test_y = _feature_matrix(rows, heldout)
        fitted = _logistic_fit(train_x, train_y)
        if fitted is None or not len(test_x):
            output['folds'].append({'train_center': train_center, 'test_center': heldout,
                                    'status': 'NOT_RUN_INSUFFICIENT_VALID_FEATURES'})
            continue
        beta, mean, scale = fitted
        design = np.column_stack((np.ones(len(test_x)), (test_x - mean) / scale))
        score = 1.0 / (1.0 + np.exp(-np.clip(design @ beta, -30., 30.)))
        corr = spearmanr(score, test_y).correlation if len(np.unique(test_y)) > 1 else np.nan
        output['folds'].append({
            'train_center': train_center, 'test_center': heldout, 'status': 'EXPLORATORY',
            'train_rows': int(len(train_x)), 'test_rows': int(len(test_x)),
            'diagnostic_score_success_median': _median(score[test_y == 1]),
            'diagnostic_score_failure_median': _median(score[test_y == 0]),
            'score_vs_nominal_label_spearman': float(corr) if math.isfinite(float(corr)) else None,
            'feature_names': list(FEATURE_NAMES),
        })
    return output


def analyse_phase3c(rows: list[dict]) -> dict:
    groups = _groups(rows)
    correlations = {}
    for feature in FEATURE_NAMES:
        correlations[feature] = {
            'vs_nominal_fraction': _spearman(groups, feature, 'nominal_fraction'),
            'vs_translation_error_median': _spearman(groups, feature, 'translation_error_median'),
            'vs_yaw_error_median': _spearman(groups, feature, 'yaw_error_median'),
        }
    failure = defaultdict(int)
    by_candidate = defaultdict(lambda: {'trials': 0, 'nominal': 0})
    for row in rows:
        if row.get('algorithm') != 'LOCAL':
            continue
        by_candidate[row['candidate']]['trials'] += 1
        by_candidate[row['candidate']]['nominal'] += int(bool(row.get('nominal_success')))
        if row.get('failure_code'):
            failure[(row['candidate'], row['failure_code'])] += 1
    return {
        'semantics': 'offline same-session descriptive analysis; NOT confidence and NOT calibrated',
        'group_count': len(groups),
        'correlations': correlations,
        'crop_support_stratified_qr': stratified_qr(groups),
        'leave_one_center_out_diagnostic': leave_one_center_out(rows),
        'candidate_nominal': dict(by_candidate),
        'failure_by_candidate': [
            {'candidate': candidate, 'failure_code': code, 'count': count}
            for (candidate, code), count in sorted(failure.items())
        ],
    }


def write_phase3c_outputs(run: Path, rows: list[dict], coverage_rows: list[dict]) -> dict:
    summary = analyse_phase3c(rows)
    (run / 'coverage_field.csv').parent.mkdir(parents=True, exist_ok=True)
    write_coverage_field(run / 'coverage_field.csv', coverage_rows)
    (run / 'coverage_geometry_summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')
    lines = [
        '# Phase 3C coverage-preserving geometry analysis', '',
        '**Offline / same-session / not calibrated.** No confidence_v2, runtime GICP change, '
        'Nav2 integration or production map publication is performed.', '',
        '## Candidate nominal observations', '',
        '| Candidate | Nominal | Trials |', '|---|---:|---:|',
    ]
    for name, item in sorted(summary['candidate_nominal'].items()):
        lines.append(f"| {name} | {item['nominal']} | {item['trials']} |")
    lines += ['', '## Exploratory feature association', '',
              '| Feature | rho vs nominal fraction | rho vs median translation error | rho vs median yaw error |',
              '|---|---:|---:|---:|']
    for feature, item in summary['correlations'].items():
        def value(key):
            v = item[key]['rho']
            return 'NA' if v is None else f'{v:.3f}'
        lines.append(f"| {feature} | {value('vs_nominal_fraction')} | "
                     f"{value('vs_translation_error_median')} | {value('vs_yaw_error_median')} |")
    lines += ['', '## Limits', '',
              '- Coverage matching controls 1 m XY cell presence and exact per-cell point quota, '
              'but does not make local geometry identical.',
              '- Query-conditioned evidence is analysis-only and never selects a candidate map.',
              '- Leave-one-center-out uses only the two current Tier1 centers and is EXPLORATORY.',
              '- Tier2 independent-session validation remains NOT_RUN.',
              '']
    (run / 'coverage_geometry_report.md').write_text('\n'.join(lines), encoding='utf-8')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plots = run / 'plots'
        plots.mkdir(exist_ok=True)
        names = sorted(summary['candidate_nominal'])
        if names:
            fractions = [summary['candidate_nominal'][n]['nominal'] /
                         summary['candidate_nominal'][n]['trials'] for n in names]
            fig, ax = plt.subplots(figsize=(12, 4.5))
            ax.bar(range(len(names)), fractions)
            ax.set_ylim(0, 1)
            ax.set_ylabel('Observed nominal fraction (NOT predicted probability)')
            ax.set_xticks(range(len(names)), names, rotation=65, ha='right')
            ax.set_title('Phase 3C LOCAL — coverage controls')
            fig.tight_layout()
            fig.savefig(plots / 'phase3c_success_vs_candidate.png', dpi=130)
            plt.close(fig)
    except ImportError:
        (run / 'phase3c_plots_not_run.txt').write_text('matplotlib unavailable\n', encoding='utf-8')
    return summary
