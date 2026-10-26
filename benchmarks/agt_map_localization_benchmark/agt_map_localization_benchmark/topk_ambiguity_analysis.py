"""Offline, read-only analysis of the authoritative native candidate trace.

No retrieval, BBS, or registration is implemented here. Missing physical labels and
unattempted stage outcomes are unavailable (CSV empty cells), never false results.
Confidence intervals resample scene/query units, never candidate entries.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

K_VALUES = (1, 3, 5, 10)
DISTANCE_THRESHOLDS_M = (1.0, 2.0, 5.0)
SCENE_TYPES = ('ROW_ENTRY', 'ROW_MIDDLE', 'ROW_END', 'HEADLAND', 'OTHER', 'UNKNOWN')
UNKNOWN = 'UNKNOWN'


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analysis_source_provenance() -> dict:
    source = Path(__file__).resolve()
    result = {'analysis_source_path': str(source), 'analysis_source_sha256': sha256(source),
              'mapping_git_commit': UNKNOWN, 'mapping_tracked_source_diff_sha256': None,
              'mapping_source_tree_sha256': None}
    try:
        repo = source.parents[3]
        result['mapping_git_commit'] = subprocess.check_output(
            ['git', '-C', str(repo), 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL, text=True).strip()
        owned_paths = ['benchmarks/agt_map_localization_benchmark', 'tools/agt_greenhouse_annotation']
        diff = subprocess.check_output(['git', '-C', str(repo), 'diff', '--binary', 'HEAD', '--']+owned_paths,
                                       stderr=subprocess.DEVNULL)
        result['mapping_tracked_source_diff_sha256'] = hashlib.sha256(diff).hexdigest()
        paths = subprocess.check_output(['git', '-C', str(repo), 'ls-files', '-z', '--cached', '--others',
                                         '--exclude-standard', '--']+owned_paths, stderr=subprocess.DEVNULL)
        digest = hashlib.sha256()
        for encoded in sorted(set(paths.split(b'\0'))-{b''}):
            file = repo / encoded.decode('utf-8')
            if file.is_file():
                digest.update(encoded+b'\0'+hashlib.sha256(file.read_bytes()).digest())
        result['mapping_source_tree_sha256'] = digest.hexdigest()
        result['mapping_source_tree_scope'] = owned_paths
        result['mapping_source_tree_semantics'] = 'SHA256 of sorted relative path, NUL, and file-content SHA256; includes tracked and untracked source files'
    except (subprocess.SubprocessError, OSError, UnicodeError, IndexError):
        # Installed copies may have no working tree; the actual module bytes
        # remain identified even in that deployment.
        pass
    return result


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_trace(trace: dict) -> None:
    """Reject ambiguous trace contracts, while allowing unavailable stage data."""
    def check_finite(value, location='trace'):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f'{location}: unavailable numeric data must use JSON null, not NaN/Inf')
        if isinstance(value, dict):
            for key, item in value.items(): check_finite(item, f'{location}.{key}')
        elif isinstance(value, list):
            for index, item in enumerate(value): check_finite(item, f'{location}[{index}]')
    check_finite(trace)
    if trace.get('schema_version') != 1:
        raise ValueError('trace schema_version must be 1')
    for field in ('query', 'descriptor', 'ranked_candidates', 'selected'):
        if field not in trace:
            raise ValueError(f'trace missing {field}')
    descriptor = trace['descriptor']
    for field in ('database_size', 'prefilter', 'candidate_top_k'):
        if not isinstance(descriptor.get(field), int) or descriptor[field] < 0:
            raise ValueError(f'descriptor.{field} must be a nonnegative integer')
    semantics = descriptor.get('score_semantics', {})
    if semantics.get('sector_similarity', 'higher_is_better') != 'higher_is_better':
        raise ValueError('sector_similarity must declare higher_is_better')
    if semantics.get('ring_distance', 'lower_is_better') != 'lower_is_better':
        raise ValueError('ring_distance must declare lower_is_better')
    candidates = trace['ranked_candidates']
    if not isinstance(candidates, list):
        raise ValueError('ranked_candidates must be a list')
    if len(candidates) > min(descriptor['database_size'], descriptor['prefilter']):
        raise ValueError('ranked candidate count exceeds recorded prefilter/database size')
    for rank, candidate in enumerate(candidates, 1):
        if candidate.get('rank') != rank:
            raise ValueError('candidate ranks must be contiguous and ordered from 1')
        for field in ('patch', 'keyframe', 'descriptor', 'map_pose', 'bbs', 'gicp'):
            if field not in candidate:
                raise ValueError(f'candidate rank {rank} missing {field}')
        for field in ('ring_distance', 'sector_similarity'):
            value = candidate['descriptor'].get(field)
            if value is not None and not finite(value):
                raise ValueError(f'candidate rank {rank} nonfinite {field}')
    selected = trace['selected'].get('candidate_rank')
    if selected is not None and (not isinstance(selected, int) or selected < 1 or selected > len(candidates)):
        raise ValueError('selected candidate_rank outside ranked_candidates')


def _yaw(pose: dict) -> float | None:
    if finite(pose.get('yaw_deg')):
        return math.radians(pose['yaw_deg'])
    if finite(pose.get('yaw_rad')):
        return pose['yaw_rad']
    if all(finite(pose.get(k)) for k in ('qx', 'qy', 'qz', 'qw')):
        x, y, z, w = (pose[k] for k in ('qx', 'qy', 'qz', 'qw'))
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = (v / norm for v in (x, y, z, w))
            return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return None


def pose_error(pose: dict | None, reference: dict | None) -> tuple[float | None, float | None]:
    if not pose or not reference or not all(finite(p.get(k)) for p in (pose, reference) for k in ('x', 'y')):
        return None, None
    xy = math.hypot(pose['x']-reference['x'], pose['y']-reference['y'])
    y1, y2 = _yaw(pose), _yaw(reference)
    yaw = None if y1 is None or y2 is None else abs(math.degrees(math.atan2(math.sin(y1-y2), math.cos(y1-y2))))
    return xy, yaw


def _unknown_label(reason: str) -> dict:
    return {'physical_row_id': UNKNOWN, 'along_row_s_m': None, 'lateral_d_m': None,
            'relative_heading_deg': None, 'zone_type': UNKNOWN, 'headland_id': None,
            'label_confidence': UNKNOWN, 'label_unavailable_reason': reason}


def canonical_pose(topology: dict, backend_id: str, pose: dict) -> dict | None:
    """Transform backend coordinates only using an explicit recorded transform."""
    canonical = str(topology.get('source', {}).get('backend_id', ''))
    if not all(finite(pose.get(k)) for k in ('x', 'y')):
        return None
    if backend_id == canonical and canonical:
        return dict(pose)
    transform = topology.get('backend_transforms', {}).get(backend_id)
    if not transform:
        return None
    matrix = np.asarray(transform.get('matrix'), dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f'invalid backend transform for {backend_id}')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f'invalid homogeneous backend transform for {backend_id}')
    xyz = matrix @ np.array([pose['x'], pose['y'], pose.get('z', 0.0) or 0.0, 1.0])
    result = {'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2])}
    yaw = _yaw(pose)
    if yaw is not None:
        direction = matrix[:3, :3] @ np.array([math.cos(yaw), math.sin(yaw), 0.0])
        result['yaw_deg'] = math.degrees(math.atan2(direction[1], direction[0]))
    return result


def label_backend_pose(topology: dict, backend_id: str, pose: dict,
                       keyframe: Any = None, labels: dict | None = None) -> dict:
    if topology.get('annotation', {}).get('manual_review_confirmed') is not True:
        return _unknown_label('manual_topology_not_confirmed')
    transformed = canonical_pose(topology, backend_id, pose)
    if transformed is None:
        return _unknown_label('missing_explicit_backend_to_canonical_transform_or_pose')
    from agt_greenhouse_annotation.topology import label_pose
    yaw = _yaw(transformed)
    result = label_pose(topology, transformed['x'], transformed['y'], yaw or 0.0)
    if yaw is None:
        result['relative_heading_deg'] = None
    # CSV labels are a checked view of this frozen geometry, never an override
    # that could attach labels from an older annotation to a new trace.
    lookup = (labels or {}).get((backend_id, str(keyframe)))
    if lookup is not None:
        for field in ('physical_row_id', 'zone_type', 'headland_id'):
            if (lookup.get(field) or UNKNOWN) != (result.get(field) or UNKNOWN):
                raise ValueError(f'stale/inconsistent keyframe labels for {backend_id}:{keyframe}: {field}')
        for field in ('along_row_s_m', 'lateral_d_m', 'relative_heading_deg'):
            first, second = lookup.get(field), result.get(field)
            if first is not None and second is not None and abs(first-second)>1e-4:
                raise ValueError(f'stale/inconsistent keyframe labels for {backend_id}:{keyframe}: {field}')
    return result


def load_labels(paths: list[Path]) -> dict:
    result = {}
    for path in paths:
        with path.open(newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                if not row.get('backend_id'):
                    raise ValueError(f'{path}: keyframe labels require backend_id')
                key = (row['backend_id'], str(row['keyframe']))
                if key in result:
                    raise ValueError(f'duplicate keyframe label {key}')
                label = {k: row.get(k) or None for k in (
                    'physical_row_id', 'zone_type', 'headland_id', 'label_confidence')}
                label['physical_row_id'] = label['physical_row_id'] or UNKNOWN
                for field in ('along_row_s_m', 'lateral_d_m', 'relative_heading_deg'):
                    value = row.get(field)
                    label[field] = float(value) if value else None
                    if label[field] is not None and not finite(label[field]):
                        raise ValueError(f'{path}: nonfinite {field}')
                result[key] = label
    return result


def _known_row(label: dict) -> bool:
    return (label.get('physical_row_id') not in (None, '', UNKNOWN)
            and label.get('label_confidence') == 'confirmed')


def observable_recall(values: list[bool | None], complete: bool) -> bool | None:
    """A known hit is sufficient; a miss requires all requested entries observed."""
    if any(value is True for value in values):
        return True
    if complete and values and all(value is False for value in values):
        return False
    return None


def score_margin(first: float | None, second: float | None, higher_is_better: bool = True) -> float | None:
    if not finite(first) or not finite(second):
        return None
    sign = 1.0 if higher_is_better else -1.0
    return sign * (first-second) / (abs(first) + 1e-12)


def row_entropy(candidates: list[dict], mode: str = 'rank_frequency', tau: float | None = None) -> dict:
    if mode not in ('rank_frequency', 'softmax'):
        raise ValueError('unknown entropy mode')
    if mode == 'softmax' and (not finite(tau) or tau <= 0):
        raise ValueError('softmax entropy requires explicitly fixed positive tau')
    weights = np.ones(len(candidates), dtype=float)
    if mode == 'softmax' and candidates:
        scores = [c['descriptor'].get('sector_similarity') for c in candidates]
        if not all(finite(score) for score in scores):
            return {'row_entropy': None, 'effective_row_count': None,
                    'entropy_known_weight_fraction': None}
        logits = np.asarray(scores, dtype=float) / tau
        weights = np.exp(logits - logits.max())
    masses = defaultdict(float)
    for candidate, weight in zip(candidates, weights):
        if _known_row(candidate['label']):
            masses[candidate['label']['physical_row_id']] += float(weight)
    total = sum(masses.values())
    fraction = None if not len(weights) else total / float(weights.sum())
    if not total:
        return {'row_entropy': None, 'effective_row_count': None,
                'entropy_known_weight_fraction': fraction}
    probabilities = np.array(list(masses.values())) / total
    entropy = float(-np.sum(probabilities * np.log(probabilities)))
    return {'row_entropy': entropy, 'effective_row_count': math.exp(entropy),
            'entropy_known_weight_fraction': fraction}


def spatial_spread(candidates: list[dict], query_row: str | None) -> dict:
    poses = [c['map_pose'] for c in candidates if all(finite(c['map_pose'].get(k)) for k in ('x', 'y'))]
    result = {'spatial_xy_observed_count': len(poses), 'centroid_x': None, 'centroid_y': None,
              'mean_radius_m': None, 'p95_radius_m': None, 'max_pairwise_distance_m': None,
              's_spread_m': None, 'within_row_s_spread_max_m': None,
              'known_candidate_row_count': 0}
    if poses:
        xy = np.array([[p['x'], p['y']] for p in poses])
        centroid = xy.mean(axis=0)
        radii = np.linalg.norm(xy-centroid, axis=1)
        pairwise = np.linalg.norm(xy[:, None]-xy[None, :], axis=2)
        result.update(centroid_x=float(centroid[0]), centroid_y=float(centroid[1]),
                      mean_radius_m=float(radii.mean()), p95_radius_m=float(np.percentile(radii, 95)),
                      max_pairwise_distance_m=float(pairwise.max()))
    stations = defaultdict(list)
    for candidate in candidates:
        label = candidate['label']
        if _known_row(label) and finite(label.get('along_row_s_m')):
            stations[label['physical_row_id']].append(label['along_row_s_m'])
    result['known_candidate_row_count'] = len({c['label']['physical_row_id'] for c in candidates if _known_row(c['label'])})
    if stations:
        result['within_row_s_spread_max_m'] = max(max(v)-min(v) for v in stations.values())
    if query_row in stations:
        result['s_spread_m'] = max(stations[query_row])-min(stations[query_row])
    return result


def _stage_outcome(candidate: dict, stage: str, reference: dict) -> bool | None:
    data = candidate[stage]
    if not all(finite(reference.get(k)) for k in ('x', 'y')) or _yaw(reference) is None:
        return None
    if data.get('attempted') is not True:
        return None
    if stage == 'bbs':
        if data.get('valid') is False:
            return None if data.get('timed_out') is True else False
        if data.get('valid') is not True:
            return None
        pose, xy_limit, yaw_limit = data.get('coarse_pose'), 1.0, 10.0
    else:
        if data.get('converged') is False:
            return False
        if data.get('converged') is not True:
            return None
        pose, xy_limit, yaw_limit = data.get('final_pose'), 0.5, 5.0
    xy, yaw = pose_error(pose, reference)
    if xy is None or yaw is None:
        return None
    return xy <= xy_limit and yaw <= yaw_limit


def analyze_trace(trace: dict, topology: dict, labels: dict | None = None,
                  entropy_mode: str = 'rank_frequency', tau: float | None = None) -> tuple[list[dict], list[dict]]:
    validate_trace(trace)
    query = trace['query']
    backend = str(query.get('backend_id', UNKNOWN))
    reference = query.get('reference_pose') or {}
    query_label = label_backend_pose(topology, backend, reference, query.get('keyframe'), labels)
    query_row = query_label.get('physical_row_id') if _known_row(query_label) else None
    scene_id = str(query.get('scene_id') or f"timestamp_{query.get('timestamp', 'UNKNOWN')}")
    alignment = topology.get('backend_transforms', {}).get(backend, {})
    residual = alignment.get('residual_xy_m', {})
    base = {'backend_id': backend, 'scene_id': scene_id,
            'scene_type': str(query.get('scene_type', UNKNOWN)).upper(),
            'frames': int(query.get('frames', 1)), 'timestamp': query.get('timestamp'),
            'query_keyframe': query.get('keyframe'), 'query_row': query_row or UNKNOWN,
            'query_s_m': query_label.get('along_row_s_m'), 'query_zone_type': query_label.get('zone_type'),
            'query_headland_id': query_label.get('headland_id'),
            'query_label_unavailable_reason': query_label.get('label_unavailable_reason'),
            'descriptor_prefilter': trace['descriptor']['prefilter'],
            'candidate_top_k': trace['descriptor']['candidate_top_k'],
            'descriptor_database_size': trace['descriptor']['database_size'],
            'ranking_observed_count': len(trace['ranked_candidates']),
            'candidate_spread_coordinate_frame': f'backend_map:{backend}',
            'backend_alignment_method': alignment.get('method', 'canonical_backend' if backend == str(topology.get('source',{}).get('backend_id')) else UNKNOWN),
            'backend_alignment_p95_m': residual.get('p95'), 'backend_alignment_max_m': residual.get('max'),
            'entropy_mode': entropy_mode, 'softmax_tau': tau}
    enriched, candidate_rows = [], []
    for candidate in trace['ranked_candidates']:
        candidate = dict(candidate)
        candidate['label'] = label_backend_pose(topology, backend, candidate['map_pose'], candidate['keyframe'], labels)
        label = candidate['label']
        same_row = None if not query_row or not _known_row(label) else label['physical_row_id'] == query_row
        delta_s = (abs(label['along_row_s_m']-query_label['along_row_s_m'])
                   if same_row and finite(label.get('along_row_s_m')) and finite(query_label.get('along_row_s_m')) else None)
        xy, yaw = pose_error(candidate['map_pose'], reference)
        candidate.update(same_row=same_row, delta_s_m=delta_s, reference_xy_error_m=xy,
                         bbs_outcome=_stage_outcome(candidate, 'bbs', reference),
                         gicp_outcome=_stage_outcome(candidate, 'gicp', reference))
        enriched.append(candidate)
        candidate_rows.append(dict(base, rank=candidate['rank'], patch=candidate['patch'],
            candidate_keyframe=candidate['keyframe'], candidate_row=label.get('physical_row_id', UNKNOWN),
            candidate_s_m=label.get('along_row_s_m'), candidate_d_m=label.get('lateral_d_m'),
            candidate_psi_rel_deg=label.get('relative_heading_deg'), candidate_zone_type=label.get('zone_type'),
            candidate_headland_id=label.get('headland_id'), same_physical_row=same_row, delta_s_m=delta_s,
            sector_similarity=candidate['descriptor'].get('sector_similarity'),
            ring_distance=candidate['descriptor'].get('ring_distance'),
            candidate_x=candidate['map_pose'].get('x'), candidate_y=candidate['map_pose'].get('y'),
            reference_xy_error_m=xy, reference_yaw_error_deg=yaw,
            bbs_attempted=candidate['bbs'].get('attempted'), bbs_timed_out=candidate['bbs'].get('timed_out'),
            bbs_score=candidate['bbs'].get('score'), bbs_elapsed_ms=candidate['bbs'].get('elapsed_ms'),
            bbs_basin_hit=candidate['bbs_outcome'], gicp_attempted=candidate['gicp'].get('attempted'),
            gicp_fitness=candidate['gicp'].get('fitness'), gicp_overlap=candidate['gicp'].get('overlap'),
            gicp_elapsed_ms=candidate['gicp'].get('elapsed_ms'), gicp_final_success=candidate['gicp_outcome']))
    rows = []
    for k in K_VALUES:
        candidates = enriched[:k]
        complete = len(candidates) == min(k, trace['descriptor']['database_size'])
        same = [c['same_row'] for c in candidates]
        known = [value for value in same if value is not None]
        ds = [c['delta_s_m'] for c in candidates if finite(c['delta_s_m'])]
        row = dict(base, k=k, topk_observed_count=len(candidates), requested_ranking_complete=complete,
                   physical_row_recall=observable_recall(same, complete) if query_row else None,
                   wrong_row_fraction=(sum(value is False for value in known)/len(known))
                       if known and len(known)==len(candidates) and complete else None,
                   wrong_row_fraction_known_rows=(sum(value is False for value in known)/len(known)) if known else None,
                   wrong_row_fraction_lower_bound=(sum(value is False for value in known)/len(candidates))
                       if known and candidates and complete else None,
                   wrong_row_fraction_upper_bound=((sum(value is False for value in known)+len(candidates)-len(known))/len(candidates))
                       if known and candidates and complete else None,
                   wrong_row_known_count=len(known), wrong_row_unknown_count=len(candidates)-len(known),
                   top1_wrong_row=(None if not same or same[0] is None else not same[0]),
                   same_row_candidate_count=sum(value is True for value in same),
                   longitudinal_observed_count=len(ds), longitudinal_median_m=float(np.median(ds)) if ds else None,
                   longitudinal_p95_m=float(np.percentile(ds, 95)) if ds else None)
        for threshold in DISTANCE_THRESHOLDS_M:
            suffix = f'{int(threshold)}m'
            region = [None if c['same_row'] is None else (c['same_row'] and c['delta_s_m'] < threshold)
                      if c['same_row'] is False or c['delta_s_m'] is not None else None for c in candidates]
            row[f'pose_region_recall_{suffix}'] = observable_recall(region, complete) if query_row else None
            row[f'longitudinal_fraction_gt_{suffix}'] = float(np.mean(np.asarray(ds)>threshold)) if ds else None
            xyhits = [None if c['reference_xy_error_m'] is None else c['reference_xy_error_m'] < threshold for c in candidates]
            row[f'xy_region_recall_{suffix}'] = observable_recall(xyhits, complete)
        headland_id = query_label.get('headland_id')
        if headland_id in (None, '', UNKNOWN) or query_label.get('label_confidence') != 'confirmed':
            headland_id = None
        headhits = [None if c['label'].get('zone_type') == UNKNOWN or c['label'].get('label_confidence') != 'confirmed'
                    else c['label'].get('headland_id') == headland_id for c in candidates]
        row['headland_region_recall'] = observable_recall(headhits, complete) if headland_id else None
        row.update(row_entropy(candidates, entropy_mode, tau))
        row.update(spatial_spread(candidates, query_row))
        row['score_margin_m12'] = score_margin(
            enriched[0]['descriptor'].get('sector_similarity') if enriched else None,
            enriched[1]['descriptor'].get('sector_similarity') if len(enriched)>1 else None)
        for stage, prefix in (('bbs', 'bbs_basin_hit'), ('gicp', 'gicp_final_success')):
            outcomes = [c[f'{stage}_outcome'] for c in candidates]
            observed = [value for value in outcomes if value is not None]
            row[prefix] = observable_recall(outcomes, complete)
            row[f'{stage}_observed_count'] = len(observed)
            row[f'{stage}_censored_count'] = len(candidates)-len(observed)
            row[f'{stage}_attempted_count'] = sum(c[stage].get('attempted') is True for c in candidates)
            row[f'{stage}_observed_hit_fraction'] = sum(value is True for value in observed)/len(observed) if observed else None
        acceptance = trace.get('acceptance', {})
        row['offline_reference_nominal_success'] = (acceptance.get('nominal_success')
            if acceptance.get('owner') == 'offline_reference_tolerance' else None)
        row['final_accepted_success'] = (acceptance.get('accepted') is True and acceptance.get('nominal_success') is True
            if acceptance.get('owner') == 'runtime_policy' and isinstance(acceptance.get('accepted'), bool)
            and isinstance(acceptance.get('nominal_success'), bool) else None)
        row['acceptance_owner'] = acceptance.get('owner', UNKNOWN)
        row['selected_candidate_rank'] = trace['selected'].get('candidate_rank')
        selected = next((c for c in enriched if c['rank'] == row['selected_candidate_rank']), None)
        row['selected_bbs_nominal_success'] = selected['bbs_outcome'] if selected else None
        row['selected_gicp_nominal_success'] = selected['gicp_outcome'] if selected else None
        rows.append(row)
    return rows, candidate_rows


def scene_bootstrap(values: list[dict], metric: str, samples: int, rng: np.random.Generator) -> tuple[float | None, float | None]:
    by_scene = defaultdict(list)
    for row in values:
        if finite(row.get(metric)) or isinstance(row.get(metric), bool):
            by_scene[row['scene_id']].append(float(row[metric]))
    means = np.asarray([np.mean(items) for items in by_scene.values()])
    if not len(means):
        return None, None
    if not samples or len(means) == 1:
        # A single scene does not establish population uncertainty.
        return None, None
    draws = rng.choice(means, size=(samples, len(means)), replace=True).mean(axis=1)
    return tuple(float(v) for v in np.percentile(draws, [2.5, 97.5]))


def aggregate(rows: list[dict], metrics: list[str], samples: int, seed: int) -> list[dict]:
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['backend_id'], row['frames'], row['scene_type'], row['k'])].append(row)
    result = []
    rng = np.random.default_rng(seed)
    for key, items in sorted(grouped.items()):
        for metric in metrics:
            numbers = [float(row[metric]) for row in items if finite(row.get(metric)) or isinstance(row.get(metric), bool)]
            lo, hi = scene_bootstrap(items, metric, samples, rng)
            # Equal scene weighting prevents repeated queries in one scene dominating.
            per_scene = defaultdict(list)
            for row in items:
                if finite(row.get(metric)) or isinstance(row.get(metric), bool):
                    per_scene[row['scene_id']].append(float(row[metric]))
            result.append(dict(backend_id=key[0], frames=key[1], scene_type=key[2], k=key[3],
                metric=metric, query_count=len(items), observed_query_count=len(numbers),
                unavailable_query_count=len(items)-len(numbers), observed_scene_count=len(per_scene),
                scene_weighted_mean=float(np.mean([np.mean(v) for v in per_scene.values()])) if per_scene else None,
                query_median=float(np.median(numbers)) if numbers else None,
                query_p95=float(np.percentile(numbers, 95)) if numbers else None,
                ci95_lower=lo, ci95_upper=hi))
    return result


def paired_effect(rows: list[dict], metrics: list[str], variable: str,
                  samples: int, seed: int) -> list[dict]:
    """Scene-paired effects for frame windows or backend, preserving query units."""
    result = []
    rng = np.random.default_rng(seed)
    fixed = ('backend_id', 'scene_type', 'k') if variable == 'frames' else ('frames', 'scene_type', 'k')
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in fixed)].append(row)
    for key, items in sorted(groups.items()):
        levels = sorted({row[variable] for row in items})
        for i, first in enumerate(levels):
            for second in levels[i+1:]:
                for metric in metrics:
                    per_scene = defaultdict(lambda: defaultdict(list))
                    for row in items:
                        if row[variable] in (first, second) and (finite(row.get(metric)) or isinstance(row.get(metric), bool)):
                            per_scene[row['scene_id']][row[variable]].append(float(row[metric]))
                    deltas = [np.mean(values[second])-np.mean(values[first]) for values in per_scene.values()
                              if first in values and second in values]
                    lo = hi = None
                    if samples and len(deltas)>1:
                        draws = rng.choice(deltas, size=(samples, len(deltas)), replace=True).mean(axis=1)
                        lo, hi = (float(v) for v in np.percentile(draws, [2.5, 97.5]))
                    result.append(dict(zip(fixed, key), metric=metric, first=first, second=second,
                        effect_definition='second_minus_first_scene_paired', paired_scene_count=len(deltas),
                        mean_difference=float(np.mean(deltas)) if deltas else None, ci95_lower=lo, ci95_upper=hi))
    return result


def scene_effects(rows: list[dict], metrics: list[str], samples: int, seed: int) -> list[dict]:
    groups = defaultdict(list)
    rng = np.random.default_rng(seed)
    for row in rows:
        groups[(row['backend_id'], row['frames'], row['k'])].append(row)
    result = []
    for key, items in sorted(groups.items()):
        for metric in metrics:
            distributions = {}
            for scene_type in ('ROW_MIDDLE', 'HEADLAND'):
                per_scene = defaultdict(list)
                for row in items:
                    if row['scene_type']==scene_type and (finite(row.get(metric)) or isinstance(row.get(metric), bool)):
                        per_scene[row['scene_id']].append(float(row[metric]))
                distributions[scene_type] = [np.mean(v) for v in per_scene.values()]
            middle, headland = distributions['ROW_MIDDLE'], distributions['HEADLAND']
            delta = float(np.mean(headland)-np.mean(middle)) if middle and headland else None
            lo = hi = None
            if samples and len(middle)>1 and len(headland)>1:
                draws = (rng.choice(headland, (samples,len(headland)),replace=True).mean(axis=1)
                         -rng.choice(middle, (samples,len(middle)),replace=True).mean(axis=1))
                lo, hi = (float(v) for v in np.percentile(draws,[2.5,97.5]))
            result.append(dict(backend_id=key[0],frames=key[1],k=key[2],metric=metric,
                effect_definition='HEADLAND_minus_ROW_MIDDLE_scene_bootstrap',
                row_middle_scene_count=len(middle),headland_scene_count=len(headland),
                mean_difference=delta,ci95_lower=lo,ci95_upper=hi))
    return result


def write_csv(path: Path, rows: list[dict], fallback_fields: tuple[str, ...] = ('metric', 'observed_query_count')) -> None:
    fields = list(dict.fromkeys(key for row in rows for key in row)) or list(fallback_fields)
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


RECALL_METRICS = ['physical_row_recall', 'pose_region_recall_1m', 'pose_region_recall_2m',
                  'pose_region_recall_5m', 'headland_region_recall', 'xy_region_recall_1m',
                  'xy_region_recall_2m', 'xy_region_recall_5m']
WRONG_METRICS = ['wrong_row_fraction', 'wrong_row_fraction_known_rows', 'wrong_row_fraction_lower_bound',
                 'wrong_row_fraction_upper_bound', 'top1_wrong_row', 'wrong_row_known_count', 'wrong_row_unknown_count']
LONGITUDINAL_METRICS = ['longitudinal_median_m', 'longitudinal_p95_m', 'longitudinal_fraction_gt_1m',
                        'longitudinal_fraction_gt_2m', 'longitudinal_fraction_gt_5m', 'longitudinal_observed_count']
ENTROPY_METRICS = ['row_entropy', 'effective_row_count', 'entropy_known_weight_fraction', 'known_candidate_row_count']
SPREAD_METRICS = ['mean_radius_m', 'p95_radius_m', 'max_pairwise_distance_m', 's_spread_m', 'within_row_s_spread_max_m']
STAGE_METRICS = RECALL_METRICS + ['bbs_basin_hit', 'gicp_final_success', 'selected_bbs_nominal_success',
                 'selected_gicp_nominal_success', 'offline_reference_nominal_success',
                 'final_accepted_success', 'bbs_attempted_count', 'bbs_observed_count', 'bbs_censored_count',
                 'gicp_attempted_count', 'gicp_observed_count', 'gicp_censored_count']
EFFECT_METRICS = RECALL_METRICS + ['wrong_row_fraction', 'row_entropy', 'score_margin_m12',
                                'mean_radius_m', 'longitudinal_median_m', 'bbs_basin_hit', 'gicp_final_success']


def load_traces(inputs: list[Path]) -> tuple[list[dict], list[dict]]:
    paths = []
    for source in inputs:
        paths.extend(sorted(source.rglob('*.json')) if source.is_dir() else [source])
    traces, provenance = [], []
    for path in sorted(set(paths)):
        payload = path.read_bytes()
        data = json.loads(payload)
        documents = data if isinstance(data, list) else [data]
        # A directory may contain run manifests/validation JSON beside traces.
        if not all(isinstance(item,dict) and 'ranked_candidates' in item for item in documents):
            if path in inputs:
                raise ValueError(f'{path}: not a candidate trace')
            continue
        for trace in documents:
            validate_trace(trace)
            traces.append(trace)
        provenance.append({'path': str(path.resolve()), 'sha256': hashlib.sha256(payload).hexdigest(), 'query_count': len(documents)})
    if not traces:
        raise ValueError('no candidate traces found')
    return traces, provenance


def make_figures(output: Path, topology: dict, traces: list[dict], rows: list[dict], summaries: dict) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures = output / 'figures'
    figures.mkdir(exist_ok=True)
    def save(fig, name):
        fig.tight_layout()
        fig.savefig(figures/name, dpi=180)
        plt.close(fig)
    def draw_topology(ax):
        for row in topology.get('rows', []):
            xy = np.asarray(row['centerline'])
            ax.plot(xy[:,0],xy[:,1], linewidth=2, label=row['id'])
            ax.text(xy[0,0],xy[0,1],row['id'],fontsize=8)
        for region in topology.get('headlands', []):
            xy = np.asarray(region['polygon'])
            ax.fill(xy[:,0],xy[:,1],alpha=.18)
            ax.text(xy[:,0].mean(),xy[:,1].mean(),region['id'],fontsize=8)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('Canonical map x [m]'); ax.set_ylabel('Canonical map y [m]')
    fig,ax = plt.subplots(figsize=(8,6))
    draw_topology(ax)
    trajectories = defaultdict(list)
    try:
        from agt_greenhouse_annotation.topology import load_map_package
        package_path = topology.get('source',{}).get('map_package')
        if package_path and Path(package_path).is_dir():
            package=load_map_package(package_path)
            xy=np.asarray([[p['x'],p['y']] for p in package.poses])
            if len(xy): ax.plot(xy[:,0],xy[:,1],color='0.6',linewidth=.7,label='frontend trajectory')
    except (ValueError, OSError, KeyError):
        pass
    for trace in traces:
        query=trace['query']; pose=canonical_pose(topology,str(query.get('backend_id',UNKNOWN)),query.get('reference_pose') or {})
        if pose: trajectories[str(query.get('scene_type',UNKNOWN)).upper()].append([pose['x'],pose['y']])
    for scene, points in trajectories.items():
        xy=np.asarray(points);ax.scatter(xy[:,0],xy[:,1],s=22,label=scene)
    if not topology.get('rows'):
        ax.text(.5,.5,'Physical topology not manually confirmed',transform=ax.transAxes,ha='center')
    ax.set_title('Manual topology and same-session frontend reference')
    if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=7)
    save(fig,'fig_topology_annotation.png')
    for scene,filename in [('ROW_MIDDLE','fig_topk_candidates_row_middle.png'),('HEADLAND','fig_topk_candidates_headland.png')]:
        fig,ax=plt.subplots(figsize=(8,6));draw_topology(ax)
        selected=next((t for t in traces if str(t['query'].get('scene_type','')).upper()==scene),None)
        if selected:
            query=selected['query'];backend=str(query.get('backend_id',UNKNOWN))
            canonical=canonical_pose(topology,backend,query.get('reference_pose') or {})
            use_canonical=canonical is not None
            if not use_canonical:
                ax.clear()
                ax.set_aspect('equal', adjustable='datalim')
            qpose=canonical or query.get('reference_pose') or {}
            if finite(qpose.get('x')) and finite(qpose.get('y')):ax.scatter(qpose['x'],qpose['y'],marker='*',s=180,color='black',label='query reference')
            for c in selected['ranked_candidates'][:10]:
                pose=canonical_pose(topology,backend,c['map_pose']) if use_canonical else c['map_pose']
                if pose and all(finite(pose.get(k)) for k in ('x','y')):
                    ax.scatter(pose['x'],pose['y'],s=32)
                    score=c['descriptor'].get('sector_similarity');scoretext=f'{score:.3f}' if finite(score) else 'N/A'
                    ax.annotate(f"{c['rank']}: {scoretext}",(pose['x'],pose['y']),xytext=(4,4),textcoords='offset points',fontsize=7)
            ax.set_title(f"{scene} | {query.get('scene_id','')} | {backend} | N={query.get('frames',1)}")
            if not use_canonical:
                ax.set_xlabel('Backend map x [m]');ax.set_ylabel('Backend map y [m]')
                ax.text(.01,.01,'No backend-to-canonical transform: physical labels unavailable',transform=ax.transAxes,fontsize=8)
        else: ax.text(.5,.5,f'No {scene} trace',transform=ax.transAxes,ha='center')
        save(fig,filename)
    fig,axes=plt.subplots(1,3,figsize=(13,4),sharey=True)
    for ax,metric,title in zip(axes,['physical_row_recall','pose_region_recall_2m','xy_region_recall_2m'],['Physical row','Same row and |Δs| < 2 m','XY region < 2 m (proxy)']):
        found=False
        for backend in sorted({r['backend_id'] for r in rows}):
            for scene in ('ROW_MIDDLE','HEADLAND'):
                values=[r for r in summaries['recall'] if r['backend_id']==backend and r['scene_type']==scene and r['frames']==1 and r['metric']==metric]
                finite_values=[r for r in values if r['scene_weighted_mean'] is not None]
                if finite_values:
                    ax.plot([r['k'] for r in finite_values],[r['scene_weighted_mean'] for r in finite_values],marker='o',label=f'{backend} {scene}');found=True
        ax.set_title(title);ax.set_xticks(K_VALUES);ax.set_xlabel('K');ax.set_ylim(-.03,1.03)
        if not found:ax.text(.5,.5,'N/A: physical label/reference unavailable',transform=ax.transAxes,ha='center',fontsize=8)
    axes[0].set_ylabel('Scene-weighted Recall@K')
    handles,labels=axes[-1].get_legend_handles_labels()
    if handles:axes[-1].legend(fontsize=6)
    save(fig,'fig_recall_at_k.png')
    for metric,filename,ylabel in [('row_entropy','fig_row_entropy.png','Row entropy [nats] (known rows only)'),('longitudinal_median_m','fig_longitudinal_ambiguity.png','Per-query same-row median |Δs| [m]')]:
        fig,ax=plt.subplots(figsize=(8,4))
        distributions=[];names=[]
        for backend in sorted({r['backend_id'] for r in rows}):
            for scene in ('ROW_MIDDLE','HEADLAND'):
                values=[r[metric] for r in rows if r['backend_id']==backend and r['scene_type']==scene and r['frames']==1 and r['k']==10 and finite(r.get(metric))]
                if values:distributions.append(values);names.append(f'{backend}\n{scene}')
        if distributions:ax.boxplot(distributions,labels=names)
        else:ax.text(.5,.5,'N/A: no confirmed physical row labels',transform=ax.transAxes,ha='center')
        ax.set_ylabel(ylabel);ax.set_title('N=1, K=10; observations are queries')
        save(fig,filename)
    fig,ax=plt.subplots(figsize=(10,5))
    metrics=['xy_region_recall_2m','bbs_basin_hit','selected_gicp_nominal_success','final_accepted_success']
    for backend in sorted({r['backend_id'] for r in rows}):
        for scene in ('ROW_MIDDLE','HEADLAND'):
            values=[];counts=[]
            for metric in metrics:
                selected=[r for r in summaries['stages'] if r['backend_id']==backend and r['scene_type']==scene and r['frames']==1 and r['k']==10 and r['metric']==metric]
                values.append(selected[0]['scene_weighted_mean'] if selected and selected[0]['scene_weighted_mean'] is not None else np.nan)
                counts.append(selected[0]['observed_query_count'] if selected else 0)
            if any(math.isfinite(v) for v in values):
                ax.plot(range(4),values,marker='o',label=f'{backend} {scene}')
                for x,y,n in zip(range(4),values,counts):
                    if math.isfinite(y):ax.annotate(f'n={n}',(x,y),textcoords='offset points',xytext=(3,5),fontsize=7)
    ax.set_xticks(range(4),['Descriptor XY < 2 m proxy','BBS@K 1 m / 10° proxy','Selected GICP 0.5 m / 5°','Runtime accepted'])
    ax.set_ylim(-.04,1.08);ax.set_ylabel('Scene-weighted observed query rate')
    ax.set_title('N=1, K=10: missing/censored stages stay N/A; runtime acceptance may be unavailable')
    if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=7)
    save(fig,'fig_pipeline_failure_breakdown.png')


def run_analysis(trace_paths: list[Path], topology_path: Path, output: Path,
                 label_paths: list[Path] | None = None, entropy_mode: str = 'rank_frequency',
                 tau: float | None = None, bootstrap_samples: int = 2000, seed: int = 20261003,
                 figures: bool = True, allow_draft: bool = False, verify_map_files: bool = False) -> dict:
    from agt_greenhouse_annotation.topology import load_topology
    if bootstrap_samples < 0:
        raise ValueError('bootstrap_samples must be nonnegative')
    if entropy_mode == 'softmax' and (not finite(tau) or tau<=0):
        raise ValueError('softmax entropy requires explicitly fixed positive tau')
    initial_hash = sha256(topology_path)
    label_provenance = [{'path':str(p.resolve()),'sha256':sha256(p)} for p in (label_paths or [])]
    topology=load_topology(topology_path)
    annotation = topology.get('annotation', {})
    frozen_confirmed = annotation.get('status') == 'frozen' and annotation.get('manual_review_confirmed') is True
    if not frozen_confirmed and not allow_draft:
        raise ValueError('production analysis requires frozen, manually confirmed topology; --allow-draft is only for instrumentation smoke')
    source_package = topology.get('source', {}).get('map_package')
    topology_validation = None
    if not allow_draft or (source_package and Path(source_package).is_dir()):
        from agt_greenhouse_annotation.validator import validate_topology
        topology_validation = validate_topology(topology, annotation_path=topology_path,
                                                verify_map_files=verify_map_files)
        if not topology_validation['valid']:
            raise ValueError('topology/provenance validation failed: '+ '; '.join(topology_validation['errors']))
    else:
        topology_validation = {'valid': None, 'status': 'SYNTHETIC_INSTRUMENTATION_SMOKE_ONLY',
                               'reason': 'no real source map_package; production analysis would reject this input'}
    labels=load_labels(label_paths or [])
    traces,trace_provenance=load_traces(trace_paths)
    metrics=[];candidates=[]
    for trace in traces:
        query_rows,candidate_rows=analyze_trace(trace,topology,labels,entropy_mode,tau)
        metrics.extend(query_rows);candidates.extend(candidate_rows)
    identities=[(r['backend_id'],r['scene_id'],r['frames'],r['timestamp'],r['k']) for r in metrics]
    if len(set(identities))!=len(identities):
        raise ValueError('duplicate query identity; give repeated trials distinct scene/query IDs')
    output.mkdir(parents=True,exist_ok=True)
    tables=output/'tables';tables.mkdir(exist_ok=True)
    bundles={
        'recall':('table_topk_recall_by_scene.csv',RECALL_METRICS),
        'wrong':('table_wrong_row_by_scene.csv',WRONG_METRICS),
        'longitudinal':('table_longitudinal_ambiguity.csv',LONGITUDINAL_METRICS),
        'entropy':('table_candidate_entropy.csv',ENTROPY_METRICS),
        'margin':('table_score_margin.csv',['score_margin_m12']+SPREAD_METRICS),
        'stages':('table_bbs_gicp_stage_success.csv',STAGE_METRICS),
    }
    summaries={}
    for key,(filename,fields) in bundles.items():
        summaries[key]=aggregate(metrics,fields,bootstrap_samples,seed)
        write_csv(tables/filename,summaries[key])
    write_csv(tables/'table_multiframe_topk.csv',paired_effect(metrics,EFFECT_METRICS,'frames',bootstrap_samples,seed))
    write_csv(tables/'table_cross_backend_topk.csv',paired_effect(metrics,EFFECT_METRICS,'backend_id',bootstrap_samples,seed))
    write_csv(tables/'table_scene_effect_bootstrap.csv',scene_effects(metrics,EFFECT_METRICS,bootstrap_samples,seed))
    write_csv(tables/'query_metrics.csv',metrics)
    write_csv(tables/'candidate_row_coordinates.csv',candidates)
    if figures:make_figures(output,topology,traces,metrics,summaries)
    if sha256(topology_path)!=initial_hash:
        raise RuntimeError('frozen topology changed during analysis')
    for record in trace_provenance + label_provenance:
        if sha256(Path(record['path'])) != record['sha256']:
            raise RuntimeError(f"read-only input changed during analysis: {record['path']}")
    manifest={
        'schema_version':1,'status':'COMPLETE','query_count':len(traces),'candidate_count':len(candidates),
        'analysis_source_provenance':analysis_source_provenance(),
        'topology':{'path':str(topology_path.resolve()),'sha256':initial_hash,'read_only':True,
                    'status':annotation.get('status',UNKNOWN),'manual_review_confirmed':annotation.get('manual_review_confirmed',False)},
        'instrumentation_smoke_allow_draft':allow_draft,
        'topology_validation':topology_validation,
        'keyframe_labels':label_provenance,
        'canonical_backend_id':topology.get('source',{}).get('backend_id',UNKNOWN),
        'backend_transforms':topology.get('backend_transforms',{}),
        'traces':trace_provenance,'entropy_mode':entropy_mode,'softmax_tau':tau,
        'score_semantics':{'sector_similarity':'higher_is_better','ring_distance':'lower_is_better',
                           'margin_uses':'sector_similarity only'},
        'bootstrap':{'unit':'scene/query; never candidate','samples':bootstrap_samples,'seed':seed,
                     'ci':'percentile 95%; unavailable for one scene; equal scene weighting'},
        'reference_status':'MANUAL_TOPOLOGY + SAME_SESSION_FRONTEND_REFERENCE; not absolute ground truth',
        'thresholds':{'pose_region_delta_s_m':list(DISTANCE_THRESHOLDS_M),'xy_region_m':list(DISTANCE_THRESHOLDS_M),
                      'bbs_basin_proxy':{'xy_m':1.0,'yaw_deg':10.0},'gicp_nominal':{'xy_m':0.5,'yaw_deg':5.0}},
        'limitations':[
            'Physical-row and along-row recall are N/A for queries without a confirmed row; headlands normally have no row.',
            'Headland-region and XY-region recall are supplemental region metrics, not physical-row recall.',
            'UNKNOWN candidates are reported separately; they are not wrong rows or failed retrievals.',
            'WrongRowFraction uses the full prefix and is unavailable if labels are missing; known-row conditional fraction and bounds are supplemental.',
            'Row entropy normalizes over known physical rows only; known weight coverage is reported.',
            'Unattempted, timed-out invalid, or incompletely observed stage outcomes are censored, not failed.',
            'Native success is not runtime policy acceptance; FinalAcceptedSuccess remains N/A without runtime_policy evidence.',
            'K refers to descriptor prefix; stage@K uses only actual observed candidates in that prefix.',
            'A known hit makes Recall@K true; a miss is false only for a complete and fully observed prefix.',
            'Longitudinal summary rows average per-query candidate statistics, not independent candidate samples.',
            'Cross-backend effects pair scene IDs; explicit backend-to-canonical transforms are required for physical labels.',
            'A rigid same-timestamp alignment has residual error and does not establish physical ground truth; transformed labels require manual review against row spacing/corridors.',
            'Shared BBS time budgets mean stage outcomes in a prefix are not a counterfactual new run with a smaller candidate_top_k.'
        ]}
    (output/'METRIC_DEFINITIONS.md').write_text(
        '# Top-K metric definitions\n\n'+'\n'.join('- '+item for item in manifest['limitations'])+'\n\n'
        'Descriptor similarity is higher-is-better; ring distance is lower-is-better. '
        'M12 = (similarity1 − similarity2) / (|similarity1| + 1e−12); ring distance is never mixed into M12. '
        'Entropy defaults to uniform rank-frequency over known row labels. Softmax requires an explicit fixed tau. '
        'Δs is compared only within the same physical row. Pose-region uses strict |Δs| < 1/2/5 m; '
        'longitudinal ambiguity reports strict |Δs| > 1/2/5 m. XY-region uses strict distance < 1/2/5 m. '
        'BBS basin hit requires valid coarse XY ≤ 1 m and yaw ≤ 10° versus frontend reference; '
        'GICP requires converged final XY ≤ 0.5 m and yaw ≤ 5°. These are fixed reference-tolerance proxies.\n\n'
        'Eight core CSV files use scene/query units. Additional query and candidate CSVs retain missing values, '
        'observed/censored counts, physical labels, and raw descriptor scores. Scene contrasts use independent '
        'scene bootstrap; multiframe/backend contrasts use scene-paired bootstrap. One scene per group has no CI.\n',encoding='utf-8')
    artifacts = sorted(tables.glob('*.csv')) + sorted((output/'figures').glob('*.png')) + [output/'METRIC_DEFINITIONS.md']
    manifest['output_artifacts'] = [{'path':str(p.relative_to(output)), 'sha256':sha256(p), 'bytes':p.stat().st_size} for p in artifacts]
    (output/'analysis_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--traces',nargs='+',type=Path,required=True,help='JSON traces or directories, read only')
    parser.add_argument('--topology',type=Path,required=True,help='Frozen manually confirmed topology YAML')
    parser.add_argument('--keyframe-labels',type=Path,action='append',default=[])
    parser.add_argument('--output',type=Path,required=True,help='e.g. docs/paper/topk')
    parser.add_argument('--entropy-mode',choices=('rank_frequency','softmax'),default='rank_frequency')
    parser.add_argument('--softmax-tau',type=float,default=None)
    parser.add_argument('--bootstrap-samples',type=int,default=2000)
    parser.add_argument('--bootstrap-seed',type=int,default=20261003)
    parser.add_argument('--no-figures',action='store_true')
    parser.add_argument('--allow-draft',action='store_true',help='Instrumentation smoke only; unconfirmed topology yields N/A physical metrics')
    parser.add_argument('--verify-map-files',action='store_true',help='Also verify all map/patch checksums, beyond mandatory metadata/pose/provenance checks')
    args=parser.parse_args(argv)
    try:
        result=run_analysis(args.traces,args.topology,args.output,args.keyframe_labels,args.entropy_mode,
                            args.softmax_tau,args.bootstrap_samples,args.bootstrap_seed,not args.no_figures,args.allow_draft,args.verify_map_files)
    except (ValueError,OSError,RuntimeError) as exc:
        parser.error(str(exc))
    print(json.dumps({'status':result['status'],'query_count':result['query_count'],'output':str(args.output)},indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
