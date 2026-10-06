"""Editable, reviewed reference geometry; separate from physical ground truth."""
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import yaml

from .structure_artifacts import GlobalRowProposal, AisleProposal, GreenhouseStructureConfig, _plain


def simplify_reference_line(line, tolerance_m=.08):
    """Keep turns while removing grid-scale jitter and redundant edit handles."""
    xy = np.asarray(line, dtype=float).reshape(-1, 2)
    if len(xy) < 3:
        return tuple(tuple(float(v) for v in point) for point in xy)
    keep = {0, len(xy)-1}
    stack = [(0, len(xy)-1)]
    while stack:
        first, last = stack.pop()
        if last-first < 2:
            continue
        delta = xy[last]-xy[first]
        t = np.clip(((xy[first+1:last]-xy[first])@delta)/max(float(delta@delta), 1e-12), 0., 1.)
        distances = np.linalg.norm(xy[first+1:last]-(xy[first]+t[:, None]*delta), axis=1)
        index = int(np.argmax(distances))
        if distances[index] > tolerance_m:
            middle = first+1+index
            keep.add(middle)
            stack.extend(((first, middle), (middle, last)))
    return tuple(tuple(float(v) for v in xy[index]) for index in sorted(keep))


def reference_document(package, analysis, rows, aisles):
    return {
        'schema_version': 1,
        'artifact_type': 'greenhouse_reference_lines',
        'reference_status': 'MANUALLY_REVIEWED_REFERENCE',
        'absolute_ground_truth': False,
        'map_package_hash': package.map_package_hash,
        'map_frame': package.map_frame,
        'analysis_hash': analysis.analysis_hash if analysis else None,
        'config': analysis.config.to_dict() if analysis else {},
        'rows': [_plain(asdict(row)) for row in rows],
        'aisles': [_plain(asdict(aisle)) for aisle in aisles],
    }


def save_reference_draft(path, package, analysis, rows, aisles):
    document = reference_document(package, analysis, rows, aisles)
    document['reference_status'] = 'EDITABLE_REFERENCE_DRAFT'
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding='utf-8')
    temporary.replace(path)


def load_reference_draft(path, package):
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if value['map_package_hash'] != package.map_package_hash:
        raise ValueError('参考草稿与当前地图哈希不一致')
    rows = []
    aisles = []
    for item in value['rows']:
        item['centerline_xy'] = tuple(tuple(point) for point in item['centerline_xy'])
        item['direction_xy'] = tuple(item['direction_xy'])
        rows.append(GlobalRowProposal(**item))
    for item in value['aisles']:
        for field in ('centerline_xy', 'safe_centerline_xy'):
            item[field] = tuple(tuple(point) for point in item[field])
        aisles.append(AisleProposal(**item))
    return rows, aisles, GreenhouseStructureConfig.from_mapping(value['config']), value['analysis_hash']


def export_reviewed_reference(path, package, analysis, rows, aisles):
    active_rows = [row for row in rows if row.decision != 'rejected']
    active_aisles = [aisle for aisle in aisles if aisle.decision != 'rejected']
    if not active_rows and not active_aisles:
        raise ValueError('没有可导出的参考线，请先生成或绘制')
    features = []
    for kind, items in (('row', active_rows), ('aisle', active_aisles)):
        for item in items:
            if not item.diagnostics.get('reference_reviewed', False):
                raise ValueError(f'{item.auto_id} 尚未勾选审查完成')
            xy = np.asarray(item.centerline_xy, dtype=float)
            if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all() or np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() < .01:
                raise ValueError(f'{item.auto_id} 的参考线几何无效')
            features.append({'type': 'Feature', 'geometry': {'type': 'LineString', 'coordinates': xy.tolist()},
                'properties': {'id': item.auto_id, 'kind': kind, 'reference_reviewed': True,
                    'manually_edited': bool(item.diagnostics.get('manually_edited', False)),
                    'absolute_ground_truth': False}})
    path = Path(path)
    geojson = path.with_suffix('.geojson')
    if path.exists() or geojson.exists():
        raise FileExistsError('导出文件已存在，请另选文件名')
    path.parent.mkdir(parents=True, exist_ok=True)
    document = reference_document(package, analysis, active_rows, active_aisles)
    document['manual_review_confirmed'] = True
    path.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding='utf-8')
    geojson.write_text(json.dumps({'type': 'FeatureCollection', 'map_frame': package.map_frame,
        'map_package_hash': package.map_package_hash, 'features': features}, ensure_ascii=False, indent=2), encoding='utf-8')
    return path, geojson
