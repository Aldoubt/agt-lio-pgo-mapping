"""Validate physical geometry, source binding, scene correspondence and labels."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Polygon

from .topology import (REFERENCE_STATUS, labels_for_package, load_map_package,
                       load_topology, sha256_file)


def validate_topology(topology, package=None, annotation_path=None, labels_path=None,
                      verify_map_files=False):
    errors, warnings = [], []
    result = dict(schema_version=1, valid=False, errors=errors, warnings=warnings)
    if not isinstance(topology, dict):
        errors.append('topology must be a mapping')
        return result
    def mapping(name):
        value = topology.get(name, {})
        if not isinstance(value, dict):
            errors.append(f'{name} must be a mapping')
            return {}
        return value
    def entries_for(name):
        value = topology.get(name, [])
        if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
            errors.append(f'{name} must be a list of mappings')
            return []
        return value
    source, annotation = mapping('source'), mapping('annotation')
    scenes = entries_for('scenes')
    transforms = mapping('backend_transforms')
    if topology.get('schema_version') != 1:
        errors.append('schema_version must be 1')
    if annotation.get('status') not in ('draft', 'frozen'):
        errors.append('annotation.status must be draft or frozen')
    if annotation.get('annotator') != 'manual':
        errors.append('physical topology requires annotation.annotator=manual')
    if annotation.get('reference_status') != REFERENCE_STATUS or annotation.get('absolute_ground_truth') is not False:
        errors.append('reference status must describe manual topology plus same-session frontend, absolute_ground_truth=false')
    if annotation.get('status') == 'frozen' and not annotation.get('manual_review_confirmed'):
        errors.append('frozen annotation requires explicit manual_review_confirmed')
    settings = mapping('assignment')
    for key in ('max_row_assignment_distance_m', 'scene_timestamp_tolerance_s'):
        try:
            value = float(settings.get(key, 1.0 if key.startswith('max') else .5))
            if not math.isfinite(value) or value <= 0:
                errors.append(f'assignment.{key} must be finite and positive')
        except (TypeError, ValueError):
            errors.append(f'assignment.{key} must be numeric')
    def check_finite(value, context='annotation'):
        if isinstance(value, float) and not math.isfinite(value):
            errors.append(f'NaN/Inf in {context}')
        elif isinstance(value, dict):
            for key, sub in value.items():
                check_finite(sub, context+'.'+str(key))
        elif isinstance(value, list):
            for sub in value:
                check_finite(sub, context)
    check_finite(topology)
    corridors = []
    for collection, geometry in (('rows', 'centerline'), ('headlands', 'polygon')):
        entries = entries_for(collection)
        ids = [item.get('id') for item in entries]
        if any(not isinstance(i, str) or not i.strip() for i in ids) or len(ids) != len(set(ids)):
            errors.append(f'{collection}: IDs must be unique nonempty strings')
        for item in entries:
            identifier = item.get('id', 'UNKNOWN')
            try:
                points = np.asarray(item[geometry], dtype=float)
                minimum = 2 if geometry == 'centerline' else 3
                if points.ndim != 2 or points.shape[1] != 2 or len(points) < minimum or not np.isfinite(points).all():
                    raise ValueError(f'requires at least {minimum} finite XY vertices')
                if geometry == 'centerline':
                    line = LineString(points)
                    width = float(item['nominal_width_m'])
                    if line.length <= 1e-9 or not line.is_simple:
                        raise ValueError('centerline must have positive length and not self-intersect')
                    if not math.isfinite(width) or width <= 0:
                        raise ValueError('nominal_width_m must be finite and positive')
                    if item.get('direction') not in ('bidirectional', 'forward', 'reverse', 'unknown'):
                        raise ValueError('unknown direction value')
                    if item.get('confidence') not in ('confirmed', 'unconfirmed'):
                        raise ValueError('confidence must be confirmed or unconfirmed')
                    corridors.append((identifier, line.buffer(width/2, cap_style=2)))
                else:
                    polygon = Polygon(points)
                    if not polygon.is_valid or polygon.area <= 1e-9:
                        raise ValueError('headland polygon must be valid with positive area')
                if annotation.get('status') == 'frozen' and item.get('confidence') != 'confirmed':
                    errors.append(f'{identifier}: frozen physical geometry must be manually confirmed')
            except (ValueError, KeyError, TypeError) as exc:
                errors.append(f'{identifier}: {exc}')
    try:
        overlap_limit = float(settings.get('large_corridor_overlap_fraction', .2))
        if not math.isfinite(overlap_limit) or not 0 <= overlap_limit <= 1:
            raise ValueError('must be finite between 0 and 1')
    except (ValueError, TypeError):
        errors.append('large_corridor_overlap_fraction must be finite between 0 and 1')
        overlap_limit = .2
    overlaps = []
    for i, (name_a, a) in enumerate(corridors):
        for name_b, b in corridors[i+1:]:
            area = a.intersection(b).area
            fraction = area/max(min(a.area, b.area), 1e-12)
            if fraction > 1e-6:
                overlaps.append(dict(rows=[name_a, name_b], area_m2=area, fraction_of_smaller=fraction))
            if fraction > overlap_limit:
                errors.append(f'abnormally large row corridor overlap: {name_a}/{name_b} {fraction:.3f} > {overlap_limit}')
    result['corridor_overlaps'] = overlaps
    scene_ids = [s.get('scene_id') for s in scenes]
    if any(not isinstance(i, str) or not i for i in scene_ids) or len(scene_ids) != len(set(scene_ids)):
        errors.append('scene IDs must be unique nonempty strings')
    if any(isinstance(i, str) and any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in i) for i in scene_ids):
        errors.append('scene IDs must use ASCII letters, digits, _ or - for the existing benchmark')
    if annotation.get('status') == 'frozen' and not topology.get('rows') and not topology.get('headlands'):
        errors.append('cannot freeze empty physical topology')
    try:
        package = package or load_map_package(source['map_package'])
        if package.manifest_sha256 != source.get('map_package_manifest_sha256'):
            errors.append('source manifest hash mismatch')
        if package.map_package_hash != source.get('map_package_hash'):
            errors.append('source map_package hash mismatch')
        if package.map_frame != source.get('map_frame'):
            errors.append('source coordinate frame mismatch')
        if package.backend_id != source.get('backend_id'):
            errors.append('source backend ID mismatch')
        backend_commit = package.metadata.get('mapping_backend', {}).get('source_commit', 'UNKNOWN')
        if source.get('backend_commit') != backend_commit:
            errors.append('source backend commit mismatch')
        bag = source.get('rosbag')
        bag_hash = source.get('rosbag_metadata_sha256')
        if bag_hash and (not bag or not (Path(bag)/'metadata.yaml').exists() or sha256_file(Path(bag)/'metadata.yaml') != bag_hash):
            errors.append('rosbag metadata hash mismatch')
        for scene in scenes:
            identifier = scene.get('scene_id', 'UNKNOWN')
            try:
                timestamp = float(scene['bag_timestamp'])
                if not math.isfinite(timestamp):
                    raise ValueError('scene timestamp must be finite')
                nearest = package.nearest_timestamp(timestamp)
                dt = abs(nearest['timestamp']-timestamp)
                tolerance = float(settings.get('scene_timestamp_tolerance_s', .5))
                if dt > tolerance:
                    errors.append(f'{identifier}: timestamp has no keyframe within {tolerance}s (dt={dt:.6f}s)')
                keyframe = scene.get('keyframe')
                if keyframe is not None and keyframe not in {p['keyframe'] for p in package.poses}:
                    errors.append(f'{identifier}: keyframe not present in map_package')
                elif keyframe is not None:
                    declared = next(p for p in package.poses if p['keyframe'] == keyframe)
                    if abs(declared['timestamp']-timestamp) > tolerance:
                        errors.append(f'{identifier}: scene timestamp and declared keyframe mismatch')
                if scene.get('scene_type') not in ('ROW_ENTRY', 'ROW_MIDDLE', 'ROW_END', 'HEADLAND', 'OTHER'):
                    errors.append(f'{identifier}: invalid scene type')
                manual_row = scene.get('manual_physical_row_id', 'UNKNOWN')
                if manual_row != 'UNKNOWN' and manual_row not in {r['id'] for r in topology.get('rows', [])}:
                    errors.append(f'{identifier}: manual physical row does not exist')
                manual_headland = scene.get('manual_headland_id', 'UNKNOWN')
                if manual_headland != 'UNKNOWN' and manual_headland not in {h['id'] for h in topology.get('headlands', [])}:
                    errors.append(f'{identifier}: manual headland does not exist')
            except (KeyError, ValueError, TypeError) as exc:
                errors.append(f'{identifier}: {exc}')
        for backend, transform in transforms.items():
            if not isinstance(transform, dict):
                errors.append(f'{backend}: transform must be a mapping')
                continue
            provenance = transform.get('provenance', {})
            if not isinstance(provenance, dict):
                errors.append(f'{backend}: transform provenance must be a mapping')
                continue
            try:
                matrix = np.asarray(transform.get('matrix'), dtype=float)
            except (ValueError, TypeError):
                errors.append(f'{backend}: backend matrix must be finite 4x4')
                continue
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                errors.append(f'{backend}: backend matrix must be finite 4x4')
                continue
            if not np.allclose(matrix[3], [0, 0, 0, 1]) or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5):
                errors.append(f'{backend}: backend transform is not rigid')
            if not np.allclose(matrix[2], [0, 0, 1, 0], atol=1e-5) or not np.allclose(matrix[:2, 2], [0, 0], atol=1e-5):
                errors.append(f'{backend}: topology correspondence transform must be SE2')
            if transform.get('target_backend') != package.backend_id or transform.get('target_frame') != package.map_frame:
                errors.append(f'{backend}: backend transform target mismatch')
            if provenance.get('target_manifest_sha256') != package.manifest_sha256:
                errors.append(f'{backend}: backend transform canonical provenance mismatch')
            other_path = provenance.get('source_map_package')
            try:
                other = load_map_package(other_path)
                if (other.backend_id != backend or other.map_frame != transform.get('source_frame') or
                    other.manifest_sha256 != provenance.get('source_manifest_sha256')):
                    errors.append(f'{backend}: backend source provenance mismatch')
                if package.metadata.get('source', {}).get('rosbag') != other.metadata.get('source', {}).get('rosbag'):
                    errors.append(f'{backend}: backend correspondence requires the same rosbag source')
                if verify_map_files:
                    checked_other = 0
                    for name, digest in other.manifest.get('files', {}).items():
                        file = (other.path/name).resolve()
                        if other.path not in file.parents or not file.exists() or sha256_file(file) != digest:
                            errors.append(f'{backend}: source file checksum mismatch {name}')
                        checked_other += 1
                    result.setdefault('other_backend_map_files_checksum_verified', {})[backend] = checked_other
            except (ValueError, TypeError, OSError) as exc:
                errors.append(f'{backend}: backend source validation: {exc}')
        labels = labels_for_package(topology, package) if not errors else []
        if labels:
            unknown = sum(p['zone_type'] == 'UNKNOWN' for p in labels)
            result['coverage'] = dict(keyframes=len(labels), labeled=len(labels)-unknown,
                                      unknown=unknown, unknown_percentage=100*unknown/len(labels))
            if unknown/len(labels) > .5:
                warnings.append(f'UNKNOWN coverage {100*unknown/len(labels):.1f}%: manual topology does not cover most poses')
        if labels_path:
            with Path(labels_path).open(newline='') as stream:
                stored = list(csv.DictReader(stream))
            keyframes = [int(p['keyframe']) for p in stored]
            if set(keyframes) != {p['keyframe'] for p in package.poses} or len(keyframes) != len(set(keyframes)):
                errors.append('keyframe label CSV coverage or uniqueness mismatch')
            expected = {p['keyframe']: p for p in labels}
            for row in stored:
                e = expected.get(int(row['keyframe']))
                if e and any(row.get(k) != str(e[k]) for k in ('physical_row_id', 'zone_type', 'headland_id')):
                    errors.append(f'keyframe {row["keyframe"]}: stored labels disagree with topology')
        # Small mandatory source hashes are checked even without full PCD scan.
        checked = 0
        for name, expected_hash in package.manifest.get('files', {}).items():
            if verify_map_files or name in ('metadata.yaml', 'poses_timed.txt'):
                file = (package.path/name).resolve()
                if package.path not in file.parents:
                    errors.append(f'unsafe manifest path: {name}')
                    continue
                if not file.exists() or sha256_file(file) != expected_hash:
                    errors.append(f'map_package content hash mismatch: {name}')
                checked += 1
        result['map_files_checksum_verified'] = checked
    except (ValueError, KeyError, TypeError, OSError) as exc:
        errors.append(f'map_package/label validation: {exc}')
    if annotation_path:
        annotation_path = Path(annotation_path)
        manifest_path = annotation_path.with_suffix('.manifest.json')
        if not manifest_path.exists():
            errors.append('annotation provenance manifest is missing')
        else:
            try:
                manifest = json.loads(manifest_path.read_text())
                if manifest.get('annotation_file_sha256') != sha256_file(annotation_path):
                    errors.append('annotation file hash/provenance mismatch')
                for filename, digest in manifest.get('sidecar_hashes', {}).items():
                    file = annotation_path.parent/filename
                    if not file.exists() or sha256_file(file) != digest:
                        errors.append(f'annotation sidecar hash mismatch: {filename}')
            except (ValueError, OSError) as exc:
                errors.append(f'annotation provenance invalid: {exc}')
    result['valid'] = not errors
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('annotation')
    parser.add_argument('--labels')
    parser.add_argument('--output')
    parser.add_argument('--require-frozen', action='store_true')
    parser.add_argument('--verify-map-files', action='store_true')
    args = parser.parse_args(argv)
    topology = load_topology(args.annotation)
    default_labels = Path(args.annotation).parent/'keyframe_topology_labels.csv'
    result = validate_topology(topology, annotation_path=args.annotation,
        labels_path=args.labels or (default_labels if default_labels.exists() else None),
        verify_map_files=args.verify_map_files)
    if args.require_frozen and topology.get('annotation', {}).get('status') != 'frozen':
        result['errors'].append('benchmark requires frozen topology')
        result['valid'] = False
    output = Path(args.output or Path(args.annotation).parent/'annotation_validation.json')
    output.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2))
    return 0 if result['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
