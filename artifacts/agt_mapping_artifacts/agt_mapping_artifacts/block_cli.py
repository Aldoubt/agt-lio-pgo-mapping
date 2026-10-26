"""Command line entry points for immutable keyframe block assets."""
from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

from .frontend_package import require_output_outside, sha256
from .frontend_package import _write_pcd
from .keyframe_blocks import build_keyframe_blocks, verify_keyframe_blocks


def _reviewed_physical_identity(annotation: dict, value) -> str:
    if (annotation.get('status') == 'frozen' and
            annotation.get('manual_review_confirmed') is True and
            isinstance(value, str) and value not in ('', 'UNKNOWN')):
        return value
    return 'UNKNOWN'


def _reviewed_labels(topology_path: Path, source: Path):
    from agt_greenhouse_annotation.topology import labels_for_package, load_map_package, load_topology
    from agt_greenhouse_annotation.validator import validate_topology

    topology = load_topology(topology_path)
    package = load_map_package(source)
    label_path = topology_path.parent / 'keyframe_topology_labels.csv'
    frozen = (topology.get('annotation', {}).get('status') == 'frozen' and
              topology.get('annotation', {}).get('manual_review_confirmed') is True)
    validation = validate_topology(topology, package=package,
                                   annotation_path=topology_path if frozen else None,
                                   labels_path=label_path if label_path.exists() else None,
                                   verify_map_files=True)
    if not validation.get('valid'):
        raise ValueError(f'topology validation failed: {validation.get("errors", [])}')
    annotation = topology.get('annotation', {})
    if not frozen:
        return {}, None
    frame_labels = {}
    scenes_by_frame = {int(scene['keyframe']): scene for scene in topology.get('scenes', [])
                       if scene.get('keyframe') is not None}
    for item in labels_for_package(topology, package):
        label = {
            'physical_row_id': item.get('physical_row_id', 'UNKNOWN'),
            'along_row_s_m': item.get('along_row_s_m'),
            'scene_type': item.get('zone_type', 'UNKNOWN'),
            'label_confidence': item.get('label_confidence', 'UNKNOWN'),
        }
        scene = scenes_by_frame.get(int(item['keyframe']))
        if scene is not None:
            manual_row = scene.get('manual_physical_row_id', 'UNKNOWN')
            if manual_row not in (None, '', 'UNKNOWN'):
                label.update(physical_row_id=str(manual_row),
                             along_row_s_m=item.get('along_row_s_m'),
                             scene_type=scene.get('scene_type', 'UNKNOWN'),
                             label_confidence='confirmed')
            elif scene.get('scene_type') not in (None, '', 'OTHER'):
                label.update(physical_row_id='UNKNOWN', along_row_s_m=None,
                             scene_type=scene.get('scene_type'), label_confidence='UNKNOWN')
        frame_labels[int(item['keyframe'])] = label
    return frame_labels, sha256(topology_path)


def build_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Build deterministic blocks from a verified frontend map package.')
    parser.add_argument('--source-package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--block-keyframe-count', type=int, required=True,
                        help='number of consecutive source keyframes in each map block')
    parser.add_argument('--stride', type=int,
                        help='block start stride; defaults to block-keyframe-count (non-overlapping)')
    parser.add_argument('--max-timestamp-gap-s', type=float, default=2.0)
    parser.add_argument('--voxel-leaf-m', type=float)
    parser.add_argument('--topology', type=Path,
                        help='optional existing agt_greenhouse_annotation schema v1 file')
    args = parser.parse_args(argv)
    labels, annotation_digest = ({}, None)
    if args.topology:
        labels, annotation_digest = _reviewed_labels(args.topology, args.source_package)
    manifest = build_keyframe_blocks(
        args.source_package, args.output,
        block_keyframe_count=args.block_keyframe_count, stride=args.stride,
        max_timestamp_gap_s=args.max_timestamp_gap_s, voxel_leaf_m=args.voxel_leaf_m,
        frame_labels=labels, row_annotation_sha256=annotation_digest)
    print(json.dumps({'status': 'PASS', 'block_set_id': manifest['block_set_id'],
                      'block_count': manifest['block_count'], 'output': str(args.output)}, indent=2))
    return 0


def verify_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Verify immutable keyframe block assets.')
    parser.add_argument('block_dir', type=Path)
    parser.add_argument('--source-package', type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(verify_keyframe_blocks(args.block_dir, source_package=args.source_package), indent=2))
    return 0


def preview_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Create a display-only sparse PCD of verified block bounds and centers.')
    parser.add_argument('--block-dir', type=Path, required=True)
    parser.add_argument('--source-package', type=Path, required=True)
    parser.add_argument('--output-pcd', type=Path, required=True)
    args = parser.parse_args(argv)
    block_root = args.block_dir.expanduser().resolve(strict=True)
    source_root = args.source_package.expanduser().resolve(strict=True)
    output = require_output_outside(args.output_pcd, [source_root, block_root],
                                    label='block preview output')
    if output.exists():
        raise FileExistsError(f'refusing to overwrite display preview: {output}')
    summary = verify_keyframe_blocks(block_root, source_package=source_root)
    import yaml
    root = block_root
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
    points = []
    corners = {}
    for block in manifest['blocks']:
        lower = np.asarray(block['bbox']['min_xyz_m'], dtype='f8')
        upper = np.asarray(block['bbox']['max_xyz_m'], dtype='f8')
        corners[block['block_id']] = [np.asarray([upper[i] if bit[i] else lower[i] for i in range(3)])
                                      for bit in itertools.product((0, 1), repeat=3)]
        center = np.asarray(block['center_pose_xyz_m'], dtype='f8')
        points.append(center)
        ids = list(itertools.product((0, 1), repeat=3))
        for index, bits in enumerate(ids):
            for axis in range(3):
                if bits[axis] != 0:
                    continue
                other = list(bits)
                other[axis] = 1
                a = corners[block['block_id']][index]
                b = np.asarray([upper[i] if other[i] else lower[i] for i in range(3)])
                count = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / 0.5)) + 1)
                points.extend(np.linspace(a, b, count))
    if not points:
        raise ValueError('block set contains no displayable bounds')
    _write_pcd(output, np.column_stack((np.asarray(points, dtype='<f4'), np.zeros(len(points), dtype='<f4'))))
    print(json.dumps({'status': 'PASS', 'display_only': True, 'block_set_id': summary['block_set_id'],
                      'block_count': summary['block_count'], 'output_pcd': str(output),
                      'output_sha256': sha256(output)}, indent=2))
    return 0


def structure_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Validate existing schema-v1 topology and export display-only structure points.')
    parser.add_argument('--source-package', type=Path, required=True)
    parser.add_argument('--topology', type=Path, required=True)
    parser.add_argument('--output-pcd', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    args = parser.parse_args(argv)
    from agt_greenhouse_annotation.topology import load_map_package, load_topology
    from agt_greenhouse_annotation.validator import validate_topology
    import yaml
    topology_path = args.topology.expanduser().resolve(strict=True)
    source = args.source_package.expanduser().resolve(strict=True)
    output_pcd = require_output_outside(args.output_pcd, [source, topology_path.parent],
                                        label='structure overlay output')
    output_json = require_output_outside(args.output_json, [source, topology_path.parent],
                                         label='structure metadata output')
    if output_pcd == output_json or output_pcd.exists() or output_json.exists():
        raise FileExistsError('structure overlay outputs must be distinct new files')
    topology = load_topology(topology_path)
    annotation = topology.get('annotation', {})
    topology_package = load_map_package(source)
    label_csv = topology_path.parent / 'keyframe_topology_labels.csv'
    validation = validate_topology(topology, package=topology_package,
                                   annotation_path=topology_path if annotation.get('status') == 'frozen' else None,
                                   labels_path=label_csv if label_csv.exists() else None,
                                   verify_map_files=True)
    if not validation.get('valid'):
        raise ValueError(f'topology/source validation failed: {validation.get("errors", [])}')
    z_values = np.asarray([float(p['z']) for p in topology_package.poses], dtype='f8')
    display_z = float(np.median(z_values)) if len(z_values) else 0.0
    points = []
    row_records = []
    for row in topology.get('rows', []):
        vertices = np.asarray(row.get('centerline', []), dtype='f8')
        if vertices.ndim != 2 or vertices.shape[1] != 2:
            continue
        for a, b in zip(vertices, vertices[1:]):
            count = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / 0.2)) + 1)
            points.extend(np.column_stack((np.linspace(a[0], b[0], count),
                                           np.linspace(a[1], b[1], count),
                                           np.full(count, display_z))))
        row_records.append({'id': row.get('id'), 'confidence': row.get('confidence', 'unconfirmed'),
                            'direction': row.get('direction', 'unknown'),
                            'centerline': vertices.tolist(),
                            'left_boundary': row.get('left_boundary'),
                            'right_boundary': row.get('right_boundary')})
    headland_records = []
    for headland in topology.get('headlands', []):
        polygon = np.asarray(headland.get('polygon', []), dtype='f8')
        if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
            continue
        closed = np.vstack((polygon, polygon[0]))
        for a, b in zip(closed, closed[1:]):
            count = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / 0.2)) + 1)
            points.extend(np.column_stack((np.linspace(a[0], b[0], count),
                                           np.linspace(a[1], b[1], count),
                                           np.full(count, display_z))))
        headland_records.append({'id': headland.get('id'), 'confidence': headland.get('confidence', 'unconfirmed'),
                                 'polygon': polygon.tolist()})
    pose_by_keyframe = {int(p['keyframe']): p for p in topology_package.poses}
    scenes = []
    for scene in topology.get('scenes', []):
        pose = pose_by_keyframe.get(int(scene.get('keyframe', -1)))
        if pose is None:
            continue
        scenes.append({'scene_id': scene.get('scene_id'), 'scene_type': scene.get('scene_type'),
                       'keyframe': int(scene['keyframe']), 'x': pose['x'], 'y': pose['y'],
                       'yaw_rad': pose['yaw_rad'],
                       'physical_row_id': _reviewed_physical_identity(
                           annotation, scene.get('manual_physical_row_id')),
                       'headland_id': _reviewed_physical_identity(
                           annotation, scene.get('manual_headland_id'))})
        # A compact cross marker, placed at the source trajectory height.
        for dx, dy in ((-0.2, 0), (0.2, 0), (0, -0.2), (0, 0.2)):
            points.append(np.asarray([pose['x'] + dx, pose['y'] + dy, display_z]))
    if points:
        _write_pcd(output_pcd,
                   np.column_stack((np.asarray(points, dtype='<f4'), np.zeros(len(points), dtype='<f4'))))
    data = {'schema_version': 1, 'asset_type': 'agt.structure_display_overlay/v1',
            'topology_sha256': sha256(topology_path),
            'source_manifest_sha256': topology_package.manifest_sha256,
            'map_frame': topology_package.map_frame,
            'annotation_status': annotation.get('status', 'UNKNOWN'),
            'manual_review_confirmed': annotation.get('manual_review_confirmed') is True,
            'display_z_m': display_z, 'rows': row_records,
            'headlands': headland_records, 'scenes': scenes,
            'physical_identity_rule': 'unconfirmed geometry never assigns a physical row ID',
            'overlay_pcd': str(output_pcd) if points else None,
            'overlay_sha256': sha256(output_pcd) if points else None,
            'validation': validation}
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'status': 'PASS', 'rows': len(row_records),
                      'headlands': len(headland_records), 'scenes': len(scenes),
                      'annotation_status': data['annotation_status'],
                      'manual_review_confirmed': data['manual_review_confirmed'],
                      'output_json': str(output_json), 'overlay_pcd': data['overlay_pcd']}, indent=2))
    return 0
