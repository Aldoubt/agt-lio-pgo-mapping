"""Geometry and immutable export API; this module has no Qt or ROS dependency.

Distances are metres, yaw arguments radians. Signed d is positive to the left
of the centerline's stored vertex order. Direction is metadata and does not
silently reverse s/d. Automatic assignment requires both the manual corridor
(width/2) and max_row_assignment_distance_m; ambiguity remains UNKNOWN.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import copy
import csv
import hashlib
import json
import math
import os
import subprocess

import numpy as np
import yaml
from shapely.geometry import Point, Polygon

from . import __version__

REFERENCE_STATUS = 'MANUAL_TOPOLOGY_PLUS_SAME_SESSION_FRONTEND_REFERENCE'


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def now_iso():
    return datetime.now(timezone.utc).isoformat()


def git_commit(path):
    try:
        return subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'],
            stderr=subprocess.DEVNULL, text=True).strip()
    except (subprocess.SubprocessError, OSError):
        return 'UNKNOWN'


def wrap_radians(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class MapPackage:
    path: Path
    metadata: dict
    manifest: dict
    poses: list
    map_frame: str
    backend_id: str
    manifest_sha256: str
    map_package_hash: str

    def nearest_timestamp(self, timestamp):
        return min(self.poses, key=lambda p: abs(p['timestamp'] - timestamp))


def load_map_package(path):
    """Load timed pose metadata, not the large map cloud.

    poses_timed convention is patch timestamp x y z qw qx qy qz, following
    the existing AGT exporter. Map hash is the content hash of manifest.yaml,
    whose file checksums commit to map/poses/patches. Validator can verify those
    checksums against bytes with --verify-map-files.
    """
    root = Path(path).expanduser().resolve()
    for name in ('map.pcd', 'poses_timed.txt', 'metadata.yaml', 'manifest.yaml', 'patches'):
        if not (root / name).exists():
            raise ValueError(f'map_package missing {name}: {root}')
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text()) or {}
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text()) or {}
    files = manifest.get('files', {})
    for name in ('map.pcd', 'metadata.yaml', 'poses_timed.txt'):
        digest = files.get(name)
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError(f'map_package manifest needs SHA256 for {name}')
        if name != 'map.pcd' and sha256_file(root/name) != digest:
            raise ValueError(f'map_package manifest checksum mismatch: {name}')
    poses = []
    for lineno, line in enumerate((root / 'poses_timed.txt').read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        values = line.split()
        if len(values) != 9:
            raise ValueError(f'poses_timed.txt line {lineno}: expected 9 columns')
        patch = values[0]
        timestamp, x, y, z, qw, qx, qy, qz = map(float, values[1:])
        if not all(math.isfinite(v) for v in (timestamp, x, y, z, qw, qx, qy, qz)):
            raise ValueError(f'nonfinite pose at line {lineno}')
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm < 1e-12:
            raise ValueError(f'invalid quaternion at line {lineno}')
        qw, qx, qy, qz = (v / norm for v in (qw, qx, qy, qz))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        poses.append(dict(keyframe=int(Path(patch).stem), patch=patch,
                          timestamp=timestamp, x=x, y=y, z=z,
                          yaw_rad=yaw, yaw=yaw, yaw_deg=math.degrees(yaw)))
    if not poses:
        raise ValueError('map_package has no timed poses')
    if len({p['keyframe'] for p in poses}) != len(poses):
        raise ValueError('duplicate keyframe ID in map_package')
    if any(b['timestamp'] < a['timestamp'] for a, b in zip(poses, poses[1:])):
        raise ValueError('map_package timestamps must be monotonic')
    frame = metadata.get('frames', {}).get('map')
    if not frame:
        raise ValueError('metadata.frames.map is required')
    backend = metadata.get('mapping_backend', {}).get('id', 'UNKNOWN')
    manifest_hash = sha256_file(root / 'manifest.yaml')
    return MapPackage(root, metadata, manifest, poses, frame, backend,
                      manifest_hash, manifest_hash)


def new_topology(package, max_assignment_distance_m=1.0):
    bag = package.metadata.get('source', {}).get('rosbag')
    bag_metadata = Path(bag) / 'metadata.yaml' if bag else None
    repo = Path(__file__).resolve().parents[3]
    return {
        'schema_version': 1,
        'source': {
            'map_package': str(package.path),
            'map_package_manifest_sha256': package.manifest_sha256,
            'map_package_hash': package.map_package_hash,
            'map_package_hash_semantics': 'sha256_of_checksum_manifest',
            'map_frame': package.map_frame,
            'backend_id': package.backend_id,
            'backend_commit': package.metadata.get('mapping_backend', {}).get('source_commit', 'UNKNOWN'),
            'mapping_repository_commit': git_commit(repo),
            'rosbag': bag,
            'rosbag_metadata_sha256': sha256_file(bag_metadata) if bag_metadata and bag_metadata.exists() else None,
        },
        'annotation': {
            'created_at': now_iso(), 'tool_version': __version__,
            'tool_commit': git_commit(repo), 'annotator': 'manual',
            'status': 'draft', 'manual_review_confirmed': False,
            'reference_status': REFERENCE_STATUS,
            'absolute_ground_truth': False,
        },
        'assignment': {
            'max_row_assignment_distance_m': float(max_assignment_distance_m),
            'overlap_ambiguity_distance_m': 0.05,
            'scene_timestamp_tolerance_s': 0.5,
            'large_corridor_overlap_fraction': 0.2,
        },
        'rows': [], 'headlands': [], 'scenes': [], 'backend_transforms': {},
    }


def load_topology(path):
    topology = yaml.safe_load(Path(path).read_text())
    if not isinstance(topology, dict) or topology.get('schema_version') != 1:
        raise ValueError('greenhouse topology schema_version must be 1')
    return topology


def project_to_row(row, x, y, yaw_rad=0.0):
    points = np.asarray(row['centerline'], dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        raise ValueError('row centerline needs at least two XY vertices')
    if not np.isfinite(points).all() or not all(math.isfinite(float(v)) for v in (x, y, yaw_rad)):
        raise ValueError('nonfinite row projection input')
    p = np.array([x, y], dtype=float)
    best = None
    cumulative = 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length <= 1e-12:
            continue
        unit = delta / length
        t = float(np.clip(np.dot(p-a, unit), 0, length))
        projected = a + t*unit
        residual = p-projected
        distance = float(np.linalg.norm(residual))
        if best is None or distance < best['distance_m']:
            row_yaw = math.atan2(unit[1], unit[0])
            best = dict(physical_row_id=row['id'], along_row_s_m=cumulative+t,
                        lateral_d_m=float(unit[0]*residual[1] - unit[1]*residual[0]),
                        relative_heading_deg=math.degrees(wrap_radians(yaw_rad-row_yaw)),
                        distance_m=distance, projection_xy=projected.tolist(),
                        row_yaw_rad=row_yaw, segment_index=index)
        cumulative += length
    if best is None:
        raise ValueError('row centerline has zero total length')
    return best


def label_pose(topology, x, y, yaw_rad):
    if not all(math.isfinite(float(v)) for v in (x, y, yaw_rad)):
        raise ValueError('nonfinite pose')
    label = dict(physical_row_id='UNKNOWN', along_row_s_m=None, lateral_d_m=None,
                 relative_heading_deg=None, zone_type='UNKNOWN',
                 headland_id='UNKNOWN', label_confidence='UNKNOWN')
    settings = topology.get('assignment', {})
    maximum = float(settings.get('max_row_assignment_distance_m', 1.0))
    candidates = []
    for row in topology.get('rows', []):
        if row.get('confidence') != 'confirmed':
            continue
        projected = project_to_row(row, x, y, yaw_rad)
        corridor = min(maximum, float(row['nominal_width_m']) / 2.0)
        if projected['distance_m'] <= corridor + 1e-10:
            candidates.append((projected['distance_m'], row, projected))
    candidates.sort(key=lambda item: item[0])
    tie = len(candidates) > 1 and candidates[1][0] - candidates[0][0] <= float(
        settings.get('overlap_ambiguity_distance_m', .05))
    if candidates and not tie:
        _, row, projected = candidates[0]
        for key in ('physical_row_id', 'along_row_s_m', 'lateral_d_m', 'relative_heading_deg'):
            label[key] = projected[key]
        label.update(zone_type='ROW', label_confidence=row.get('confidence', 'unconfirmed'))
    headlands = [h for h in topology.get('headlands', []) if h.get('confidence') == 'confirmed'
                 if Polygon(h['polygon']).covers(Point(float(x), float(y)))]
    if len(headlands) == 1:
        label.update(zone_type='HEADLAND', headland_id=headlands[0]['id'],
                     label_confidence=headlands[0].get('confidence', 'confirmed'))
    elif len(headlands) > 1:
        label.update(zone_type='UNKNOWN', headland_id='UNKNOWN', label_confidence='AMBIGUOUS')
    elif tie:
        label['label_confidence'] = 'AMBIGUOUS'
    return label


def transform_pose(matrix, x, y, yaw_rad):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('backend transform needs finite 4x4 matrix')
    point = matrix @ np.array([x, y, 0, 1])
    yaw = wrap_radians(yaw_rad + math.atan2(matrix[1, 0], matrix[0, 0]))
    return float(point[0]), float(point[1]), yaw


def align_backend(canonical, other, max_dt_s=0.5):
    """Fit only SE(2) from same-bag nearest timed keyframes, saving residuals.

    This correspondence does not establish physical ground truth. All poses
    stay rigidly transformed; no trajectory correction is performed.
    """
    bag_a = canonical.metadata.get('source', {}).get('rosbag')
    bag_b = other.metadata.get('source', {}).get('rosbag')
    if not bag_a or not bag_b or Path(bag_a).resolve() != Path(bag_b).resolve():
        raise ValueError('backend correspondence requires the same recorded rosbag')
    pairs = [(p, other.nearest_timestamp(p['timestamp'])) for p in canonical.poses]
    pairs = [(a, b) for a, b in pairs if abs(a['timestamp']-b['timestamp']) <= max_dt_s]
    if len(pairs) < 3:
        raise ValueError('at least three timestamp correspondences are required')
    target = np.array([[a['x'], a['y']] for a, b in pairs])
    source = np.array([[b['x'], b['y']] for a, b in pairs])
    src_mean, dst_mean = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-src_mean).T @ (target-dst_mean))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = dst_mean - rotation @ src_mean
    matrix = np.eye(4)
    matrix[:2, :2], matrix[:2, 3] = rotation, translation
    residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    return dict(matrix=matrix.tolist(), source_frame=other.map_frame,
                target_frame=canonical.map_frame, source_backend=other.backend_id,
                target_backend=canonical.backend_id,
                method='same_bag_timestamp_se2_least_squares',
                correspondences=len(pairs), max_timestamp_difference_s=max_dt_s,
                residual_xy_m=dict(median=float(np.median(residual)),
                                   p95=float(np.percentile(residual, 95)), max=float(residual.max())),
                provenance=dict(source_manifest_sha256=other.manifest_sha256,
                                target_manifest_sha256=canonical.manifest_sha256,
                                source_map_package=str(other.path)))


def labels_for_package(topology, package):
    canonical = topology['source']['backend_id']
    matrix = None
    if package.backend_id != canonical:
        transform = topology.get('backend_transforms', {}).get(package.backend_id)
        if not transform:
            raise ValueError(f'no {package.backend_id} -> canonical backend transform')
        matrix = transform['matrix']
    result = []
    for pose in package.poses:
        x, y, yaw = pose['x'], pose['y'], pose['yaw_rad']
        cx, cy, cyaw = transform_pose(matrix, x, y, yaw) if matrix is not None else (x, y, yaw)
        row = dict(keyframe=pose['keyframe'], timestamp=pose['timestamp'],
                   x=x, y=y, yaw=yaw, yaw_unit='radians', backend_id=package.backend_id,
                   canonical_x=cx, canonical_y=cy, canonical_yaw=cyaw)
        row.update(label_pose(topology, cx, cy, cyaw))
        result.append(row)
    return result


def write_csv(path, rows, fieldnames=None):
    if fieldnames is None:
        fieldnames = list(rows[0]) if rows else []
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def geojson(topology):
    features = []
    for row in topology.get('rows', []):
        features.append(dict(type='Feature', properties={k: v for k, v in row.items() if k != 'centerline'},
                             geometry=dict(type='LineString', coordinates=row['centerline'])))
    for headland in topology.get('headlands', []):
        ring = copy.deepcopy(headland['polygon'])
        if ring and ring[0] != ring[-1]:
            ring.append(ring[0])
        features.append(dict(type='Feature', properties={k: v for k, v in headland.items() if k != 'polygon'},
                             geometry=dict(type='Polygon', coordinates=[ring])))
    for scene in topology.get('scenes', []):
        if 'x' in scene and 'y' in scene:
            features.append(dict(type='Feature', properties=copy.deepcopy(scene),
                                 geometry=dict(type='Point', coordinates=[scene['x'], scene['y']])))
    return dict(type='FeatureCollection', coordinate_reference=dict(
        frame=topology['source']['map_frame'], units='metres', georeferenced=False), features=features)


def scene_correspondence(topology, canonical, other):
    result = []
    tolerance = topology.get('assignment', {}).get('scene_timestamp_tolerance_s', .5)
    for scene in topology.get('scenes', []):
        timestamp = float(scene['bag_timestamp'])
        a, b = canonical.nearest_timestamp(timestamp), other.nearest_timestamp(timestamp)
        row = dict(scene_id=scene['scene_id'], bag_timestamp=timestamp)
        for package, pose in ((canonical, a), (other, b)):
            # Stable conventional aliases requested by the field protocol.
            name = {'point_lio': 'pointlio', 'fast_livo2_lio': 'fastlivo2'}.get(package.backend_id, package.backend_id)
            row[name+'_keyframe'] = pose['keyframe']
            row[name+'_dt'] = pose['timestamp']-timestamp
            row[name+'_within_tolerance'] = abs(pose['timestamp']-timestamp) <= tolerance
        result.append(row)
    return result


def scene_candidates(topology, labels):
    """Review suggestions only; never inserts physical IDs or manual scenes."""
    lengths = {r['id']: sum(math.dist(a, b) for a, b in zip(r['centerline'], r['centerline'][1:]))
               for r in topology.get('rows', [])}
    result = []
    for label in labels:
        if label['zone_type'] == 'UNKNOWN':
            continue
        suggestion = 'HEADLAND'
        if label['zone_type'] == 'ROW':
            fraction = label['along_row_s_m']/max(lengths[label['physical_row_id']], 1e-12)
            suggestion = 'ROW_ENTRY' if fraction < .15 else ('ROW_END' if fraction > .85 else 'ROW_MIDDLE')
        result.append(dict(keyframe=label['keyframe'], bag_timestamp=label['timestamp'],
                           suggested_scene_type=suggestion, physical_row_id=label['physical_row_id'],
                           status='NEEDS_MANUAL_REVIEW', rule='stored_centerline_fraction_0.15_0.85'))
    return result


def benchmark_scenes(topology, package):
    """Export only explicit manually chosen markers into the existing runner.

    keyframe is the existing benchmark's pose-list index; map_keyframe_id keeps
    the original PCD stem. Five-frame boundary/OTHER/timestamp failures are
    recorded as exclusions and are never silently replaced with another scene.
    """
    scenes, excluded = [], []
    tolerance = float(topology.get('assignment', {}).get('scene_timestamp_tolerance_s', .5))
    for marker in topology.get('scenes', []):
        pose = package.nearest_timestamp(float(marker['bag_timestamp']))
        index = next(i for i, item in enumerate(package.poses) if item['keyframe'] == pose['keyframe'])
        reason = None
        if marker['scene_type'] == 'OTHER':
            reason = 'OTHER is not an existing greenhouse benchmark scene category'
        elif abs(pose['timestamp']-float(marker['bag_timestamp'])) > tolerance:
            reason = 'no same-bag keyframe within timestamp tolerance'
        elif not (2 <= index < len(package.poses)-2):
            reason = 'keyframe does not support a full five-frame query'
        if reason:
            excluded.append(dict(scene_id=marker['scene_id'], reason=reason))
            continue
        scenes.append(dict(id=marker['scene_id'], type=marker['scene_type'].lower(), keyframe=index,
                           map_keyframe_id=pose['keyframe'], bag_timestamp=marker['bag_timestamp'],
                           note=marker.get('notes', 'manual scene marker')))
    return dict(schema_version=1,
                row_id_semantics='physical_row_ids_from_frozen_manual_topology_labels_only',
                topology_reference_status=REFERENCE_STATUS,
                topology_source_backend=topology['source']['backend_id'],
                query_backend=package.backend_id,
                status='READY' if scenes else 'NO_RUNNABLE_MANUAL_SCENES',
                scenes=scenes, rows=[], excluded_manual_scenes=excluded)


def save_topology(topology, path, package=None, other_package=None):
    """Save new artifact version and all sidecars; frozen files cannot change."""
    from .validator import validate_topology
    package = package or load_map_package(topology['source']['map_package'])
    validation = validate_topology(topology, package)
    if not validation['valid']:
        raise ValueError('; '.join(validation['errors']))
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    content = yaml.safe_dump(topology, sort_keys=False, allow_unicode=True)
    if output.exists():
        old = load_topology(output)
        if old.get('annotation', {}).get('status') == 'frozen' and output.read_text() != content:
            raise ValueError('Frozen annotation is immutable. Use a new draft filename/version.')
        if old.get('annotation', {}).get('status') == 'frozen':
            result = validate_topology(old, package, output, output.parent/'keyframe_topology_labels.csv')
            if not result['valid']:
                raise ValueError('Frozen annotation artifact validation failed: '+'; '.join(result['errors']))
            return json.loads(output.with_suffix('.manifest.json').read_text())
    # Required fixed-name CSV sidecars must not collide with another frozen
    # annotation version. Keep each version in its own output directory.
    for peer_manifest in output.parent.glob('*.manifest.json'):
        if peer_manifest == output.with_suffix('.manifest.json'):
            continue
        peer = json.loads(peer_manifest.read_text())
        if peer.get('annotation', {}).get('status') == 'frozen':
            raise ValueError('Use a new directory: another frozen annotation owns these sidecar filenames')
    for peer_yaml in output.parent.glob('*.yaml'):
        if peer_yaml == output:
            continue
        peer = yaml.safe_load(peer_yaml.read_text())
        if isinstance(peer, dict) and isinstance(peer.get('annotation'), dict) and peer['annotation'].get('status') == 'frozen':
            raise ValueError('Use a new directory: another frozen annotation owns these sidecar filenames')
    labels = labels_for_package(topology, package)
    other_labels = labels_for_package(topology, other_package) if other_package is not None else None
    output.write_text(content)
    output.with_suffix('.geojson').write_text(json.dumps(geojson(topology), indent=2, allow_nan=False))
    write_csv(output.parent/'keyframe_topology_labels.csv', labels)
    candidates = scene_candidates(topology, labels)
    write_csv(output.parent/'scene_candidates_for_manual_review.csv', candidates,
              ['keyframe', 'bag_timestamp', 'suggested_scene_type', 'physical_row_id', 'status', 'rule'])
    if other_package is not None:
        write_csv(output.parent/'other_backend_keyframe_topology_labels.csv', other_labels)
        correspondence = scene_correspondence(topology, package, other_package)
        if correspondence:
            write_csv(output.parent/'backend_scene_correspondence.csv', correspondence)
        else:
            write_csv(output.parent/'backend_scene_correspondence.csv', [],
                      ['scene_id', 'bag_timestamp', 'pointlio_keyframe', 'pointlio_dt', 'fastlivo2_keyframe', 'fastlivo2_dt'])
    package_dir = Path(__file__).resolve().parent
    tool_hash = hashlib.sha256(''.join(sha256_file(p) for p in sorted(package_dir.glob('*.py'))).encode()).hexdigest()
    sidecars = [output.with_suffix('.geojson'), output.parent/'keyframe_topology_labels.csv',
                output.parent/'scene_candidates_for_manual_review.csv']
    for backend_package in [package] + ([other_package] if other_package is not None else []):
        scene_path = output.parent/f'benchmark_scenes_{backend_package.backend_id}.yaml'
        scene_path.write_text(yaml.safe_dump(benchmark_scenes(topology, backend_package), sort_keys=False))
        sidecars.append(scene_path)
    if other_package is not None:
        sidecars += [output.parent/'other_backend_keyframe_topology_labels.csv', output.parent/'backend_scene_correspondence.csv']
    provenance = dict(schema_version=1, generated_at=now_iso(), annotation_file=str(output),
                      annotation_file_sha256=sha256_file(output), source=copy.deepcopy(topology['source']),
                      annotation=copy.deepcopy(topology['annotation']),
                      annotation_tool_source_sha256=tool_hash,
                      sidecar_hashes={p.name: sha256_file(p) for p in sidecars})
    output.with_suffix('.manifest.json').write_text(json.dumps(provenance, indent=2, allow_nan=False))
    # Version snapshots preserve previous hashes even when a draft is edited.
    archive = output.parent/'annotation_versions'/sha256_file(output)
    archive.mkdir(parents=True, exist_ok=True)
    (archive/output.name).write_text(content)
    (archive/output.with_suffix('.manifest.json').name).write_text(json.dumps(provenance, indent=2))
    return provenance
