"""Export a self-consistent FAST-LIO2-only reference from paired frontend data.

The node intentionally does not import or start PGO. It pairs body clouds with
their FAST-LIO2 frontend odometry, selects keyframes, and derives both the map
and the pose list from those exact pairs.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import shutil
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


REFERENCE_TYPE = 'FASTLIO2_SAME_SESSION_REFERENCE'


@dataclass(frozen=True)
class Keyframe:
    patch: str
    stamp_sec: int
    stamp_nanosec: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    @property
    def stamp(self) -> float:
        return self.stamp_sec + self.stamp_nanosec * 1e-9


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _pcd_header(points: int) -> bytes:
    return (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
        'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {points}\nDATA binary\n'
    ).encode('ascii')


def write_pcd(path: Path, points_xyzi: np.ndarray) -> None:
    points = np.asarray(points_xyzi, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 4 or len(points) == 0:
        raise ValueError(f'PCD requires nonempty Nx4 XYZI points: {path}')
    if not np.isfinite(points).all():
        raise ValueError(f'PCD contains a nonfinite point: {path}')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(_pcd_header(len(points)))
        np.ascontiguousarray(points).tofile(stream)


def read_pcd(path: Path) -> np.ndarray:
    path = Path(path)
    header: dict[str, list[str]] = {}
    with path.open('rb') as stream:
        for _ in range(64):
            raw = stream.readline(4096)
            if not raw:
                raise ValueError(f'incomplete PCD header: {path}')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            tokens = line.split()
            header[tokens[0]] = tokens[1:]
            if tokens[0] == 'DATA':
                if tokens[1:] != ['binary']:
                    raise ValueError(f'PCD is not binary: {path}')
                break
        else:
            raise ValueError(f'PCD DATA header missing: {path}')
        if header.get('FIELDS') != ['x', 'y', 'z', 'intensity']:
            raise ValueError(f'unsupported PCD fields: {path}')
        if header.get('SIZE') != ['4', '4', '4', '4'] or header.get('TYPE') != ['F', 'F', 'F', 'F']:
            raise ValueError(f'unsupported PCD scalar layout: {path}')
        if header.get('COUNT') != ['1', '1', '1', '1']:
            raise ValueError(f'unsupported PCD field count: {path}')
        count = int(header['POINTS'][0])
        if count < 1 or int(header['WIDTH'][0]) * int(header['HEIGHT'][0]) != count:
            raise ValueError(f'invalid PCD point count: {path}')
        payload = stream.read()
    if len(payload) != count * 16:
        raise ValueError(f'PCD payload size does not match POINTS: {path}')
    points = np.frombuffer(payload, dtype='<f4').reshape(count, 4)
    if not np.isfinite(points).all():
        raise ValueError(f'PCD contains nonfinite values: {path}')
    return points


def rotation_from_xyzw(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid zero/nonfinite quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype='f8')


def _pose_line(record: Keyframe, *, timed: bool) -> str:
    stamp = f'{record.stamp_sec}.{record.stamp_nanosec:09d} '
    qx, qy, qz, qw = record.quaternion_xyzw
    values = (*record.position, qw, qx, qy, qz)
    prefix = record.patch + ' ' + (stamp if timed else '')
    return prefix + ' '.join(f'{float(value):.12g}' for value in values) + '\n'


def _hashes(root: Path, *, exclude: set[str]) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): _sha256(path)
        for path in sorted(root.rglob('*'))
        if path.is_file() and path.relative_to(root).as_posix() not in exclude
    }


def write_reference_package(
    package_root: Path, source_patches: Path, records: list[Keyframe],
    frames: dict[str, str], *, source_bag: str,
) -> dict[str, int | str]:
    """Write the map and pose files from the same FAST-LIO2 cloud/pose pairs."""
    package_root = Path(package_root)
    source_patches = Path(source_patches)
    if package_root.exists():
        raise FileExistsError(f'refusing to overwrite reference package: {package_root}')
    if len(records) < 7:
        raise ValueError(f'FAST-LIO2 reference has too few keyframes: {len(records)}')
    package_root.mkdir(parents=True)
    patches = package_root / 'patches'
    patches.mkdir()
    world_chunks: list[np.ndarray] = []
    point_count = 0
    pose_lines: list[str] = []
    timed_lines: list[str] = []
    previous_stamp = -math.inf
    for record in records:
        if record.stamp <= previous_stamp:
            raise ValueError('FAST-LIO2 keyframe timestamps are not strictly increasing')
        previous_stamp = record.stamp
        source = source_patches / record.patch
        points = read_pcd(source)
        shutil.copy2(source, patches / record.patch)
        rotation = rotation_from_xyzw(record.quaternion_xyzw)
        position = np.asarray(record.position, dtype='f8')
        world = points.copy()
        world[:, :3] = (points[:, :3].astype('f8') @ rotation.T + position).astype('<f4')
        world_chunks.append(world)
        point_count += len(points)
        pose_lines.append(_pose_line(record, timed=False))
        timed_lines.append(_pose_line(record, timed=True))
    map_points = np.concatenate(world_chunks, axis=0)
    write_pcd(package_root / 'map.pcd', map_points)
    (package_root / 'poses.txt').write_text(''.join(pose_lines), encoding='ascii')
    (package_root / 'poses_timed.txt').write_text(''.join(timed_lines), encoding='ascii')
    (package_root / 'calibration.yaml').write_text(yaml.safe_dump({
        'format_version': 1, 'calibration_status': 'unavailable',
    }, sort_keys=True), encoding='utf-8')
    reference = {
        'source': 'fastlio2_odometry', 'pgo_applied': False, 'optimized': False,
        'absolute_ground_truth': False, 'same_session': True,
    }
    metadata = {
        'format_version': 1,
        'artifact_kind': 'fastlio2_same_session_reference',
        'reference_type': REFERENCE_TYPE,
        'reference': reference,
        'backend': 'FAST-LIO2',
        'pose_semantics': 'T_fastlio_map_body',
        'frames': {
            'map_frame': frames['odom_parent'],
            'cloud_frame': frames['cloud'],
            'body_frame': frames['odom_child'],
        },
        'source': {
            'rosbag': source_bag,
            'cloud_topic': '/mapping/frontend/cloud',
            'odometry_topic': '/mapping/frontend/odometry',
            'pgo_disabled': True,
        },
        'keyframe_count': len(records),
        'map_point_count': point_count,
        'outputs': {
            'map': 'map.pcd', 'poses': 'poses.txt', 'poses_timed': 'poses_timed.txt',
            'patches_dir': 'patches',
        },
    }
    (package_root / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
    payload_hashes = _hashes(package_root, exclude={'manifest.yaml', 'checksums.sha256'})
    manifest = {
        'format_version': 1, 'reference_type': REFERENCE_TYPE, 'reference': reference,
        'source_rosbag': source_bag, 'pgo_disabled': True,
        'files': payload_hashes,
    }
    (package_root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')
    indexed_hashes = _hashes(package_root, exclude={'checksums.sha256'})
    (package_root / 'checksums.sha256').write_text(
        ''.join(f'{digest}  {name}\n' for name, digest in indexed_hashes.items()), encoding='ascii')
    return verify_fastlio_reference_package(package_root)


def verify_fastlio_reference_package(package_root: Path) -> dict[str, int | str]:
    """Check provenance, byte checksums, poses, patches, and map transform consistency."""
    source_root = Path(package_root)
    if source_root.is_symlink():
        raise ValueError('FAST-LIO2 package root may not be a symlink')
    root = source_root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError('FAST-LIO2 package root is not a regular directory')
    required = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'patches', 'metadata.yaml',
                'manifest.yaml', 'checksums.sha256')
    if any(not (root / name).exists() for name in required):
        raise ValueError('FAST-LIO2 package is missing a required file/directory')
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text(encoding='utf-8'))
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
    if metadata.get('reference_type') != REFERENCE_TYPE or manifest.get('reference_type') != REFERENCE_TYPE:
        raise ValueError('package is not labelled FASTLIO2_SAME_SESSION_REFERENCE')
    reference = metadata.get('reference', {})
    if (reference.get('source') != 'fastlio2_odometry' or reference.get('pgo_applied') is not False
            or reference.get('optimized') is not False or reference.get('absolute_ground_truth') is not False
            or reference.get('same_session') is not True or manifest.get('pgo_disabled') is not True
            or manifest.get('reference') != reference):
        raise ValueError('FAST-LIO2 reference provenance flags are inconsistent')
    indexed: dict[str, str] = {}
    for line in (root / 'checksums.sha256').read_text(encoding='ascii').splitlines():
        fields = line.split('  ', 1)
        if len(fields) != 2 or len(fields[0]) != 64:
            raise ValueError(f'malformed checksum entry: {line!r}')
        digest, relative = fields
        path = Path(relative)
        if path.is_absolute() or '..' in path.parts or relative in indexed:
            raise ValueError(f'unsafe or duplicate checksum path: {relative}')
        file = root / path
        if not file.is_file() or file.is_symlink() or _sha256(file) != digest:
            raise ValueError(f'checksum mismatch or missing file: {relative}')
        indexed[relative] = digest
    actual = set(_hashes(root, exclude={'checksums.sha256'}))
    if set(indexed) != actual:
        raise ValueError('checksum index does not cover every package file exactly once')
    if set(manifest.get('files', {})) != actual - {'manifest.yaml'}:
        raise ValueError('manifest file list does not cover all payload files')
    for relative, digest in manifest.get('files', {}).items():
        if indexed.get(relative) != digest:
            raise ValueError(f'manifest file checksum mismatch: {relative}')
    if not {'map.pcd', 'poses_timed.txt', 'metadata.yaml', 'manifest.yaml'}.issubset(indexed):
        raise ValueError('checksum index omits required reference files')

    pose_lines = (root / 'poses_timed.txt').read_text(encoding='ascii').splitlines()
    legacy_lines = (root / 'poses.txt').read_text(encoding='ascii').splitlines()
    if len(pose_lines) < 7 or len(pose_lines) != int(metadata.get('keyframe_count', -1)):
        raise ValueError('pose/keyframe count is invalid')
    if len(legacy_lines) != len(pose_lines):
        raise ValueError('poses.txt and poses_timed.txt record counts differ')
    patch_dir = root / 'patches'
    if patch_dir.is_symlink() or not patch_dir.is_dir():
        raise ValueError('patches must be a regular directory')
    poses: list[tuple[str, float, np.ndarray, np.ndarray]] = []
    previous_stamp = -math.inf
    point_count = 0
    map_points = read_pcd(root / 'map.pcd')
    cursor = 0
    for index, line in enumerate(pose_lines):
        tokens = line.split()
        if len(tokens) != 9:
            raise ValueError(f'invalid poses_timed record {index}')
        legacy_tokens = legacy_lines[index].split()
        if len(legacy_tokens) != 8 or legacy_tokens[0] != tokens[0]:
            raise ValueError(f'poses.txt does not match timed pose record {index}')
        patch = tokens[0]
        if Path(patch).name != patch or not patch.endswith('.pcd'):
            raise ValueError(f'unsafe patch reference: {patch}')
        values = np.asarray([float(value) for value in tokens[1:]], dtype='f8')
        if not np.isfinite(values).all():
            raise ValueError(f'nonfinite pose record {index}')
        stamp = float(values[0])
        if stamp <= previous_stamp:
            raise ValueError('pose timestamps are not strictly increasing')
        previous_stamp = stamp
        position = values[1:4]
        qw, qx, qy, qz = values[4:8]
        legacy_values = np.asarray([float(value) for value in legacy_tokens[1:]], dtype='f8')
        if not np.allclose(legacy_values, values[1:], rtol=0.0, atol=1e-10):
            raise ValueError(f'poses.txt values differ from timed pose record {index}')
        quat = np.array([qx, qy, qz, qw], dtype='f8')
        qnorm = float(np.linalg.norm(quat))
        if not math.isfinite(qnorm) or abs(qnorm - 1.0) > 1e-3:
            raise ValueError(f'invalid quaternion norm at keyframe {index}: {qnorm}')
        patch_path = root / 'patches' / patch
        if not patch_path.is_file() or patch_path.is_symlink():
            raise ValueError(f'missing/unsafe patch: {patch_path}')
        body_points = read_pcd(patch_path)
        transform = rotation_from_xyzw(quat)
        expected = body_points[:, :3].astype('f8') @ transform.T + position
        expected = expected.astype('<f4')
        next_cursor = cursor + len(body_points)
        if next_cursor > len(map_points):
            raise ValueError('map has fewer points than the transformed patches')
        if not np.array_equal(map_points[cursor:next_cursor, :3], expected):
            raise ValueError(f'map/patch/pose geometry mismatch at keyframe {index}')
        if not np.array_equal(map_points[cursor:next_cursor, 3], body_points[:, 3]):
            raise ValueError(f'map/patch intensity mismatch at keyframe {index}')
        cursor = next_cursor
        point_count += len(body_points)
        poses.append((patch, stamp, position, quat))
    if cursor != len(map_points) or point_count != int(metadata.get('map_point_count', -1)):
        raise ValueError('map point count does not match the transformed patches')
    listed = {path.name for path in (root / 'patches').iterdir() if path.is_file()}
    if listed != {pose[0] for pose in poses}:
        raise ValueError('patch directory does not exactly match pose references')
    return {
        'status': 'PASS', 'reference_type': REFERENCE_TYPE,
        'keyframes': len(poses), 'map_points': point_count,
        'map_frame': str(metadata['frames']['map_frame']),
    }


def main(args=None) -> int:
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class FastlioReferenceExporter(Node):
        def __init__(self):
            super().__init__('fastlio_reference_exporter')
            self.output_dir = Path(str(self.declare_parameter('output_dir', '').value)).resolve()
            self.bag_path = str(self.declare_parameter('bag_path', '').value)
            self.distance_threshold = float(self.declare_parameter('keyframe_distance_m', 0.5).value)
            self.rotation_threshold_deg = float(self.declare_parameter('keyframe_rotation_deg', 10.0).value)
            if not str(self.output_dir) or not self.bag_path:
                raise ValueError('output_dir and bag_path parameters are required')
            self.source_dir = self.output_dir / '.fastlio_reference_source'
            self.source_patches = self.source_dir / 'patches'
            if self.source_dir.exists():
                raise FileExistsError(f'FAST-LIO staging directory already exists: {self.source_dir}')
            self.source_patches.mkdir(parents=True)
            self.package_root = self.output_dir / 'fastlio_reference_package'
            self.records: list[Keyframe] = []
            self.last_keyframe_position: np.ndarray | None = None
            self.last_keyframe_quaternion: np.ndarray | None = None
            self.last_stamp = -math.inf
            self.cloud_frame = ''
            self.odom_parent = ''
            self.odom_child = ''
            self.fatal_error = ''
            self.status_pub = self.create_publisher(String, '/mapping/backend/status', 10)
            self.export_srv = self.create_service(Trigger, '/mapping/backend/export_artifact', self.export)
            cloud_sub = Subscriber(self, PointCloud2, '/mapping/frontend/cloud', qos_profile=10)
            odom_sub = Subscriber(self, Odometry, '/mapping/frontend/odometry', qos_profile=10)
            self.sync = ApproximateTimeSynchronizer([cloud_sub, odom_sub], queue_size=10, slop=0.1)
            self.sync.registerCallback(self.on_pair)
            self.publish_status('fastlio_reference_ready_pgo_disabled')
            self.get_logger().info('FAST-LIO2-only reference exporter ready; PGO is not launched')

        def publish_status(self, value: str):
            message = String()
            message.data = value
            self.status_pub.publish(message)

        def on_pair(self, cloud, odometry):
            if self.fatal_error or self.package_root.exists():
                return
            try:
                stamp_sec = int(cloud.header.stamp.sec)
                stamp_nanosec = int(cloud.header.stamp.nanosec)
                stamp = stamp_sec + stamp_nanosec * 1e-9
                if stamp <= self.last_stamp:
                    return
                self.last_stamp = stamp
                cloud_frame = str(cloud.header.frame_id)
                odom_parent = str(odometry.header.frame_id)
                odom_child = str(odometry.child_frame_id)
                if not self.cloud_frame:
                    self.cloud_frame, self.odom_parent, self.odom_child = cloud_frame, odom_parent, odom_child
                elif (self.cloud_frame, self.odom_parent, self.odom_child) != (cloud_frame, odom_parent, odom_child):
                    raise ValueError('frontend cloud/odometry frame IDs changed during replay')
                pose = odometry.pose.pose
                position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype='f8')
                quaternion = np.array([
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                ], dtype='f8')
                if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
                    raise ValueError('frontend odometry contains nonfinite pose values')
                qnorm = float(np.linalg.norm(quaternion))
                if qnorm < 1e-9 or abs(qnorm - 1.0) > 1e-3:
                    raise ValueError(f'frontend quaternion norm is invalid: {qnorm}')
                quaternion /= qnorm
                take = self.last_keyframe_position is None
                if not take:
                    distance = float(np.linalg.norm(position - self.last_keyframe_position))
                    dot = min(1.0, abs(float(np.dot(quaternion, self.last_keyframe_quaternion))))
                    angle_deg = math.degrees(2.0 * math.acos(dot))
                    take = distance > self.distance_threshold or angle_deg > self.rotation_threshold_deg
                if not take:
                    return
                fields = {field.name for field in cloud.fields}
                if not {'x', 'y', 'z'}.issubset(fields):
                    raise ValueError('frontend body cloud lacks x/y/z fields')
                selected_fields = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in fields else [])
                raw = point_cloud2.read_points(cloud, field_names=selected_fields, skip_nans=True)
                if len(raw) == 0:
                    raise ValueError('FAST-LIO2 selected an empty keyframe cloud')
                xyz = np.column_stack([raw[name] for name in ('x', 'y', 'z')]).astype('<f4')
                intensity = (np.asarray(raw['intensity'], dtype='<f4') if 'intensity' in selected_fields
                             else np.zeros(len(xyz), dtype='<f4'))
                finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
                points = np.column_stack((xyz[finite], intensity[finite])).astype('<f4')
                if len(points) == 0:
                    raise ValueError('FAST-LIO2 selected keyframe has no finite points')
                patch = f'{len(self.records)}.pcd'
                write_pcd(self.source_patches / patch, points)
                self.records.append(Keyframe(
                    patch, stamp_sec, stamp_nanosec, tuple(float(v) for v in position),
                    tuple(float(v) for v in quaternion),
                ))
                self.last_keyframe_position = position.copy()
                self.last_keyframe_quaternion = quaternion.copy()
                if len(self.records) % 100 == 0:
                    self.get_logger().info(f'FAST-LIO2 reference keyframes captured: {len(self.records)}')
            except Exception as exc:
                self.fatal_error = str(exc)
                self.publish_status('fastlio_reference_capture_failed')
                self.get_logger().error(f'FAST-LIO2 reference capture failed: {exc}')

        def export(self, _request, response):
            if self.fatal_error:
                response.success = False
                response.message = self.fatal_error
                return response
            if len(self.records) < 7:
                response.success = False
                response.message = f'need at least seven FAST-LIO2 keyframes; captured {len(self.records)}'
                self.publish_status('fastlio_reference_export_no_poses')
                return response
            try:
                stats = write_reference_package(
                    self.package_root, self.source_patches, self.records,
                    {'cloud': self.cloud_frame, 'odom_parent': self.odom_parent,
                     'odom_child': self.odom_child}, source_bag=self.bag_path,
                )
                response.success = True
                response.message = str(self.package_root)
                self.publish_status('fastlio_reference_exported_pgo_disabled')
                self.get_logger().info(
                    f"FAST-LIO2 reference verified: {stats['keyframes']} keyframes, "
                    f"{stats['map_points']} map points"
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.publish_status('fastlio_reference_export_failed')
                self.get_logger().error(f'FAST-LIO2 reference export failed: {exc}')
            return response

    rclpy.init(args=args)
    node = FastlioReferenceExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
