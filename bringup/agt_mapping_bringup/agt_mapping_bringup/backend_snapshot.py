"""Freeze the effective FAST-LIVO2 inputs once, before any ROS process starts.

This is session provenance inside the existing source-package calibration
payload, not another asset schema. Native translation is metres, rotation is
row-major T_imu_lidar; the supported adapter body is the IMU, not a chassis.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np
import yaml

from .backend_registry import BackendSelectionError, validate_profile


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _vector(value, size, name):
    if not isinstance(value, list) or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise BackendSelectionError(f'{name} must be a numeric YAML array')
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise BackendSelectionError(f'{name} must be a finite numeric vector') from exc
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise BackendSelectionError(f'{name} must have {size} finite values')
    return vector


def validate_native_config(data: bytes, profile: dict) -> dict:
    """Check the actual bytes consumed by the native node and adapter agree."""
    try:
        document = yaml.safe_load(data)
        if not isinstance(document, dict) or set(document) != {'/**'}:
            raise ValueError('only the /** ROS parameter scope is supported; node-specific overrides are ambiguous')
        parameters = document['/**']['ros__parameters']
        common = parameters['common']
        imu = parameters['imu']
        extrinsic = parameters['extrin_calib']
        if not all(isinstance(v, dict) for v in (parameters, common, imu, extrinsic)):
            raise ValueError('parameter groups must be mappings')
        if any('.' in key for key in parameters):
            raise ValueError('flat dotted parameter overrides are ambiguous; use nested groups')
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise BackendSelectionError(f'cannot inspect FAST-LIVO2 LIO-only config: {exc}') from exc
    if (type(common.get('img_en')) is not int or common['img_en'] != 0
            or type(common.get('lidar_en')) is not int or common['lidar_en'] != 1
            or imu.get('imu_en') is not True):
        raise BackendSelectionError('FAST-LIVO2 requires common.img_en=0, common.lidar_en=1 and imu.imu_en=true')
    sensor = profile['sensor']
    expected_outputs = {
        'output_odom_topic': '/aft_mapped_to_init', 'output_cloud_topic': '/cloud_registered_lidar',
        'output_path_topic': '/path', 'output_cloud_frame_mode': 'lidar',
    }
    if any(profile['backend'].get(key) != value for key, value in expected_outputs.items()):
        raise BackendSelectionError('FAST-LIVO2 profile outputs must match the native lidar-cloud/odometry/path contract')
    if profile['mapping']['map_frame'] != 'camera_init':
        raise BackendSelectionError('FAST-LIVO2 native map frame is camera_init; frame remapping is not implemented')
    if sensor['body_frame'] != sensor['imu_frame']:
        raise BackendSelectionError('FAST-LIVO2 body_frame must be the IMU frame; a chassis transform is not inferred')
    if sensor.get('imu_acceleration_unit') != 'g':
        raise BackendSelectionError('this FAST-LIVO2 input contract requires native IMU acceleration in g')
    input_format = profile['backend']['input_format']
    if not isinstance(input_format, dict) or input_format.get('imu_acceleration_unit') != 'g_native_benchmark_input':
        raise BackendSelectionError('FAST-LIVO2 profile must retain the verified native IMU acceleration unit')
    translation = _vector(extrinsic.get('extrinsic_T'), 3, 'extrin_calib.extrinsic_T (metres)')
    rotation = _vector(extrinsic.get('extrinsic_R'), 9, 'extrin_calib.extrinsic_R').reshape(3, 3)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0):
        raise BackendSelectionError('native extrinsic_R must be a proper rotation matrix')
    expected_t = _vector(sensor.get('T_body_lidar_translation_m'), 3, 'T_body_lidar_translation_m')
    q = _vector(sensor.get('T_body_lidar_quaternion_xyzw'), 4, 'T_body_lidar_quaternion_xyzw')
    if not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6, rtol=0):
        raise BackendSelectionError('T_body_lidar quaternion must have unit norm')
    x, y, z, w = q
    expected_r = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    if not np.allclose(translation, expected_t, atol=1e-9, rtol=0) or not np.allclose(
            rotation, expected_r, atol=1e-6, rtol=0):
        raise BackendSelectionError('native extrinsic_T/R disagree with profile T_body_lidar (metres, lidar-to-IMU)')
    return document


@dataclass(frozen=True)
class BackendSnapshot:
    profile_path: Path
    config_path: Path
    profile_bytes: bytes
    config_bytes: bytes

    @property
    def profile(self):
        return yaml.safe_load(self.profile_bytes)

    @property
    def profile_sha256(self):
        return digest(self.profile_bytes)

    def write(self):
        self.profile_path.parent.mkdir(exist_ok=False)
        for path, data in ((self.config_path, self.config_bytes), (self.profile_path, self.profile_bytes)):
            with path.open('xb') as stream:
                stream.write(data)
            path.chmod(0o444)

    def verify_files(self):
        for path, expected in ((self.profile_path, self.profile_bytes), (self.config_path, self.config_bytes)):
            if path.is_symlink() or path.read_bytes() != expected:
                raise BackendSelectionError(f'startup backend snapshot changed: {path}')

    def calibration_provenance(self):
        self.verify_files()
        sensor = self.profile['sensor']
        return {
            'status': 'explicit profile extrinsic; no measurement accuracy inferred',
            'T_body_lidar_translation_m': sensor['T_body_lidar_translation_m'],
            'T_body_lidar_quaternion_xyzw': sensor['T_body_lidar_quaternion_xyzw'],
            'native_config_snapshot': {'sha256': digest(self.config_bytes),
                                       'yaml': self.config_bytes.decode('utf-8')},
            'backend_profile_snapshot': {'sha256': self.profile_sha256,
                                         'yaml': self.profile_bytes.decode('utf-8')},
        }


def prepare_backend_snapshot(profile, output, *, lidar_topic, imu_topic, use_sim_time):
    """Prepare in memory; write only after the session output has been reserved."""
    profile = deepcopy(profile)
    validate_profile(profile, 'fast_livo2_lio')
    backend, sensor = profile['backend'], profile['sensor']
    source_path = Path(backend['config_path']).expanduser().resolve()
    original = source_path.read_bytes()
    if digest(original) != backend.get('config_sha256'):
        raise BackendSelectionError(f'backend config changed before startup: {source_path}')
    document = validate_native_config(original, profile)
    parameters = document['/**']['ros__parameters']
    parameters['common']['lid_topic'] = lidar_topic
    parameters['common']['imu_topic'] = imu_topic
    parameters['use_sim_time'] = bool(use_sim_time)
    # ROS parameter arrays and the adapter's scalar extrinsics are doubles;
    # normalize numeric YAML spelling without changing the transform.
    for key in ('extrinsic_T', 'extrinsic_R'):
        parameters['extrin_calib'][key] = [float(value) for value in parameters['extrin_calib'][key]]
    for key in ('T_body_lidar_translation_m', 'T_body_lidar_quaternion_xyzw'):
        sensor[key] = [float(value) for value in sensor[key]]
    config_bytes = yaml.safe_dump(document, sort_keys=True).encode('utf-8')
    destination = Path(output).resolve() / 'backend_snapshot'
    config_path, profile_path = destination / 'native.yaml', destination / 'profile.yaml'
    backend.update(config_origin_path=str(source_path), config_origin_sha256=digest(original),
                   config_path=str(config_path), config_sha256=digest(config_bytes), startup_config_frozen=True)
    sensor.update(lidar_topic=lidar_topic, imu_topic=imu_topic, backend_imu_topic=imu_topic)
    profile_bytes = yaml.safe_dump(profile, sort_keys=True).encode('utf-8')
    return BackendSnapshot(profile_path, config_path, profile_bytes, config_bytes)


def load_backend_snapshot(profile_path, expected_profile_sha256):
    """Exporter requires the launch-pinned digest; never reload the mounted input."""
    profile_path = Path(profile_path).resolve()
    profile_bytes = profile_path.read_bytes()
    if not expected_profile_sha256 or digest(profile_bytes) != expected_profile_sha256:
        raise BackendSelectionError('startup profile snapshot hash mismatch')
    profile = yaml.safe_load(profile_bytes)
    validate_profile(profile, 'fast_livo2_lio')
    backend = profile['backend']
    config_path = Path(backend['config_path'])
    if backend.get('startup_config_frozen') is not True or config_path != profile_path.parent / 'native.yaml':
        raise BackendSelectionError('export requires the startup-frozen native config beside its profile')
    config_bytes = config_path.read_bytes()
    if digest(config_bytes) != backend['config_sha256']:
        raise BackendSelectionError('startup native config snapshot hash mismatch')
    document = validate_native_config(config_bytes, profile)
    common = document['/**']['ros__parameters']['common']
    if (common['lid_topic'], common['imu_topic']) != (profile['sensor']['lidar_topic'], profile['sensor']['imu_topic']):
        raise BackendSelectionError('snapshot native input topics disagree with profile')
    snapshot = BackendSnapshot(profile_path, config_path, profile_bytes, config_bytes)
    snapshot.verify_files()
    return snapshot
