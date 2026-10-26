"""Synthetic native inputs for policy tests; never evidence of a working LIO."""
from pathlib import Path

import yaml

from agt_mapping_bringup.backend_registry import resolve_backend
from agt_mapping_bringup.backend_snapshot import digest


def backend_fixture(root):
    root = Path(root)
    source = root / 'synthetic_source'
    source.mkdir(exist_ok=True)
    executable = source / 'synthetic_livo'
    executable.write_text('#!/bin/sh\nexit 0\n')
    executable.chmod(0o755)
    config = root / 'mounted_native.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'common': {'img_en': 0, 'lidar_en': 1, 'lid_topic': '/original/lidar', 'imu_topic': '/original/imu'},
        'imu': {'imu_en': True},
        'extrin_calib': {'extrinsic_T': [0.011, 0.02329, -0.04412],
                         'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
    }}}))
    profile = resolve_backend(check_sources=False)
    backend = profile['backend']
    for key in ('containing_repository_commit', 'source_tree_path', 'source_tree_object',
                'profile_origin_path', 'profile_origin_sha256'):
        backend.pop(key, None)
    backend.update(source_path=str(source), source_commit='synthetic-fixture-not-a-native-build',
                   config_path=str(config), config_sha256=digest(config.read_bytes()),
                   required_executables=[str(executable)],
                   source_files_sha256={str(executable): digest(executable.read_bytes())})
    destination = root / 'fast_livo2_lio.yaml'
    destination.write_text(yaml.safe_dump(profile))
    return destination
