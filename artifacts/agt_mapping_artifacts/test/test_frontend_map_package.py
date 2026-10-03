from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_mapping_artifacts.frontend_package import (
    verify_frontend_map_package,
    write_frontend_map_package,
)


def _provenance(backend_id='lio_sam_noloop'):
    return {
        'mapping_backend': {
            'id': backend_id,
            'project': 'test estimator',
            'mode': 'no_loop' if backend_id == 'lio_sam_noloop' else 'lio_only',
            'source_commit': 'a' * 40,
            'config_sha256': 'b' * 64,
            'loop_closure': False,
            'gps_factor': False,
            'external_global_correction': False,
        },
        'source': {'rosbag': '/tmp/read_only_bag', 'lidar_topic': '/input/lidar', 'imu_topic': '/input/imu'},
        'frames': {'map': 'odom', 'body': 'body', 'lidar': 'lidar', 'imu': 'imu'},
        'reference': {
            'same_session': True, 'absolute_ground_truth': False,
            'source': 'mapping_frontend_odometry', 'pgo_applied': False, 'optimized': False,
        },
    }


def _records():
    cloud = np.asarray([[1.0, 2.0, 3.0, 10.0], [2.0, 1.0, 0.0, 20.0]], dtype='<f4')
    records = []
    for index in range(7):
        records.append({
            'stamp_sec': 10 + index,
            'stamp_nanosec': 0,
            'position': np.asarray([float(index), 0.0, 0.0]),
            'quaternion_xyzw': np.asarray([0.0, 0.0, 0.0, 1.0]),
            'points_xyzi': cloud.copy(),
        })
    return records


@pytest.mark.parametrize('backend_id', ['lio_sam_noloop', 'point_lio', 'fast_livo2_lio'])
def test_frontend_package_is_backend_independent(tmp_path, backend_id):
    root = tmp_path / 'map_package'
    result = write_frontend_map_package(root, _records(), _provenance(backend_id))
    assert result['status'] == 'PASS'
    assert result['backend_id'] == backend_id
    assert result['keyframes'] == 7
    assert result['map_points'] == 14
    assert verify_frontend_map_package(root) == result
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    assert metadata['reference']['absolute_ground_truth'] is False
    assert metadata['mapping_backend']['id'] == backend_id
    assert (root / 'patches' / '6.pcd').is_file()


def test_frontend_package_rejects_unsupported_backend(tmp_path):
    with pytest.raises(ValueError, match='unsupported'):
        write_frontend_map_package(tmp_path / 'map_package', _records(), _provenance('other_lio'))


def test_frontend_package_rejects_no_loop_profile_with_loop_or_gps(tmp_path):
    provenance = _provenance()
    provenance['mapping_backend']['loop_closure'] = True
    with pytest.raises(ValueError, match='no-loop'):
        write_frontend_map_package(tmp_path / 'map_package', _records(), provenance)


def test_frontend_package_requires_same_source_poses_and_clouds(tmp_path):
    records = _records()
    records[4]['stamp_sec'] = records[3]['stamp_sec']
    with pytest.raises(ValueError, match='strictly increasing'):
        write_frontend_map_package(tmp_path / 'map_package', records, _provenance())


def test_frontend_package_refuses_absolute_ground_truth_claim(tmp_path):
    provenance = _provenance()
    provenance['reference']['absolute_ground_truth'] = True
    with pytest.raises(ValueError, match='absolute ground truth'):
        write_frontend_map_package(tmp_path / 'map_package', _records(), provenance)
