"""Execute mounted-profile/snapshot/export contracts without a ROS runtime."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_mapping_artifacts.frontend_package import write_frontend_map_package
from agt_mapping_bringup.backend_registry import BackendSelectionError, resolve_backend
from agt_mapping_bringup.backend_snapshot import (
    digest, load_backend_snapshot, prepare_backend_snapshot, validate_native_config,
)
from agt_mapping_bringup.cli import main, parser, _live_plan
from agt_mapping_bringup.frontend_map_exporter import build_frontend_provenance
from backend_fixture import backend_fixture


def prepared(tmp_path):
    path = backend_fixture(tmp_path)
    profile = resolve_backend(profile_path=path)
    snapshot = prepare_backend_snapshot(profile, tmp_path, lidar_topic='/capture/lidar',
                                        imu_topic='/capture/imu', use_sim_time=True)
    snapshot.write()
    return path, profile, snapshot


def test_mounted_profile_validates_source_and_freezes_effective_parameters(tmp_path):
    path, original, snapshot = prepared(tmp_path)
    loaded = load_backend_snapshot(snapshot.profile_path, snapshot.profile_sha256)
    assert loaded.profile == snapshot.profile
    native = yaml.safe_load(loaded.config_bytes)['/**']['ros__parameters']
    assert native['common']['lid_topic'] == '/capture/lidar'
    assert native['common']['imu_topic'] == '/capture/imu'
    assert native['common']['img_en'] == 0
    assert native['imu']['imu_en'] is True
    assert native['use_sim_time'] is True
    assert snapshot.profile['backend']['config_sha256'] == digest(snapshot.config_bytes)
    assert snapshot.profile['backend']['config_origin_sha256'] == original['backend']['config_sha256']
    assert snapshot.profile['backend']['profile_origin_sha256'] == digest(path.read_bytes())
    assert original['backend']['config_path'] != snapshot.profile['backend']['config_path']


def test_default_registry_uses_same_snapshot_contract(tmp_path):
    path = backend_fixture(tmp_path)
    # Default backend selection is real; only its installed profile directory is a fixture.
    profile = resolve_backend(profile_dir=path.parent)
    snapshot = prepare_backend_snapshot(profile, tmp_path, lidar_topic='/live/lidar',
                                        imu_topic='/live/imu', use_sim_time=False)
    snapshot.write()
    assert snapshot.profile['backend']['id'] == 'fast_livo2_lio'
    assert yaml.safe_load(snapshot.config_bytes)['/**']['ros__parameters']['use_sim_time'] is False


def test_export_archives_actual_startup_bytes_after_mount_is_changed(tmp_path):
    path, original, snapshot = prepared(tmp_path)
    Path(original['backend']['config_path']).write_text('changed: after_start\n')
    path.write_text('changed: profile_after_start\n')
    loaded = load_backend_snapshot(snapshot.profile_path, snapshot.profile_sha256)
    provenance = build_frontend_provenance(loaded, bag_path='fixture_bag', lidar_topic='/capture/lidar',
                                           imu_topic='/capture/imu', map_frame='camera_init', body_frame='livox_imu')
    package = tmp_path / 'map_package'
    result = write_frontend_map_package(package, [{
        'stamp_sec': index + 1, 'stamp_nanosec': 1, 'position': np.array([float(index), 0., 0.]),
        'quaternion_xyzw': np.array([0., 0., 0., 1.]),
        'points_xyzi': np.array([[1., 2., 3., 4.]], dtype='<f4'),
    } for index in range(7)], provenance)
    assert result['backend_id'] == 'fast_livo2_lio'
    calibration = yaml.safe_load((package / 'calibration.yaml').read_text())
    for key, data in [('native_config_snapshot', snapshot.config_bytes),
                      ('backend_profile_snapshot', snapshot.profile_bytes)]:
        assert calibration[key]['yaml'].encode() == data
        assert calibration[key]['sha256'] == digest(data)
    assert provenance['reference']['pgo_applied'] is False
    assert provenance['reference']['optimized'] is False


@pytest.mark.parametrize('which', ['config', 'profile'])
def test_changed_snapshot_fails_export_and_startup_validation(tmp_path, which):
    _, _, snapshot = prepared(tmp_path)
    path = snapshot.config_path if which == 'config' else snapshot.profile_path
    path.chmod(0o644)
    path.write_bytes(path.read_bytes() + b'\n# changed\n')
    with pytest.raises(BackendSelectionError, match='snapshot changed'):
        snapshot.calibration_provenance()
    with pytest.raises(BackendSelectionError, match='hash mismatch'):
        load_backend_snapshot(snapshot.profile_path, snapshot.profile_sha256)


def test_config_changed_between_resolution_and_startup_is_rejected(tmp_path):
    path = backend_fixture(tmp_path)
    profile = resolve_backend(profile_path=path)
    Path(profile['backend']['config_path']).write_text('changed: before_start\n')
    with pytest.raises(BackendSelectionError, match='changed before startup'):
        prepare_backend_snapshot(profile, tmp_path, lidar_topic='/l', imu_topic='/i', use_sim_time=True)


@pytest.mark.parametrize('problem', ['visual', 'imu_disabled', 'translation', 'rotation', 'body_frame',
                                     'imu_units', 'quaternion', 'node_override', 'flat_override', 'legacy_config'])
def test_invalid_native_or_frame_contract_fails(tmp_path, problem):
    path = backend_fixture(tmp_path)
    profile = resolve_backend(profile_path=path)
    doc = yaml.safe_load(Path(profile['backend']['config_path']).read_bytes())
    params = doc['/**']['ros__parameters']
    if problem == 'visual': params['common']['img_en'] = 1
    elif problem == 'imu_disabled': params['imu']['imu_en'] = False
    elif problem == 'translation': params['extrin_calib']['extrinsic_T'][0] = 123.0
    elif problem == 'rotation': params['extrin_calib']['extrinsic_R'][0] = 2.0
    elif problem == 'body_frame': profile['sensor']['body_frame'] = 'mkmini_rear_axle'
    elif problem == 'imu_units': profile['sensor']['imu_acceleration_unit'] = 'm/s2'
    elif problem == 'quaternion': profile['sensor']['T_body_lidar_quaternion_xyzw'] = [0., 0., 0., 2.]
    elif problem == 'node_override': doc['fastlivo_mapping'] = {'ros__parameters': {'common.img_en': 1}}
    elif problem == 'flat_override': params['common.img_en'] = 1
    elif problem == 'legacy_config': doc = {'t_il': [0., 0., 0.], 'r_il': [1.]*9}
    with pytest.raises(BackendSelectionError):
        validate_native_config(yaml.safe_dump(doc).encode(), profile)


@pytest.mark.parametrize('problem', ['config_hash', 'source_hash', 'binary', 'profile_id'])
def test_mounted_profile_never_bypasses_registry_validation(tmp_path, problem):
    path = backend_fixture(tmp_path)
    profile = yaml.safe_load(path.read_text())
    backend = profile['backend']
    if problem == 'config_hash': backend['config_sha256'] = '0' * 64
    elif problem == 'source_hash': backend['source_files_sha256'][backend['required_executables'][0]] = '0' * 64
    elif problem == 'binary': backend['required_executables'] = [str(tmp_path / 'missing')]
    elif problem == 'profile_id': backend['id'] = 'fast_lio2_legacy'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(BackendSelectionError):
        resolve_backend(profile_path=path)


def test_export_mismatch_is_not_silently_relabelled(tmp_path):
    _, _, snapshot = prepared(tmp_path)
    with pytest.raises(ValueError, match='disagree'):
        build_frontend_provenance(snapshot, bag_path='bag', lidar_topic='/wrong/lidar',
                                  imu_topic='/capture/imu', map_frame='camera_init', body_frame='livox_imu')


def test_old_fastlio_cli_is_explicitly_rejected(capsys):
    assert main(['--live', '--fastlio-config', '/mounted/old.yaml']) == 2
    assert '--backend-profile' in capsys.readouterr().err
    assert parser().parse_args(['--backend-profile', '/mounted/profile.yaml']).backend_profile == '/mounted/profile.yaml'


def test_nonidentity_calibration_uses_lidar_to_imu_direction(tmp_path):
    path = backend_fixture(tmp_path)
    profile = resolve_backend(profile_path=path)
    doc = yaml.safe_load(Path(profile['backend']['config_path']).read_bytes())
    rotation = [0., -1., 0., 1., 0., 0., 0., 0., 1.]
    doc['/**']['ros__parameters']['extrin_calib']['extrinsic_R'] = rotation
    profile['sensor']['T_body_lidar_quaternion_xyzw'] = [0., 0., 2**-0.5, 2**-0.5]
    validate_native_config(yaml.safe_dump(doc).encode(), profile)
    profile['sensor']['T_body_lidar_quaternion_xyzw'][2] *= -1
    with pytest.raises(BackendSelectionError, match='disagree'):
        validate_native_config(yaml.safe_dump(doc).encode(), profile)


def test_relative_mounted_dependencies_are_anchored_to_profile(tmp_path, monkeypatch):
    path = backend_fixture(tmp_path)
    document = yaml.safe_load(path.read_bytes())
    backend = document['backend']
    for key in ('source_path', 'config_path'):
        backend[key] = str(Path(backend[key]).relative_to(tmp_path))
    backend['required_executables'] = [str(Path(item).relative_to(tmp_path)) for item in backend['required_executables']]
    backend['source_files_sha256'] = {str(Path(item).relative_to(tmp_path)): value
                                     for item, value in backend['source_files_sha256'].items()}
    path.write_text(yaml.safe_dump(document))
    monkeypatch.chdir('/')
    profile = resolve_backend(profile_path=path)
    assert Path(profile['backend']['config_path']) == tmp_path / 'mounted_native.yaml'


def test_default_dryrun_setup_path_can_be_described_without_machine_checkout():
    from agt_mapping_bringup.cli import _backend_setup
    profile = {'backend': {'source_path': '/not-installed/fixture/third_party/livo',
                           'required_executables': ['/not-installed/fixture/install/fast_livo/lib/fast_livo/fastlivo_mapping']}}
    assert _backend_setup(profile) == Path('/not-installed/fixture/install/setup.bash')
