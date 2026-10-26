from pathlib import Path

import pytest

from agt_mapping_bringup.backend_registry import (
    BACKEND_IDS,
    BackendSelectionError,
    PROFILE_DIR,
    SELECTION_PATH,
    _verify_local_provenance,
    resolve_backend,
    validate_profile,
)


def test_registry_enumerates_only_explicit_backend_ids():
    assert BACKEND_IDS == {
        'lio_sam_noloop', 'point_lio', 'fast_livo2_lio',
        'fast_lio2_legacy', 'lio_sam_loop_experimental',
    }


def test_default_is_user_selected_fast_livo2_lio():
    profile = resolve_backend(check_sources=False)
    assert profile['backend']['id'] == 'fast_livo2_lio'
    assert profile['backend']['mode'] == 'lio_only'
    assert profile['backend']['loop_closure'] is False
    assert profile['backend']['gps_factor'] is False
    assert profile['backend']['external_global_correction'] is False


def test_lio_sam_candidate_is_explicit_no_loop():
    profile = resolve_backend('lio_sam_noloop', check_sources=False)
    assert profile['backend']['mode'] == 'no_loop'
    assert profile['backend']['loop_closure'] is False
    assert profile['backend']['gps_factor'] is False
    assert profile['backend']['external_global_correction'] is False


def test_unsupported_backend_fails_closed():
    with pytest.raises(BackendSelectionError, match='unsupported'):
        resolve_backend('some_other_lio', check_sources=False)


def test_fastlio2_requires_explicit_experimental_opt_in():
    with pytest.raises(BackendSelectionError, match='FAST-LIO2 is disabled'):
        resolve_backend('fast_lio2_legacy', check_sources=False)


def test_fast_livo_profile_cannot_enable_visual_or_global_correction():
    profile = resolve_backend('fast_livo2_lio', check_sources=False)
    profile['backend']['external_global_correction'] = True
    with pytest.raises(BackendSelectionError, match='must be LIO-only'):
        validate_profile(profile, 'fast_livo2_lio')


def test_loop_profile_cannot_be_mislabeled_as_no_loop():
    profile = resolve_backend('lio_sam_loop_experimental', check_sources=False,
                              allow_experimental_backend=True)
    profile['backend']['mode'] = 'no_loop'
    with pytest.raises(BackendSelectionError, match='no-loop profile enables'):
        validate_profile(profile, 'lio_sam_loop_experimental')


def test_missing_native_binary_fails_closed(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    config = tmp_path / 'config.yaml'
    config.write_text('value: true\n')
    profile = {
        'backend': {
            'id': 'point_lio', 'source_path': str(source), 'source_commit': 'x',
            'config_path': str(config), 'config_sha256': '0' * 64,
            'required_executables': [str(tmp_path / 'missing_binary')],
        }
    }
    import hashlib
    profile['backend']['config_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest()
    with pytest.raises(BackendSelectionError, match='missing or not executable'):
        _verify_local_provenance(profile)


def test_profiles_are_installed_from_the_mapping_workspace():
    assert (PROFILE_DIR / 'lio_sam_noloop.yaml').is_file()
    assert (PROFILE_DIR / 'point_lio.yaml').is_file()
    assert SELECTION_PATH.is_file()
