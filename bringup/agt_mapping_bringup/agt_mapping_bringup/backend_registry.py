"""Explicit, fail-closed registry for mapping LIO frontends."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import subprocess
from typing import Any

import yaml


BACKEND_IDS = frozenset({
    'lio_sam_noloop',
    'point_lio',
    'fast_livo2_lio',
    'fast_lio2_legacy',
    'lio_sam_loop_experimental',
})
def _package_share() -> Path:
    source_share = Path(__file__).resolve().parents[1]
    if (source_share / 'config' / 'backends').is_dir():
        return source_share
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('agt_mapping_bringup'))
    except (ImportError, LookupError):
        return source_share


PROFILE_DIR = _package_share() / 'config' / 'backends'
SELECTION_PATH = _package_share() / 'config' / 'backend_selection.yaml'


class BackendSelectionError(ValueError):
    """A backend was unsupported, unaccepted, or failed provenance checks."""


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise BackendSelectionError(f'cannot read backend configuration {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise BackendSelectionError(f'backend configuration must be a mapping: {path}')
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _profile_path(profile_dir: Path, backend_id: str) -> Path:
    if backend_id not in BACKEND_IDS:
        raise BackendSelectionError(f'unsupported mapping backend: {backend_id}')
    return profile_dir / f'{backend_id}.yaml'


def validate_profile(profile: dict[str, Any], expected_id: str) -> dict[str, Any]:
    backend = profile.get('backend')
    sensor = profile.get('sensor')
    mapping = profile.get('mapping')
    if not all(isinstance(value, dict) for value in (backend, sensor, mapping)):
        raise BackendSelectionError('profile requires backend, sensor, and mapping mappings')
    if backend.get('id') != expected_id:
        raise BackendSelectionError(
            f'profile identity mismatch: expected {expected_id}, got {backend.get("id")!r}')
    required_backend = (
        'implementation', 'project', 'mode', 'source_commit', 'input_format',
        'loop_closure', 'gps_factor', 'output_odom_topic', 'output_cloud_topic',
        'output_path_topic', 'output_cloud_frame_mode',
    )
    missing = [key for key in required_backend if key not in backend]
    if missing:
        raise BackendSelectionError(f'{expected_id} profile missing backend fields: {missing}')
    for key in ('lidar_topic', 'imu_topic', 'lidar_frame', 'imu_frame', 'body_frame'):
        if not sensor.get(key):
            raise BackendSelectionError(f'{expected_id} profile missing sensor.{key}')
    if not mapping.get('map_frame'):
        raise BackendSelectionError(f'{expected_id} profile missing mapping.map_frame')
    if not isinstance(backend.get('loop_closure'), bool):
        raise BackendSelectionError('loop_closure must be an explicit boolean')
    if not isinstance(backend.get('gps_factor'), bool):
        raise BackendSelectionError('gps_factor must be an explicit boolean')
    if not isinstance(backend.get('external_global_correction'), bool):
        raise BackendSelectionError('external_global_correction must be an explicit boolean')
    mode = backend['mode']
    if mode == 'no_loop' and (backend['loop_closure'] or backend['gps_factor']
                              or backend['external_global_correction']):
        raise BackendSelectionError('no-loop profile enables loop, GPS, or global correction')
    if expected_id == 'lio_sam_noloop' and mode != 'no_loop':
        raise BackendSelectionError('lio_sam_noloop cannot select a loop-enabled mode')
    if expected_id in {'lio_sam_noloop', 'point_lio', 'fast_livo2_lio'} and backend.get('experimental') is True:
        raise BackendSelectionError(f'{expected_id} must not be silently reclassified as experimental')
    if expected_id == 'lio_sam_loop_experimental':
        if mode != 'loop_experimental' or backend['loop_closure'] is not True:
            raise BackendSelectionError('experimental loop profile must explicitly enable loop closure')
        if backend.get('experimental') is not True:
            raise BackendSelectionError('loop profile must be explicitly marked experimental')
    if expected_id == 'fast_lio2_legacy' and backend.get('experimental') is not True:
        raise BackendSelectionError('FAST-LIO2 legacy profile must be marked experimental')
    if expected_id == 'fast_lio2_legacy' and backend.get('status') != 'experimental_legacy':
        raise BackendSelectionError('FAST-LIO2 legacy status must remain experimental_legacy')
    if backend.get('output_cloud_frame_mode') not in {'body', 'lidar', 'frontend'}:
        raise BackendSelectionError('output_cloud_frame_mode must be body, lidar, or frontend')
    if not backend.get('source_path') or not backend.get('config_path'):
        raise BackendSelectionError('profile must identify source_path and config_path')
    return profile


def _verify_local_provenance(profile: dict[str, Any]) -> None:
    backend = profile['backend']
    source = Path(str(backend.get('source_path', ''))).expanduser()
    if not source.is_dir():
        raise BackendSelectionError(f'backend source path is missing: {source}')
    config = Path(str(backend.get('config_path', ''))).expanduser()
    expected_hash = backend.get('config_sha256')
    if not config.is_file() or not isinstance(expected_hash, str):
        raise BackendSelectionError(f'backend config or config_sha256 is missing: {config}')
    actual_hash = _sha256(config)
    if actual_hash != expected_hash:
        raise BackendSelectionError(f'backend config hash mismatch for {config}')
    for executable_value in backend.get('required_executables', []):
        executable = Path(executable_value).expanduser()
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise BackendSelectionError(f'backend executable is missing or not executable: {executable}')
    if not backend.get('required_executables'):
        raise BackendSelectionError('profile must declare at least one required executable')
    patch_path_value = backend.get('source_patch')
    patch_digest = backend.get('source_patch_sha256')
    if patch_path_value is not None:
        patch_path = Path(str(patch_path_value)).expanduser()
        if not patch_path.is_file() or not isinstance(patch_digest, str):
            raise BackendSelectionError(f'source patch or source_patch_sha256 is missing: {patch_path}')
        if _sha256(patch_path) != patch_digest:
            raise BackendSelectionError(f'source patch hash mismatch: {patch_path}')
    source_files = backend.get('source_files_sha256', {})
    if not isinstance(source_files, dict):
        raise BackendSelectionError('source_files_sha256 must be a path-to-hash mapping')
    for source_file_value, expected_digest in source_files.items():
        source_file = Path(source_file_value).expanduser()
        if not source_file.is_file() or _sha256(source_file) != expected_digest:
            raise BackendSelectionError(f'backend source file hash mismatch: {source_file}')
    repo = source
    while repo != repo.parent and not (repo / '.git').exists():
        repo = repo.parent
    if (repo / '.git').exists():
        try:
            current = subprocess.run(
                ['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                check=True, capture_output=True, text=True,
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise BackendSelectionError(f'cannot verify source revision at {repo}: {exc}') from exc
        expected = str(backend['source_commit'])
        # FAST-LIVO2 is vendored under a larger repository and carries its own
        # upstream tree revision in the old benchmark manifest. Other entries
        # record the containing repository HEAD directly.
        if backend['id'] != 'fast_livo2_lio' and current != expected:
            raise BackendSelectionError(
                f'source revision mismatch for {backend["id"]}: expected {expected}, got {current}')
        expected_container = backend.get('containing_repository_commit')
        if expected_container is not None and current != expected_container:
            raise BackendSelectionError(
                f'containing repository revision mismatch for {backend["id"]}: '
                f'expected {expected_container}, got {current}')
        source_tree_path = backend.get('source_tree_path')
        expected_tree = backend.get('source_tree_object')
        if source_tree_path is not None:
            try:
                actual_tree = subprocess.run(
                    ['git', '-C', str(repo), 'rev-parse', f'HEAD:{source_tree_path}'],
                    check=True, capture_output=True, text=True,
                ).stdout.strip()
            except (OSError, subprocess.CalledProcessError) as exc:
                raise BackendSelectionError(f'cannot verify source tree {source_tree_path}: {exc}') from exc
            if expected_tree is not None and actual_tree != expected_tree:
                raise BackendSelectionError(
                    f'source subtree revision mismatch for {backend["id"]}: '
                    f'expected {expected_tree}, got {actual_tree}')


def resolve_backend(
    backend_id: str | None = None,
    *,
    allow_experimental_backend: bool = False,
    selection_path: Path = SELECTION_PATH,
    profile_dir: Path = PROFILE_DIR,
    check_sources: bool = True,
) -> dict[str, Any]:
    """Load and validate an explicitly selected backend profile.

    No backend is implicitly selected until one passes current-bag acceptance.
    Experimental/legacy profiles require an explicit opt-in.
    """
    selection = _load_yaml(Path(selection_path))
    if backend_id is None or not str(backend_id).strip():
        default = selection.get('default_backend')
        if default not in BACKEND_IDS:
            candidate = selection.get('candidate_default_backend', 'none')
            raise BackendSelectionError(
                f'default backend is not established; explicitly select a reviewed backend '
                f'(current candidate: {candidate})')
        backend_id = str(default)
    backend_id = str(backend_id)
    profile = validate_profile(_load_yaml(_profile_path(Path(profile_dir), backend_id)), backend_id)
    backend = profile['backend']
    if backend.get('experimental') is True and not allow_experimental_backend:
        raise BackendSelectionError(
            f'{backend_id} is experimental/legacy; set allow_experimental_backend:=true explicitly')
    if check_sources:
        _verify_local_provenance(profile)
    return profile
