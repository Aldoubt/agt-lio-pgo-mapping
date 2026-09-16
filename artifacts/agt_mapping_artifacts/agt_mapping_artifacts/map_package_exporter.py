"""Export the validated PGO output into the cross-repository Map Package."""

from __future__ import annotations

import hashlib
import shutil
import tempfile
from pathlib import Path
from typing import Any

import yaml


REQUIRED_FILES = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'calibration.yaml', 'metadata.yaml')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_integrity_files(root: Path) -> None:
    files = sorted(path for path in root.rglob('*') if path.is_file() and path.name not in {'checksums.sha256', 'manifest.yaml'})
    checksums = {path.relative_to(root).as_posix(): _sha256(path) for path in files}
    (root / 'checksums.sha256').write_text(
        ''.join(f'{digest}  {name}\n' for name, digest in checksums.items()), encoding='utf-8')
    manifest = {
        'schema_version': 1,
        'package_kind': 'mapping_source',
        'required_files': list(REQUIRED_FILES) + ['patches/'],
        'checksums_file': 'checksums.sha256',
        'checksums': checksums,
    }
    (root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')


class MapPackageExporter:
    """Atomically publish a PGO artifact at ``<root>/<site>/<version>``."""

    def export(self, source_dir: str | Path, map_root: str | Path, site: str, version: str,
               relocalization_assets: str | Path | None = None,
               calibration: str | Path | None = None) -> Path:
        source = Path(source_dir).expanduser().resolve()
        destination = Path(map_root).expanduser().resolve() / site / version
        if not source.is_dir():
            raise FileNotFoundError(f'PGO artifact directory does not exist: {source}')
        missing = [name for name in REQUIRED_FILES if not (source / name).is_file()]
        if not (source / 'patches').is_dir():
            missing.append('patches/')
        if missing:
            raise FileNotFoundError(f'PGO artifact is incomplete; missing: {", ".join(missing)}')
        metadata: dict[str, Any] = yaml.safe_load((source / 'metadata.yaml').read_text(encoding='utf-8')) or {}
        status = metadata.get('backend_status') or {}
        if metadata.get('backend') != 'PGO' or status.get('optimized') is not True:
            raise ValueError('only optimized PGO artifacts can be published as a Map Package')
        if destination.exists():
            raise FileExistsError(f'Map Package already exists: {destination}')

        destination.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=f'.{version}.staging-', dir=str(destination.parent)))
        try:
            for name in REQUIRED_FILES:
                shutil.copy2(source / name, staging / name)
            shutil.copytree(source / 'patches', staging / 'patches')
            if calibration:
                shutil.copy2(Path(calibration).expanduser(), staging / 'calibration.yaml')
            if relocalization_assets:
                shutil.copytree(Path(relocalization_assets).expanduser(), staging / 'relocalization')
            package_metadata = {
                'schema_version': 1,
                'package_kind': 'mapping_source',
                'site': site,
                'version': version,
                'backend': 'PGO',
                'backend_status': {'optimized': True},
                'frames': {'map': 'map', 'mapping_body': 'body', 'lidar': 'lidar'},
                'pose_semantics': 'T_map_body',
                'patches_frame': 'body',
                'poses_format': 'poses.txt: patch_name x y z qw qx qy qz',
                'source_artifact': str(source),
                'relocalization': {'available': bool(relocalization_assets), 'path': 'relocalization'},
            }
            (staging / 'metadata.yaml').write_text(yaml.safe_dump(package_metadata, sort_keys=True), encoding='utf-8')
            _write_integrity_files(staging)
            staging.rename(destination)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        return destination
