from __future__ import annotations

import hashlib
import shutil
import tempfile
from pathlib import Path

import yaml

from .nav_export import write_nav_map
from .pcd import read_pcd, write_pcd
from .rules import load_rules, point_is_removed


SOURCE_FILES = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'metadata.yaml', 'manifest.yaml',
                'checksums.sha256')
COPIED_ITEMS = ('poses.txt', 'poses_timed.txt', 'metadata.yaml', 'patches', 'calibration.yaml')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_integrity(root: Path, source_root: Path) -> None:
    files = sorted(path for path in root.rglob('*') if path.is_file()
                   and path.name not in {'checksums.sha256', 'manifest.yaml'})
    checksums = {path.relative_to(root).as_posix(): _sha256(path) for path in files}
    (root / 'checksums.sha256').write_text(
        ''.join(f'{digest}  {name}\n' for name, digest in checksums.items()), encoding='utf-8')
    manifest = {
        'schema_version': 1,
        'package_kind': 'refined_mapping_source',
        'parent_package': str(source_root),
        'parent_manifest_sha256': _sha256(source_root / 'manifest.yaml'),
        'parent_map_sha256': _sha256(source_root / 'map.pcd'),
        'checksums_file': 'checksums.sha256',
        'checksums': checksums,
    }
    (root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')


def refine_map_package(source_path: str | Path, refinement_path: str | Path,
                       output_path: str | Path, resolution: float = 0.05,
                       pcd_format: str = 'auto', write_preview_nav_map: bool = True) -> Path:
    """Create a derived ``refined_mapping_source`` package next to an immutable source.

    ``pcd_format`` selects the refined ``map.pcd`` encoding: ``auto`` keeps the source
    encoding, ``binary`` writes float32 binary, ``ascii`` writes text. The optional
    ``nav_map.*`` files are a quick preview only; navigation-grade layers come from the
    navigation repository converter.
    """
    source = Path(source_path).expanduser().resolve()
    rules_path = Path(refinement_path).expanduser().resolve()
    destination = Path(output_path).expanduser().resolve()
    if not source.is_dir():
        raise FileNotFoundError(f'map package does not exist: {source}')
    missing = [name for name in SOURCE_FILES if not (source / name).is_file()]
    if not (source / 'patches').is_dir():
        missing.append('patches/')
    if missing:
        raise FileNotFoundError(f'map package is incomplete; missing: {", ".join(missing)}')
    if destination.exists() and any(destination.iterdir()):
        raise FileExistsError(f'refined output is not empty: {destination}')
    if pcd_format not in ('auto', 'ascii', 'binary'):
        raise ValueError(f'unsupported pcd_format: {pcd_format}')
    rules = load_rules(rules_path)
    pcd = read_pcd(source / 'map.pcd')
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.refined-', dir=str(destination.parent)))
    try:
        for name in COPIED_ITEMS:
            source_item = source / name
            if source_item.exists():
                if source_item.is_dir():
                    shutil.copytree(source_item, staging / name)
                else:
                    shutil.copy2(source_item, staging / name)
        filtered_rows = [row for row in pcd.rows if not point_is_removed(
            float(row[pcd.x_index]), float(row[pcd.y_index]), float(row[pcd.z_index]), rules.operations)]
        refined = type(pcd)(pcd.header, pcd.fields, filtered_rows, pcd.source_format)
        written_format = write_pcd(staging / 'map.pcd', refined, pcd_format)
        shutil.copy2(rules_path, staging / 'refinement.yaml')
        if write_preview_nav_map:
            write_nav_map(refined, rules.operations, staging / 'nav_map.pgm', staging / 'nav_map.yaml',
                          resolution=resolution)
        (staging / 'filter_report.yaml').write_text(yaml.safe_dump({
            'status': 'deferred',
            'dynamic_filter': 'offline_evidence_interface_only',
            'input_map_points': len(pcd.rows),
            'output_map_points': len(filtered_rows),
            'removed_by_rules': len(pcd.rows) - len(filtered_rows),
            'rule_count': len(rules.operations),
            'map_pcd_format': written_format,
            'patches': 'copied unfiltered from parent package as source evidence',
            'nav_map_preview': bool(write_preview_nav_map),
        }, sort_keys=True), encoding='utf-8')
        metadata = yaml.safe_load((staging / 'metadata.yaml').read_text(encoding='utf-8')) or {}
        metadata['refinement'] = {'parent_package': str(source), 'rules': 'refinement.yaml',
                                  'dynamic_filter': 'deferred', 'map_pcd_format': written_format,
                                  'removed_points': len(pcd.rows) - len(filtered_rows)}
        (staging / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
        _write_integrity(staging, source)
        staging.rename(destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination