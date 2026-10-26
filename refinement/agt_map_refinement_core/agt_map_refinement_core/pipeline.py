from __future__ import annotations

import math
import shutil
import tempfile
from pathlib import Path

import yaml

from agt_mapping_artifacts.derived_export import (
    publish_directory, require_current_source, snapshot_source,
)
from agt_mapping_artifacts.frontend_package import require_output_outside, sha256
from agt_mapping_artifacts.keyframe_blocks import _read_pose_records
from agt_mapping_artifacts.map_package_exporter import _write_integrity_files

from .nav_export import write_nav_map
from .pcd import read_pcd, write_pcd
from .rules import load_rules, point_is_removed


SOURCE_FILES = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'metadata.yaml', 'manifest.yaml',
                'checksums.sha256', 'calibration.yaml')
COPIED_ITEMS = ('poses.txt', 'poses_timed.txt', 'metadata.yaml', 'calibration.yaml')


def _write_integrity(root: Path, source_root: Path) -> None:
    # Extend the existing mapping-source manifest, not a new derivative schema.
    _write_integrity_files(root)
    manifest = yaml.safe_load((root / 'manifest.yaml').read_text(encoding='utf-8'))
    manifest.update({
        'package_kind': 'refined_mapping_source', 'parent_package': str(source_root),
        'parent_manifest_sha256': sha256(source_root / 'manifest.yaml'),
        'parent_map_sha256': sha256(source_root / 'map.pcd'),
        'parent_checksums_sha256': sha256(source_root / 'checksums.sha256'),
    })
    (root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')


def _paired_poses(source: Path):
    """Reuse the timed-pose reader and require its exact plain-pose counterpart."""
    records = _read_pose_records(source)
    plain = [line.split() for line in (source / 'poses.txt').read_text(encoding='ascii').splitlines()
             if line.strip() and not line.lstrip().startswith('#')]
    if len(plain) != len(records):
        raise ValueError('poses.txt and poses_timed.txt have different keyframe counts')
    for tokens, record in zip(plain, records):
        if len(tokens) != 8 or tokens[0] != record['patch_id']:
            raise ValueError('poses.txt and poses_timed.txt have different patch references')
        qx, qy, qz, qw = record['quaternion_xyzw']
        expected = [*record['position'], qw, qx, qy, qz]
        if (any(not math.isfinite(float(a)) or abs(float(a) - float(b)) > 1e-10
                for a, b in zip(tokens[1:], expected)) or
                abs(math.sqrt(sum(float(v)**2 for v in (qw, qx, qy, qz))) - 1.0) > 1e-3):
            raise ValueError('pose values differ or quaternion is not normalized')
    expected_names = {record['patch_id'] for record in records}
    actual_names = {path.name for path in (source / 'patches').iterdir()}
    if actual_names != expected_names:
        raise ValueError('patch directory and pose references differ')
    return records


def _xyz(pcd, row):
    point = [float(row[index]) for index in (pcd.x_index, pcd.y_index, pcd.z_index)]
    if not all(math.isfinite(value) for value in point):
        raise ValueError('refinement requires finite point coordinates')
    return point


def _refine_patches(source, staging, records, template, operations, pcd_format):
    """Filter in map coordinates; retain body rows and rebuild their exact map union."""
    (staging / 'patches').mkdir()
    world_rows = []
    input_count = 0
    removed_count = 0
    for record in records:
        patch = read_pcd(source / 'patches' / record['patch_id'])
        if (patch.fields != template.fields or
                any(patch.header_value(key) != template.header_value(key)
                    for key in ('SIZE', 'TYPE', 'COUNT'))):
            raise ValueError(f'patch and map point layouts differ: {record["patch_id"]}')
        retained = []
        for row in patch.rows:
            world = record['rotation'].dot(_xyz(patch, row)) + record['position']
            input_count += 1
            if point_is_removed(*world, operations):
                removed_count += 1
                continue
            retained.append(row)
            transformed = list(row)
            for index, value in zip((patch.x_index, patch.y_index, patch.z_index), world):
                transformed[index] = repr(float(value))
            world_rows.append(transformed)
        patch.rows = retained
        write_pcd(staging / 'patches' / record['patch_id'], patch, pcd_format)
    return world_rows, input_count, removed_count


def refine_map_package(source_path: str | Path, refinement_path: str | Path,
                       output_path: str | Path, resolution: float = 0.05,
                       pcd_format: str = 'auto', write_preview_nav_map: bool = True) -> Path:
    """Create a derived ``refined_mapping_source`` package next to an immutable source.

    ``pcd_format`` selects the refined ``map.pcd`` encoding: ``auto`` keeps the source
    encoding, ``binary`` writes float32 binary, ``ascii`` writes text. Every 3D
    removal is evaluated in map coordinates on the paired body-frame patches;
    map.pcd is rebuilt from their retained observations. Pose bytes stay unchanged.
    The optional
    ``nav_map.*`` files are a quick preview only; navigation-grade layers come from the
    navigation repository converter.
    """
    supplied_source = Path(source_path).expanduser()
    if supplied_source.is_symlink():
        raise ValueError('source root must not be a symbolic link')
    source = supplied_source.resolve()
    rules_path = Path(refinement_path).expanduser().resolve()
    destination = require_output_outside(output_path, [source], label='refined output')
    if not source.is_dir():
        raise FileNotFoundError(f'map package does not exist: {source}')
    missing = [name for name in SOURCE_FILES if not (source / name).is_file()]
    if not (source / 'patches').is_dir():
        missing.append('patches/')
    if missing:
        raise FileNotFoundError(f'map package is incomplete; missing: {", ".join(missing)}')
    if destination.exists():
        raise FileExistsError(f'refined output already exists: {destination}')
    if pcd_format not in ('auto', 'ascii', 'binary'):
        raise ValueError(f'unsupported pcd_format: {pcd_format}')
    source_snapshot = snapshot_source(source)
    rules_digest = sha256(rules_path)
    rules = load_rules(rules_path)
    pcd = read_pcd(source / 'map.pcd')
    records = _paired_poses(source)
    metadata = yaml.safe_load((source / 'metadata.yaml').read_text(encoding='utf-8')) or {}
    if not isinstance(metadata, dict):
        raise ValueError('source metadata must be a YAML mapping')
    if (metadata.get('patches_frame', 'body') != 'body' or
            metadata.get('pose_semantics', 'T_map_body') not in ('T_map_body', 'T_frontend_body')):
        raise ValueError('refinement requires body-frame patches with map-from-body poses')
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
        removed_map_points = sum(point_is_removed(*_xyz(pcd, row), rules.operations) for row in pcd.rows)
        filtered_rows, patch_input_count, patch_removed_count = _refine_patches(
            source, staging, records, pcd, rules.operations, pcd_format)
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
            'removed_by_rules': removed_map_points,
            'input_patch_points': patch_input_count,
            'removed_patch_points': patch_removed_count,
            'rule_count': len(rules.operations),
            'map_pcd_format': written_format,
            'patches': 'same map-frame removal rules applied to paired body-frame patches',
            'map': 'rebuilt from retained paired patch observations; may differ from parent map sampling',
            'trajectory': 'preserved; footprint sweep remains a downstream mapping policy',
            'nav_map_preview': bool(write_preview_nav_map),
        }, sort_keys=True), encoding='utf-8')
        # A derived source must not retain an unmodified-frontend validation claim.
        metadata['artifact_kind'] = 'refined_mapping_source'
        metadata['package_kind'] = 'refined_mapping_source'
        metadata['keyframe_count'] = len(records)
        metadata['map_point_count'] = len(filtered_rows)
        if 'relocalization' in metadata:
            metadata['relocalization'] = {
                'available': False, 'reason': 'rebuild localization assets for the refined geometry',
            }
        metadata['refinement'] = {'parent_package': str(source), 'rules': 'refinement.yaml',
                                  'dynamic_filter': 'deferred', 'map_pcd_format': written_format,
                                  'removed_points': removed_map_points,
                                  'removed_patch_points': patch_removed_count,
                                  'rules_sha256': rules_digest}
        (staging / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
        _write_integrity(staging, source)
        snapshot_source(staging)  # Validate the existing derived-source integrity contract before publish.
        require_current_source(source, source_snapshot)
        if sha256(rules_path) != rules_digest or sha256(staging / 'refinement.yaml') != rules_digest:
            raise ValueError('refinement rules changed while deriving the package')
        publish_directory(staging, destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination
