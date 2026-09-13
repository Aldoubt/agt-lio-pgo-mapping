"""Self-contained, deterministic writers for Mapping Artifact v0.1."""

from __future__ import annotations

import hashlib
import shutil
from pathlib import Path
from typing import Any, Iterable

import yaml


EMPTY_PCD = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 0
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 0
DATA ascii
"""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_checksums(root: Path) -> dict[str, str]:
    files = sorted(path for path in root.rglob('*') if path.is_file() and path.name != 'checksums.sha256')
    checksums = {str(path.relative_to(root)): sha256(path) for path in files}
    (root / 'checksums.sha256').write_text(
        ''.join(f'{value}  {name}\n' for name, value in checksums.items()), encoding='utf-8')
    return checksums


class ArtifactWriter:
    """Writes a v0.1 artifact without claiming unavailable dense-map evidence."""

    def __init__(self, output_dir: str | Path):
        self.root = Path(output_dir) / 'map_package'

    def write(
        self, keyframes: Iterable[dict[str, Any]], map_pose: dict[str, Any] | None,
        backend_status: str, calibration: dict[str, Any] | None = None,
    ) -> Path:
        if self.root.exists() and any(self.root.iterdir()):
            raise FileExistsError(f'artifact destination is not empty: {self.root}')
        patches = self.root / 'patches'
        patches.mkdir(parents=True, exist_ok=True)
        records = list(keyframes)
        (self.root / 'map.pcd').write_text(EMPTY_PCD, encoding='utf-8')
        pose_lines, timed_lines = [], []
        for index, record in enumerate(records):
            name = f'{index}.pcd'
            (patches / name).write_text(EMPTY_PCD, encoding='utf-8')
            p = record['position']
            q = record['orientation']
            stamp = record['stamp']
            pose_lines.append(f"{name} {p['x']} {p['y']} {p['z']} {q['w']} {q['x']} {q['y']} {q['z']}\n")
            timed_lines.append(
                f"{name} {stamp['sec']}.{stamp['nanosec']:09d} {p['x']} {p['y']} {p['z']} "
                f"{q['w']} {q['x']} {q['y']} {q['z']}\n")
        (self.root / 'poses.txt').write_text(''.join(pose_lines), encoding='utf-8')
        (self.root / 'poses_timed.txt').write_text(''.join(timed_lines), encoding='utf-8')
        calibration_data = calibration or {'format_version': 1, 'calibration_status': 'unavailable'}
        (self.root / 'calibration.yaml').write_text(yaml.safe_dump(calibration_data, sort_keys=True), encoding='utf-8')
        metadata = {
            'format_version': 1,
            'artifact_kind': 'mapping_artifact',
            'backend_status': {'state': backend_status, 'optimized': False},
            'pose_semantics': {
                'frontend_pose': 'T_local_mapping_body; input odometry before optimization',
                'optimized_map_pose': 'T_map_mapping_body; unavailable until external PGO correction is connected',
                'exported_map_pose': map_pose,
            },
            'dense_map': {'available': False, 'reason': 'backend contract does not provide fused clouds'},
            'patches': {'available': False, 'reason': 'backend contract does not provide keyframe clouds'},
            'keyframe_count': len(records),
            'outputs': {'map': 'map.pcd', 'patches_dir': 'patches', 'poses': 'poses.txt',
                        'poses_timed': 'poses_timed.txt', 'calibration': 'calibration.yaml'},
        }
        (self.root / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
        checksums = write_checksums(self.root)
        manifest = {'format_version': 1, 'artifact_root': 'map_package', 'checksums': checksums}
        (self.root / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')
        write_checksums(self.root)
        return self.root

    def write_optimized_pgo(self, source: str | Path, calibration: dict[str, Any] | None = None) -> Path:
        source = Path(source)
        if self.root.exists() and any(self.root.iterdir()):
            raise FileExistsError(f'artifact destination is not empty: {self.root}')
        self.root.mkdir(parents=True, exist_ok=True)
        for name in ('map.pcd', 'poses.txt', 'poses_timed.txt'):
            shutil.copy2(source / name, self.root / name)
        shutil.copytree(source / 'patches', self.root / 'patches')
        calibration_data = calibration or {'format_version': 1, 'calibration_status': 'unavailable'}
        (self.root / 'calibration.yaml').write_text(yaml.safe_dump(calibration_data, sort_keys=True), encoding='utf-8')
        metadata = {'format_version': 1, 'artifact_kind': 'mapping_artifact',
                    'backend': 'PGO', 'backend_status': {'optimized': True},
                    'pose_semantics': {'optimized_map_pose': 'T_map_mapping_body; external PGO optimized output'},
                    'dense_map': {'available': True, 'source': str(source)}}
        (self.root / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=True), encoding='utf-8')
        checksums = write_checksums(self.root)
        (self.root / 'manifest.yaml').write_text(yaml.safe_dump({'format_version': 1, 'checksums': checksums}, sort_keys=True), encoding='utf-8')
        write_checksums(self.root)
        return self.root
