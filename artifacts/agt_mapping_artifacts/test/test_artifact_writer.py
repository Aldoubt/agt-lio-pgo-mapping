import hashlib
from pathlib import Path

import yaml

from agt_mapping_artifacts import ArtifactWriter


def test_writes_complete_unoptimized_artifact_and_consistent_hashes(tmp_path):
    keyframes = [{
        'stamp': {'sec': 7, 'nanosec': 8},
        'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
    }]
    root = ArtifactWriter(tmp_path).write(keyframes, None, 'artifact_export_requested')
    for relative in ('map.pcd', 'poses.txt', 'poses_timed.txt', 'patches/0.pcd',
                     'metadata.yaml', 'calibration.yaml', 'checksums.sha256', 'manifest.yaml'):
        assert (root / relative).is_file()
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    assert metadata['backend_status']['optimized'] is False
    assert 'frontend_pose' in metadata['pose_semantics']
    assert metadata['dense_map']['available'] is False
    checksums = {name: digest for digest, name in
                 (line.split('  ', 1) for line in (root / 'checksums.sha256').read_text().splitlines())}
    assert hashlib.sha256((root / 'map.pcd').read_bytes()).hexdigest() == checksums['map.pcd']
