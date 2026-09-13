from pathlib import Path

import yaml

from agt_mapping_artifacts import ArtifactWriter
from agt_mapping_artifacts.artifact_writer import EMPTY_PCD


def test_mock_optimized_pgo_output_generates_real_artifact_copy(tmp_path):
    source = tmp_path / 'pgo'
    (source / 'patches').mkdir(parents=True)
    (source / 'map.pcd').write_text(EMPTY_PCD)
    (source / 'patches' / '0.pcd').write_text(EMPTY_PCD)
    (source / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
    (source / 'poses_timed.txt').write_text('0.pcd 1.000000000 0 0 0 1 0 0 0\n')
    root = ArtifactWriter(tmp_path / 'output').write_optimized_pgo(source)
    assert (root / 'map.pcd').read_text() == EMPTY_PCD
    assert (root / 'patches' / '0.pcd').is_file()
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    assert metadata['backend'] == 'PGO'
    assert metadata['backend_status']['optimized'] is True
    assert (root / 'checksums.sha256').is_file()
