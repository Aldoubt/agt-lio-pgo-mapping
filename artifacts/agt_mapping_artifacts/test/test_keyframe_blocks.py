from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_mapping_artifacts.frontend_package import (
    require_output_outside, sha256, verify_frontend_map_package, write_frontend_map_package,
)
from agt_mapping_artifacts.block_cli import _reviewed_physical_identity, preview_main
from agt_mapping_artifacts.keyframe_blocks import (
    build_keyframe_blocks, partition_pose_records, verify_keyframe_blocks,
)


def make_source(root: Path, count: int = 8) -> Path:
    records = []
    for i in range(count):
        records.append({
            'stamp_sec': 100 + i, 'stamp_nanosec': 0,
            'position': [float(i % 4), float(i // 4), 0.0],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            'points_xyzi': np.asarray([[0, 0, 0, i], [.1, 0, 0, i + 1],
                                       [0, .1, 0, i + 2]], dtype='<f4'),
        })
    provenance = {
        'mapping_backend': {'id': 'fast_livo2_lio', 'project': 'synthetic', 'mode': 'lio_only',
                            'source_commit': 'fixture', 'config_sha256': 'a' * 64,
                            'loop_closure': False, 'gps_factor': False,
                            'external_global_correction': False},
        'source': {'kind': 'test_fixture'}, 'frames': {'map': 'map', 'body': 'body'},
        'reference': {'source': 'mapping_frontend_odometry', 'same_session': True,
                      'pgo_applied': False, 'optimized': False,
                      'absolute_ground_truth': False},
    }
    root.mkdir(parents=True)
    package = root / 'source'
    write_frontend_map_package(package, records, provenance)
    assert verify_frontend_map_package(package)['status'] == 'PASS'
    return package


def zero_first_quaternion(package: Path):
    path = package / 'poses_timed.txt'
    tokens = path.read_text(encoding='ascii').splitlines()[0].split()
    tokens[5:9] = ['0', '0', '0', '0']
    path.write_text(' '.join(tokens) + '\n', encoding='ascii')


def test_partition_endpoints_missing_ids_and_spatial_revisit():
    records = [{'keyframe_id': i, 'timestamp': float(i), 'position': [0., 0., 0.]}
               for i in (0, 1, 2, 10, 11, 12, 13)]
    blocks = partition_pose_records(records, block_keyframe_count=3, stride=3,
                                    max_timestamp_gap_s=2.0)
    assert [[r['keyframe_id'] for r in b] for b in blocks] == [[0, 1, 2], [10, 11, 12], [13]]
    # Position is intentionally identical: spatial revisit never joins temporal segments.


@pytest.mark.parametrize('mutator', [
    lambda p: (p / 'patches' / '3.pcd').unlink(),
    lambda p: (p / 'poses_timed.txt').write_text('0.pcd 100 0 0 0 0 0 0 1\n', encoding='ascii'),
    lambda p: (p / 'patches' / '2.pcd').write_bytes((p / 'patches' / '2.pcd').read_bytes()[:-1] + b'X'),
    zero_first_quaternion,
])
def test_builder_rejects_missing_or_tampered_source(mutator, tmp_path):
    source = make_source(tmp_path / 'fixture')
    mutator(source)
    with pytest.raises((ValueError, OSError)):
        build_keyframe_blocks(source, tmp_path / 'blocks', block_keyframe_count=3)
    assert not (tmp_path / 'blocks').exists()


def test_block_assets_are_deterministic_and_verified(tmp_path):
    source = make_source(tmp_path / 'fixture')
    first = build_keyframe_blocks(source, tmp_path / 'blocks_a', block_keyframe_count=3,
                                  stride=3, max_timestamp_gap_s=2.0)
    second = build_keyframe_blocks(source, tmp_path / 'blocks_b', block_keyframe_count=3,
                                   stride=3, max_timestamp_gap_s=2.0)
    assert first['block_set_id'] == second['block_set_id']
    assert first['block_count'] == 3  # two complete blocks plus a truncated map endpoint
    for item_a, item_b in zip(first['blocks'], second['blocks']):
        assert item_a['output_content_sha256'] == item_b['output_content_sha256']
    assert all(block['row_id'] == 'UNKNOWN' and block['review_state'] == 'UNKNOWN'
               for block in first['blocks'])
    verified = verify_keyframe_blocks(tmp_path / 'blocks_a', source_package=source)
    assert verified['status'] == 'PASS'
    assert len(json.loads((tmp_path / 'blocks_a' / 'block_index.json').read_text())['timestamp_lookup']) == 8


def test_block_builder_refuses_to_write_inside_source_package(tmp_path):
    source = make_source(tmp_path / 'fixture')
    output = source / 'derived_blocks'
    with pytest.raises(ValueError, match='outside protected asset'):
        build_keyframe_blocks(source, output, block_keyframe_count=3)
    assert not output.exists()


def test_display_preview_requires_current_source_and_external_destination(tmp_path):
    source = make_source(tmp_path / 'fixture')
    blocks = tmp_path / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=3)
    with pytest.raises(ValueError, match='outside protected asset'):
        preview_main(['--block-dir', str(blocks), '--source-package', str(source),
                      '--output-pcd', str(source / 'preview.pcd')])
    alias = tmp_path / 'source-alias'
    alias.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError, match='outside protected asset'):
        require_output_outside(alias / 'preview.pcd', [source], label='preview')


def test_block_verifier_rejects_tampered_content_and_stale_source(tmp_path):
    source = make_source(tmp_path / 'fixture')
    blocks = tmp_path / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=4)
    first = blocks / 'blocks' / 'block_0000.pcd'
    first.write_bytes(first.read_bytes() + b'tamper')
    with pytest.raises(ValueError, match='checksum mismatch'):
        verify_keyframe_blocks(blocks)
    # Restoring bytes and changing the parent revision proves source binding.
    first.write_bytes(first.read_bytes()[:-6])
    source_manifest = source / 'manifest.yaml'
    manifest = yaml.safe_load(source_manifest.read_text())
    manifest['test_revision'] = 2
    source_manifest.write_text(yaml.safe_dump(manifest), encoding='utf-8')
    # The package validator rejects this changed manifest before stale binding is considered.
    with pytest.raises((ValueError, OSError)):
        verify_keyframe_blocks(blocks, source_package=source)


@pytest.mark.parametrize('index_output', [False, True])
def test_block_verifier_rejects_output_escaped_through_symlink_parent(tmp_path, index_output):
    source = make_source(tmp_path / 'fixture')
    blocks = tmp_path / 'blocks'
    build_keyframe_blocks(source, blocks, block_keyframe_count=4)
    manifest = yaml.safe_load((blocks / 'manifest.yaml').read_text(encoding='utf-8'))
    escaped = tmp_path / 'escaped-blocks'
    shutil.move(blocks / 'blocks', escaped)
    (blocks / 'blocks').symlink_to(escaped, target_is_directory=True)

    lines = []
    for path in sorted(blocks.rglob('*')):
        if path.is_file() and path.name != 'checksums.sha256':
            lines.append(f'{sha256(path)}  {path.relative_to(blocks).as_posix()}')
    if index_output:
        for item in manifest['blocks']:
            output = escaped / Path(item['output_path']).name
            lines.append(f'{sha256(output)}  {item["output_path"]}')
    (blocks / 'checksums.sha256').write_text('\n'.join(sorted(lines)) + '\n', encoding='ascii')

    expected = 'symbolic link' if index_output else 'not covered by checksums'
    with pytest.raises(ValueError, match=expected):
        verify_keyframe_blocks(blocks)


@pytest.mark.parametrize('records', [
    [{'keyframe_id': 0, 'timestamp': 1.0}, {'keyframe_id': 1, 'timestamp': 1.0}],
    [{'keyframe_id': 0, 'timestamp': 2.0}, {'keyframe_id': 1, 'timestamp': 1.0}],
    [{'keyframe_id': 0, 'timestamp': float('nan')}],
    [{'keyframe_id': 0, 'timestamp': 1.0}, {'keyframe_id': 0, 'timestamp': 2.0}],
])
def test_partition_rejects_duplicate_disordered_or_nonfinite_records(records):
    with pytest.raises(ValueError):
        partition_pose_records(records, block_keyframe_count=2, stride=2,
                               max_timestamp_gap_s=2.0)


def test_label_metadata_requires_a_reviewed_annotation_digest(tmp_path):
    source = make_source(tmp_path / 'fixture')
    labels = {i: {'physical_row_id': 'row_1', 'scene_type': 'ROW_MIDDLE',
                  'along_row_s_m': float(i), 'label_confidence': 'confirmed'}
              for i in range(3)}
    with pytest.raises(ValueError, match='annotation SHA-256'):
        build_keyframe_blocks(source, tmp_path / 'blocks', block_keyframe_count=3,
                              frame_labels=labels)
    result = build_keyframe_blocks(source, tmp_path / 'blocks_ok', block_keyframe_count=3,
                                   frame_labels=labels, row_annotation_sha256='b' * 64)
    assert result['blocks'][0]['row_id'] == 'row_1'
    assert result['blocks'][1]['row_id'] == 'UNKNOWN'


def test_draft_structure_never_exports_manual_physical_ids():
    draft = {'status': 'draft', 'manual_review_confirmed': False}
    frozen_unreviewed = {'status': 'frozen', 'manual_review_confirmed': False}
    reviewed = {'status': 'frozen', 'manual_review_confirmed': True}
    assert _reviewed_physical_identity(draft, 'row_17') == 'UNKNOWN'
    assert _reviewed_physical_identity(frozen_unreviewed, 'row_17') == 'UNKNOWN'
    assert _reviewed_physical_identity(reviewed, 'row_17') == 'row_17'
    assert _reviewed_physical_identity(reviewed, 'UNKNOWN') == 'UNKNOWN'
