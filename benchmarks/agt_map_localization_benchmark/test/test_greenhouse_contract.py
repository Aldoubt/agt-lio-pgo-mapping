from pathlib import Path

import pytest

from agt_map_localization_benchmark.greenhouse import (
    Scene, SceneConfig, grid_values, heldout_indices,
    load_scene_config, parse_frames, summarize,
)


def test_grid_values_includes_endpoint():
    assert grid_values(-1.0, 1.0, 0.5) == (-1.0, -0.5, 0.0, 0.5, 1.0)


def test_parse_frames_contract():
    assert parse_frames('1,3,5') == (1, 3, 5)
    with pytest.raises(ValueError):
        parse_frames('1,1')
    with pytest.raises(ValueError):
        parse_frames('2')


def test_scene_yaml_and_row_lookup(tmp_path: Path):
    config_path = tmp_path / 'scenes.yaml'
    config_path.write_text('''\
schema_version: 1
rows:
  - row_id: R1
    start_keyframe: 3
    end_keyframe: 10
  - row_id: R2
    start_keyframe: 11
    end_keyframe: 16
scenes:
  - id: r1_mid
    type: row_middle
    keyframe: 6
    row_id: R1
  - id: r1_end
    type: row_end
    keyframe: 9
    row_id: R1
''', encoding='utf-8')
    config = load_scene_config(config_path, pose_count=20)
    assert [s.scene_id for s in config.scenes] == ['r1_mid', 'r1_end']
    assert config.row_for_index(7) == 'R1'
    assert config.row_for_index(15) == 'R2'
    assert config.row_for_index(2) is None


def test_scene_yaml_rejects_overlap(tmp_path: Path):
    config_path = tmp_path / 'scenes.yaml'
    config_path.write_text('''\
schema_version: 1
rows:
  - row_id: R1
    start_keyframe: 2
    end_keyframe: 8
  - row_id: R2
    start_keyframe: 8
    end_keyframe: 12
scenes:
  - id: head
    type: headland
    keyframe: 5
''', encoding='utf-8')
    with pytest.raises(ValueError):
        load_scene_config(config_path, pose_count=20)


def test_heldout_covers_largest_query_windows():
    config = SceneConfig(
        scenes=(Scene('s1', 'row_middle', 10, 'R1', None),
                Scene('s2', 'headland', 20, None, None)),
        rows=(),
    )
    assert heldout_indices(config, (1, 3, 5)) == {8, 9, 10, 11, 12, 18, 19, 20, 21, 22}


def test_summary_separates_scene_types_and_bbs_seed_success():
    basin = [
        {'scene_type': 'row_middle', 'nominal_success': True, 'strict_success': False},
        {'scene_type': 'row_middle', 'nominal_success': False, 'strict_success': False},
        {'scene_type': 'headland', 'nominal_success': True, 'strict_success': True},
    ]
    global_rows = [
        {'scene_type': 'row_middle', 'final_nominal_success': False,
         'coarse_seed_gicp_nominal_success': False, 'wrong_row_candidate': True},
        {'scene_type': 'headland', 'final_nominal_success': True,
         'coarse_seed_gicp_nominal_success': True, 'wrong_row_candidate': False},
    ]
    summary = summarize(basin, global_rows)
    assert summary['by_scene_type']['row_middle']['basin_nominal']['fraction'] == 0.5
    assert summary['by_scene_type']['headland']['bbs_coarse_seed_gicp_nominal']['fraction'] == 1.0
    assert summary['overall']['wrong_row_candidate']['fraction'] == 0.5
