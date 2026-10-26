import numpy as np
import pytest
import yaml

from cross_stage_analysis.core import sha256_file
from cross_stage_analysis.pipeline import (
    _candidate_polygon,
    _check_map_hashes,
    _grid_counts,
    run_stage,
)


def test_grid_counts_use_shared_origin_resolution_and_z_filter():
    points = np.array([
        [0.1, 0.1, 0.0],
        [0.4, 0.4, 4.0],  # excluded by z filter
        [0.7, 0.2, 0.0],
    ])
    counts = _grid_counts(points, np.array([0.0, 0.0]), (2, 1), 0.5, (-1.0, 1.0))
    np.testing.assert_array_equal(counts, [[1, 1]])


def test_saved_alignment_is_bound_to_exact_map_hashes(tmp_path):
    sparse = tmp_path / "sparse"
    reference = tmp_path / "reference"
    sparse.mkdir()
    reference.mkdir()
    (sparse / "map.pcd").write_bytes(b"sparse map bytes")
    (reference / "map.pcd").write_bytes(b"reference map bytes")
    previous = {
        "sparse_map": {"file_hashes": {"map.pcd": {"sha256": sha256_file(sparse / "map.pcd")}}},
        "old_map": {"file_hashes": {"map.pcd": {"sha256": sha256_file(reference / "map.pcd")}}},
    }
    _check_map_hashes(previous, sparse, reference)
    (sparse / "map.pcd").write_bytes(b"replaced map bytes")
    with pytest.raises(ValueError, match="sparse map PCD hash"):
        _check_map_hashes(previous, sparse, reference)


def test_candidate_polygon_uses_largest_common_support_component():
    # A 2x2 component and one isolated cell; the candidate includes only the 2x2 component.
    old = np.array([[2, 2, 0, 0, 0], [2, 2, 0, 0, 0], [0, 0, 0, 0, 1]])
    sparse = np.array([[3, 4, 0, 0, 0], [2, 2, 0, 0, 0], [0, 0, 0, 0, 2]])
    polygon, cells = _candidate_polygon(old, sparse, np.array([0.0, 0.0]), 1.0, 1, 4)
    assert cells == 4
    assert len(polygon) == 4


@pytest.mark.parametrize(("allow_provisional", "expected"), [
    (False, "allow_provisional_roi_metrics"),
    (True, "greenhouse_roi.yaml is missing"),
])
def test_occupancy_stage_requires_explicit_provisional_authorization_and_roi(tmp_path, allow_provisional, expected):
    output = tmp_path / "output"
    output.mkdir()
    config = tmp_path / "experiment.yaml"
    config.write_text(yaml.safe_dump({"output_dir": str(output), "allow_provisional_roi_metrics": allow_provisional}), encoding="utf-8")

    with pytest.raises(RuntimeError) as error:
        run_stage(config, "occupancy")
    assert expected in str(error.value)


def test_all_stage_refuses_to_overwrite_existing_run_output(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    (output / "roi_validation.json").write_text("{}", encoding="utf-8")
    config = tmp_path / "experiment.yaml"
    config.write_text(yaml.safe_dump({"output_dir": str(output)}), encoding="utf-8")

    with pytest.raises(RuntimeError, match="refusing to overwrite existing all outputs"):
        run_stage(config, "all")
