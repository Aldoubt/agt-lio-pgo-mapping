import numpy as np

from agt_greenhouse_annotation import FREE, GroundRelativeNavigationConfig, NavigationMapResult
from agt_greenhouse_annotation.terrain_morphology import (
    TerrainMorphologyConfig,
    derive_terrain_morphology,
)


def _result(*, width=100, height=80, resolution=0.10):
    columns = np.arange(width, dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)
    xx, yy = np.meshgrid((columns + 0.5) * resolution, (rows + 0.5) * resolution)
    ground = 0.01 * xx
    valid = np.ones((height, width), dtype=bool)
    return NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground,
        ground_valid=valid,
        point_count=np.full((height, width), 8, dtype=np.int32),
        ground_support_count=np.full((height, width), 6, dtype=np.int32),
        obstacle_count=np.zeros((height, width), dtype=np.int32),
        slope_deg=np.zeros_like(ground),
        step_m=np.zeros_like(ground),
        occupancy=np.full((height, width), FREE, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(resolution_m=resolution),
    )


def _config():
    return TerrainMorphologyConfig(
        background_sigma_m=0.45,
        ridge_scale_m=0.08,
        depression_scale_m=0.08,
        step_scale_m=0.10,
        minimum_ground_confidence=0.20,
    )


def test_flat_scene_has_negligible_signed_relief_and_step_evidence():
    result = _result()
    morphology = derive_terrain_morphology(
        result, np.ones(result.ground_height_m.shape), _config()
    )
    interior = np.s_[8:-8, 8:-8]
    assert np.nanmedian(np.abs(morphology.signed_relief_m[interior])) < 0.01
    assert np.nanpercentile(morphology.ridge_evidence[interior], 95) < 0.10
    assert np.nanpercentile(morphology.depression_evidence[interior], 95) < 0.10
    assert np.nanpercentile(morphology.step_evidence[interior], 95) < 0.10


def test_positive_ridge_and_negative_depression_are_separated():
    result = _result()
    yy = (np.arange(result.height, dtype=np.float64) + 0.5) * result.resolution_m
    ridge_y = 2.5
    depression_y = 5.5
    ridge = 0.16 * np.exp(-((yy - ridge_y) ** 2) / (2.0 * 0.14**2))
    depression = -0.14 * np.exp(-((yy - depression_y) ** 2) / (2.0 * 0.18**2))
    result.ground_height_m[:] += ridge[:, None] + depression[:, None]

    morphology = derive_terrain_morphology(
        result, np.ones(result.ground_height_m.shape), _config()
    )
    ridge_row = int(round(ridge_y / result.resolution_m - 0.5))
    depression_row = int(round(depression_y / result.resolution_m - 0.5))
    assert float(np.median(morphology.ridge_evidence[ridge_row, 15:-15])) > 0.55
    assert float(np.median(morphology.depression_evidence[ridge_row, 15:-15])) < 0.10
    assert float(np.median(morphology.depression_evidence[depression_row, 15:-15])) > 0.45
    assert float(np.median(morphology.ridge_evidence[depression_row, 15:-15])) < 0.10


def test_abrupt_ground_step_creates_local_step_evidence():
    result = _result()
    split = result.width // 2
    result.ground_height_m[:, split:] += 0.18
    morphology = derive_terrain_morphology(
        result, np.ones(result.ground_height_m.shape), _config()
    )
    near = morphology.step_evidence[10:-10, split - 1 : split + 2]
    far = morphology.step_evidence[10:-10, 10:20]
    assert float(np.median(near)) > 0.45
    assert float(np.nanpercentile(far, 95)) < 0.15


def test_invalid_or_low_confidence_ground_has_zero_morphology_evidence():
    result = _result()
    result.ground_valid[20:35, 25:45] = False
    confidence = np.ones(result.ground_height_m.shape)
    confidence[45:60, 50:75] = 0.05
    morphology = derive_terrain_morphology(result, confidence, _config())
    assert not np.any(morphology.valid_mask[20:35, 25:45])
    assert not np.any(morphology.valid_mask[45:60, 50:75])
    for layer in (
        morphology.ridge_evidence,
        morphology.depression_evidence,
        morphology.step_evidence,
    ):
        assert np.all(layer[20:35, 25:45] == 0.0)
        assert np.all(layer[45:60, 50:75] == 0.0)
