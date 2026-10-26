"""Ground-surface morphology evidence for agricultural map review.

The outputs are geometric evidence only.  They deliberately do not assign
semantic object labels such as crop row, post, step, or rut.  Higher-level
agricultural structure code can combine these layers with obstacle evidence and
a verified agricultural frame.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .navigation_map_derivation import NavigationMapResult


@dataclass(frozen=True)
class TerrainMorphologyConfig:
    background_sigma_m: float = 0.45
    ridge_scale_m: float = 0.08
    depression_scale_m: float = 0.08
    step_scale_m: float = 0.10
    minimum_ground_confidence: float = 0.20

    def validate(self) -> None:
        if self.background_sigma_m <= 0.0:
            raise ValueError("background_sigma_m must be > 0")
        if self.ridge_scale_m <= 0.0:
            raise ValueError("ridge_scale_m must be > 0")
        if self.depression_scale_m <= 0.0:
            raise ValueError("depression_scale_m must be > 0")
        if self.step_scale_m <= 0.0:
            raise ValueError("step_scale_m must be > 0")
        if not 0.0 <= self.minimum_ground_confidence <= 1.0:
            raise ValueError("minimum_ground_confidence must be in [0, 1]")


@dataclass(frozen=True)
class TerrainMorphologyResult:
    background_height_m: np.ndarray
    signed_relief_m: np.ndarray
    ridge_evidence: np.ndarray
    depression_evidence: np.ndarray
    step_evidence: np.ndarray
    valid_mask: np.ndarray
    config: TerrainMorphologyConfig


def _require_ndimage():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("terrain morphology analysis requires scipy") from exc
    return ndimage


def derive_terrain_morphology(
    navigation: NavigationMapResult,
    ground_confidence: np.ndarray,
    config: TerrainMorphologyConfig | None = None,
) -> TerrainMorphologyResult:
    """Derive signed relief and abrupt-height evidence from the ground surface.

    A confidence-gated Gaussian background separates slowly varying terrain from
    local raised/depressed relief.  Step evidence is the magnitude of a local
    one-cell height change over the filled ground surface.  All evidence layers
    are multiplied by ground confidence and are zero outside the valid mask.
    """

    cfg = config or TerrainMorphologyConfig()
    cfg.validate()
    ndimage = _require_ndimage()

    z = np.asarray(navigation.ground_height_m, dtype=np.float64)
    confidence = np.asarray(ground_confidence, dtype=np.float64)
    if confidence.shape != z.shape:
        raise ValueError("ground_confidence shape must match ground_height_m")

    valid = (
        np.asarray(navigation.ground_valid, dtype=bool)
        & np.isfinite(z)
        & np.isfinite(confidence)
        & (confidence >= float(cfg.minimum_ground_confidence))
    )

    zeros = np.zeros(z.shape, dtype=np.float64)
    background = np.full(z.shape, np.nan, dtype=np.float64)
    signed_relief = np.full(z.shape, np.nan, dtype=np.float64)
    if not np.any(valid):
        return TerrainMorphologyResult(
            background_height_m=background,
            signed_relief_m=signed_relief,
            ridge_evidence=zeros.copy(),
            depression_evidence=zeros.copy(),
            step_evidence=zeros.copy(),
            valid_mask=valid,
            config=cfg,
        )

    _, nearest = ndimage.distance_transform_edt(~valid, return_indices=True)
    filled = z[tuple(nearest)]
    sigma_cells = max(1.0, float(cfg.background_sigma_m) / navigation.resolution_m)
    smooth_background = ndimage.gaussian_filter(
        filled, sigma=sigma_cells, mode="nearest"
    )
    background[valid] = smooth_background[valid]
    signed_relief[valid] = z[valid] - smooth_background[valid]

    ridge = zeros.copy()
    depression = zeros.copy()
    ridge[valid] = np.clip(
        np.maximum(signed_relief[valid], 0.0) / float(cfg.ridge_scale_m),
        0.0,
        1.0,
    )
    depression[valid] = np.clip(
        np.maximum(-signed_relief[valid], 0.0) / float(cfg.depression_scale_m),
        0.0,
        1.0,
    )

    # np.gradient here is intentionally in metres-per-grid-cell rather than
    # slope units.  That makes the scale parameter an intuitive local height
    # discontinuity threshold and keeps this layer distinct from robust slope.
    grad_y, grad_x = np.gradient(filled)
    local_height_change = np.hypot(grad_x, grad_y)
    step = zeros.copy()
    step[valid] = np.clip(
        local_height_change[valid] / float(cfg.step_scale_m), 0.0, 1.0
    )

    confidence_clip = np.clip(confidence, 0.0, 1.0)
    ridge *= confidence_clip
    depression *= confidence_clip
    step *= confidence_clip
    ridge[~valid] = 0.0
    depression[~valid] = 0.0
    step[~valid] = 0.0

    return TerrainMorphologyResult(
        background_height_m=background,
        signed_relief_m=signed_relief,
        ridge_evidence=ridge,
        depression_evidence=depression,
        step_evidence=step,
        valid_mask=valid,
        config=cfg,
    )
