"""Shared mathematical and provenance helpers for cross-stage analysis."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Iterable

import numpy as np


def validate_se3(value: Iterable[Iterable[float]], tolerance: float = 1e-6) -> dict:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("transform must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    determinant = float(np.linalg.det(rotation))
    bottom_row_error = float(np.max(np.abs(matrix[3] - np.array([0., 0., 0., 1.]))))
    if orthogonality_error > tolerance:
        raise ValueError(f"rotation is not orthonormal: error={orthogonality_error:.3g}")
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(f"rotation determinant must be +1: det={determinant:.9g}")
    if bottom_row_error > tolerance:
        raise ValueError(f"invalid homogeneous bottom row: error={bottom_row_error:.3g}")
    return {
        "valid": True,
        "orthogonality_error_fro": orthogonality_error,
        "rotation_determinant": determinant,
        "bottom_row_max_error": bottom_row_error,
        "tolerance": float(tolerance),
    }


def invert_se3(value: Iterable[Iterable[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    validate_se3(matrix)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return inverse


def transform_xyz(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    matrix = np.asarray(transform, dtype=np.float64)
    validate_se3(matrix)
    if points.ndim != 2 or points.shape[1] < 3 or not np.isfinite(points[:, :3]).all():
        raise ValueError("points must be finite Nx3 or NxM data")
    result = points.copy()
    result[:, :3] = points[:, :3] @ matrix[:3, :3].T + matrix[:3, 3]
    return result


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_bounds(points: np.ndarray, percentiles: tuple[float, float] = (1., 99.)) -> dict:
    xyz = np.asarray(points, dtype=np.float64)[:, :3]
    if not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError("cannot calculate bounds from empty or nonfinite points")
    low, high = np.percentile(xyz, percentiles, axis=0)
    return {
        "percentiles": [float(percentiles[0]), float(percentiles[1])],
        "xyz_low_m": low.tolist(),
        "xyz_high_m": high.tolist(),
        "xyz_extent_m": (high - low).tolist(),
    }
