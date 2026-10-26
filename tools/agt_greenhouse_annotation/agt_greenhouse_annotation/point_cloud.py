"""Small PCD adapter for offline greenhouse structure analysis."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class NavigationPointCloud:
    """XYZ view consumed by the structure detector; never used for editing."""

    points: np.ndarray

    def xyz(self) -> np.ndarray:
        values = np.asarray(self.points, dtype=np.float64)
        if values.ndim != 2 or values.shape[1] < 3:
            raise ValueError("point cloud must be an N x 3-or-more array")
        return values[:, :3]


def read_pcd_cloud(path: str | Path) -> NavigationPointCloud:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("python3-open3d is required to read the mapping PCD") from exc
    cloud = o3d.io.read_point_cloud(str(Path(path)), remove_nan_points=True,
                                    remove_infinite_points=True)
    points = np.asarray(cloud.points, dtype=np.float64).copy()
    if points.ndim != 2 or points.shape[0] == 0:
        raise ValueError(f"PCD contains no readable XYZ points: {path}")
    return NavigationPointCloud(points)
