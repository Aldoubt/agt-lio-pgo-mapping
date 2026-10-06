"""Map-frame polygons for review clipping and explicit row-evidence exclusions."""
import numpy as np


def validate_polygon(polygon):
    values = np.asarray(polygon, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 3 or not np.isfinite(values).all():
        raise ValueError('区域需要至少三个有限 XY 顶点')
    area = np.sum(values[:, 0]*np.roll(values[:, 1], -1) - values[:, 1]*np.roll(values[:, 0], -1))
    if abs(area) < 1e-8:
        raise ValueError('区域面积不能为零')
    return values


def points_in_polygon(xy, polygon):
    """Ray crossing with boundary included, independent of vertex winding."""
    vertices = validate_polygon(polygon)
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    x, y = xy.T
    inside = np.zeros(len(xy), dtype=bool)
    boundary = inside.copy()
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        dx, dy = b-a
        cross = (x-a[0])*dy - (y-a[1])*dx
        boundary |= ((np.abs(cross) <= 1e-9 * max(1., np.hypot(dx, dy)))
                     & (x >= min(a[0], b[0])-1e-9) & (x <= max(a[0], b[0])+1e-9)
                     & (y >= min(a[1], b[1])-1e-9) & (y <= max(a[1], b[1])+1e-9))
        if abs(dy) > 1e-12:
            inside ^= ((a[1] > y) != (b[1] > y)) & (x < a[0] + (y-a[1])*dx/dy)
    return (inside | boundary) & np.isfinite(xy).all(axis=1)
