from __future__ import annotations

import math
from pathlib import Path

from .pcd import AsciiPcd
from .rules import forbidden_polygons, point_in_polygon


def write_nav_map(pcd: AsciiPcd, operations: list[dict], output_pgm: Path, output_yaml: Path,
                  resolution: float = 0.05, min_z: float = -0.2, max_z: float = 2.0) -> None:
    points = [(float(row[pcd.x_index]), float(row[pcd.y_index]), float(row[pcd.z_index]))
              for row in pcd.rows]
    points = [point for point in points if min_z <= point[2] <= max_z]
    polygons = forbidden_polygons(operations)
    if not points and not polygons:
        raise ValueError('cannot generate navigation map from no points and no forbidden zones')
    xs = [point[0] for point in points] or [point[0] for polygon in polygons for point in polygon]
    ys = [point[1] for point in points] or [point[1] for polygon in polygons for point in polygon]
    origin_x = math.floor(min(xs) / resolution) * resolution
    origin_y = math.floor(min(ys) / resolution) * resolution
    width = max(1, math.ceil((max(xs) - origin_x) / resolution) + 1)
    height = max(1, math.ceil((max(ys) - origin_y) / resolution) + 1)
    occupied = set()
    for x, y, _ in points:
        occupied.add((int(math.floor((x - origin_x) / resolution)),
                      int(math.floor((y - origin_y) / resolution))))
    forbidden = set()
    for row in range(height):
        for column in range(width):
            x = origin_x + (column + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution
            if any(point_in_polygon(x, y, polygon) for polygon in polygons):
                forbidden.add((column, row))
    pixels = []
    for row in reversed(range(height)):
        pixels.extend(100 if (column, row) in forbidden or (column, row) in occupied else 254
                      for column in range(width))
        pixels.append(10)
    output_pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes(pixels))
    import yaml
    output_yaml.write_text(yaml.safe_dump({
        'image': output_pgm.name,
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8')