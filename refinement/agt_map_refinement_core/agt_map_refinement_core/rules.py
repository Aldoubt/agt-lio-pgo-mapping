from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


REMOVE_TYPES = ('remove_polygon', 'remove_box', 'remove_sphere', 'remove_height_band')
RESTRICTION_TYPES = ('forbidden_zone',)
SUPPORTED_TYPES = REMOVE_TYPES + RESTRICTION_TYPES


@dataclass(frozen=True)
class RefinementRules:
    operations: list[dict[str, Any]]


def _require_xyz(value: Any, label: str) -> None:
    if not isinstance(value, dict) or not all(axis in value for axis in ('x', 'y', 'z')):
        raise ValueError(f'{label} requires x, y and z')


def _validate_z_range(operation: dict[str, Any]) -> None:
    z_range = operation.get('z_range')
    if z_range is None:
        return
    if (not isinstance(z_range, (list, tuple)) or len(z_range) != 2
            or any(not isinstance(v, (int, float)) for v in z_range) or z_range[0] > z_range[1]):
        raise ValueError('z_range must be [min_z, max_z] with min_z <= max_z')


def validate_operation(operation: Any) -> None:
    if not isinstance(operation, dict) or operation.get('type') not in SUPPORTED_TYPES:
        raise ValueError(f'unsupported refinement operation: {operation}')
    kind = operation['type']
    if kind in {'remove_polygon', 'forbidden_zone'}:
        points = operation.get('points', operation.get('polygon'))
        if not isinstance(points, list) or len(points) < 3:
            raise ValueError(f'{kind} requires at least three points')
        if kind == 'remove_polygon':
            _validate_z_range(operation)
    elif kind == 'remove_box':
        if not all(key in operation for key in ('min', 'max')):
            raise ValueError('remove_box requires min and max')
        _require_xyz(operation['min'], 'remove_box.min')
        _require_xyz(operation['max'], 'remove_box.max')
    elif kind == 'remove_sphere':
        _require_xyz(operation.get('center'), 'remove_sphere.center')
        radius = operation.get('radius')
        if not isinstance(radius, (int, float)) or radius <= 0:
            raise ValueError('remove_sphere requires a positive radius')
    elif kind == 'remove_height_band':
        if 'min_z' not in operation or 'max_z' not in operation or operation['min_z'] > operation['max_z']:
            raise ValueError('remove_height_band requires min_z <= max_z')


def load_rules(path) -> RefinementRules:
    import yaml
    from pathlib import Path

    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    if data.get('version', 1) != 1:
        raise ValueError('refinement.yaml version must be 1')
    operations = data.get('operations') or []
    if not isinstance(operations, list):
        raise ValueError('refinement.yaml operations must be a list')
    for operation in operations:
        validate_operation(operation)
    return RefinementRules(operations)


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    for index, (point_x, point_y) in enumerate(polygon):
        next_x, next_y = polygon[(index + 1) % len(polygon)]
        intersects = (point_y > y) != (next_y > y)
        if intersects and x < (next_x - point_x) * (y - point_y) / (next_y - point_y) + point_x:
            inside = not inside
    return inside


def point_is_removed(x: float, y: float, z: float, operations: list[dict[str, Any]]) -> bool:
    for operation in operations:
        kind = operation['type']
        if kind == 'remove_polygon':
            z_range = operation.get('z_range')
            if z_range is not None and not (z_range[0] <= z <= z_range[1]):
                continue
            if point_in_polygon(x, y, operation.get('points', operation.get('polygon'))):
                return True
        elif kind == 'remove_box':
            minimum, maximum = operation['min'], operation['max']
            if (minimum['x'] <= x <= maximum['x'] and minimum['y'] <= y <= maximum['y']
                    and minimum['z'] <= z <= maximum['z']):
                return True
        elif kind == 'remove_sphere':
            center = operation['center']
            if math.dist((x, y, z), (center['x'], center['y'], center['z'])) <= operation['radius']:
                return True
        elif kind == 'remove_height_band':
            if operation['min_z'] <= z <= operation['max_z']:
                return True
    return False


def forbidden_polygons(operations: list[dict[str, Any]]) -> list[list[list[float]]]:
    return [operation.get('polygon', operation.get('points')) for operation in operations
            if operation['type'] == 'forbidden_zone']