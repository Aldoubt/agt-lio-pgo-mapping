from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RefinementRules:
    operations: list[dict[str, Any]]


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
        if not isinstance(operation, dict) or operation.get('type') not in {
            'remove_polygon', 'remove_box', 'forbidden_zone'}:
            raise ValueError(f'unsupported refinement operation: {operation}')
        if operation['type'] in {'remove_polygon', 'forbidden_zone'}:
            points = operation.get('points', operation.get('polygon'))
            if not isinstance(points, list) or len(points) < 3:
                raise ValueError(f'{operation["type"]} requires at least three points')
        if operation['type'] == 'remove_box' and not all(
                key in operation for key in ('min', 'max')):
            raise ValueError('remove_box requires min and max')
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
        if operation['type'] == 'remove_polygon':
            if point_in_polygon(x, y, operation['points']):
                return True
        elif operation['type'] == 'remove_box':
            minimum, maximum = operation['min'], operation['max']
            if (minimum['x'] <= x <= maximum['x'] and minimum['y'] <= y <= maximum['y']
                    and minimum['z'] <= z <= maximum['z']):
                return True
    return False


def forbidden_polygons(operations: list[dict[str, Any]]) -> list[list[list[float]]]:
    return [operation.get('polygon', operation.get('points')) for operation in operations
            if operation['type'] == 'forbidden_zone']