"""Derived, conservative two-dimensional clearance; source rasters are read-only."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import shapely
from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
import yaml

from .models import CoverageError, GridMap


MAX_CELLS = 4_000_000
MAX_OBSTACLE_RUNS = 200_000
MAX_ANALYSIS_SAMPLES = 200_000


def validate_grid(grid):
    values = grid.data
    if (not isinstance(values, np.ndarray) or values.ndim != 2 or min(values.shape) <= 0
            or values.size > MAX_CELLS):
        raise CoverageError('INVALID_GRID', f'Grid must contain 1..{MAX_CELLS} cells in two dimensions')
    if (not np.issubdtype(values.dtype, np.number) or not np.isfinite(values).all()
            or not np.isin(values, (-1, 0, 100)).all()):
        raise CoverageError('INVALID_GRID', 'Grid values must be trinary -1, 0 or 100')
    if (not math.isfinite(grid.resolution) or grid.resolution <= 0
            or len(grid.origin) != 3 or not all(math.isfinite(value) for value in grid.origin)):
        raise CoverageError('INVALID_GRID', 'Grid resolution and origin must be finite, resolution positive')
    if not all(math.isfinite(size * grid.resolution) for size in values.shape):
        raise CoverageError('INVALID_GRID', 'Grid metric extent exceeds finite range')
    extent = math.hypot(*(size * grid.resolution for size in values.shape))
    if not math.isfinite(max(abs(grid.origin[0]), abs(grid.origin[1])) + extent):
        raise CoverageError('INVALID_GRID', 'World map extent exceeds finite range')


def _geometry_valid(geometry, name):
    if (geometry is None or not hasattr(geometry, 'geom_type') or geometry.is_empty
            or geometry.geom_type not in ('Polygon', 'MultiPolygon')
            or not geometry.is_valid or not math.isfinite(geometry.area)
            or not np.isfinite(shapely.get_coordinates(geometry)).all()):
        raise CoverageError('INVALID_GEOMETRY', f'{name} must be finite valid polygon geometry')


def _to_world(geometry, grid):
    x, y, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    return affinity.affine_transform(geometry, [c, -s, s, c, x, y])


def _to_local(geometry, grid):
    x, y, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    return affinity.affine_transform(geometry, [c, s, -s, c, -c*x-s*y, s*x-c*y])


def map_polygon(grid):
    # Full data validation belongs at API entry; this geometry helper is called
    # repeatedly by footprint checks and must not scan all cells every time.
    height, width = grid.data.shape
    return _to_world(box(0, 0, width*grid.resolution, height*grid.resolution), grid)


def world_to_grid(x, y, grid):
    if not all(math.isfinite(value) for value in (x, y)):
        raise CoverageError('INVALID_COORDINATE', 'World coordinates must be finite')
    ox, oy, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    return (math.floor((c*(x-ox)+s*(y-oy))/grid.resolution),
            math.floor((-s*(x-ox)+c*(y-oy))/grid.resolution))


def mask_geometry(mask, grid):
    """Merge horizontal cell runs before the union; no per-cell polygon object cache."""
    rectangles = []
    r = grid.resolution
    for row, values in enumerate(mask):
        indices = np.flatnonzero(np.diff(np.pad(values.astype(np.int8), (1, 1))))
        if len(rectangles) + len(indices) // 2 > MAX_OBSTACLE_RUNS:
            raise CoverageError('GRID_COMPLEXITY_LIMIT',
                                'Raster exceeds bounded obstacle-run union; review/resample the map first')
        rectangles.extend(box(a*r, row*r, b*r, (row+1)*r)
                          for a, b in zip(indices[::2], indices[1::2]))
    return _to_world(unary_union(rectangles), grid)


def forbidden_geometry(context):
    grid = context.grid
    if not isinstance(grid.data, np.ndarray):
        validate_grid(grid)
    _geometry_valid(context.boundary, 'field boundary')
    for keepout in context.keepouts:
        _geometry_valid(keepout, 'keepout')
    # Only cache owned immutable arrays. Mutable fixtures/callers deliberately
    # bypass the cache, so editing a cell cannot retain an old collision index.
    cacheable = grid.data.flags.owndata and grid.data.flags.c_contiguous and not grid.data.flags.writeable
    key = (id(grid.data), grid.data.shape, grid.data.dtype.str, grid.resolution, tuple(grid.origin), context.boundary.wkb,
           tuple(item.wkb for item in context.keepouts)) if cacheable else None
    cached = getattr(context, '_coverage_forbidden_cache', None)
    if key is not None and cached is not None and cached[0] == key:
        return cached[1]
    validate_grid(grid)
    obstacles = mask_geometry(grid.data != 0, grid)
    result = unary_union([obstacles, map_polygon(grid).difference(context.boundary), *context.keepouts])
    if key is not None:
        context._coverage_forbidden_cache = (key, result, grid.data)
    return result


def footprint_at(point, robot):
    dx, dy = robot.rear_axle_to_base
    c, s = math.cos(point.yaw), math.sin(point.yaw)
    return Polygon([(point.x+c*(x+dx)-s*(y+dy), point.y+s*(x+dx)+c*(y+dy))
                    for x, y in robot.footprint])


def planning_cells(grid, keepouts=(), boundary=None):
    """Conservative cell intersection: partial keepout cells are not drivable."""
    validate_grid(grid)
    keepouts = tuple(keepouts)
    for item in keepouts:
        _geometry_valid(item, 'keepout')
    if boundary is not None:
        _geometry_valid(boundary, 'boundary')
    result = grid.data.copy()
    obstacle = _to_local(unary_union(list(keepouts)), grid)
    domain = _to_local(boundary, grid) if boundary is not None else None
    height, width = result.shape
    r = grid.resolution
    # Bounded vectorized chunks avoid allocating millions of Python polygons.
    for begin in range(0, result.size, 32768):
        indices = np.arange(begin, min(result.size, begin + 32768))
        ys, xs = indices // width, indices % width
        cells = shapely.box(xs*r, ys*r, (xs+1)*r, (ys+1)*r)
        blocked = shapely.intersects(cells, obstacle)
        if domain is not None:
            blocked |= ~shapely.covers(domain, cells)
        selected = (result[ys, xs] == 0) & blocked
        result[ys[selected], xs[selected]] = 100
    return result


def clearance_field(grid, keepouts=(), boundary=None):
    cells = planning_cells(grid, keepouts, boundary)
    free = cells == 0
    distances = distance_transform_edt(np.pad(free, 1, constant_values=False),
                                       sampling=grid.resolution)[1:-1, 1:-1]
    # EDT measures centre-to-centre; subtract half a cell diagonal for a lower bound
    # to occupied square geometry. This layer never replaces footprint validation.
    distances = np.maximum(0.0, distances-grid.resolution/math.sqrt(2.0))
    distances[~free] = 0.0
    return {'distance_m': distances.astype(np.float32), 'known': grid.data != -1,
            'free': free, 'planning_cells': cells}


def analyze_traversability(context):
    context.check_cancelled()
    layers = clearance_field(context.grid, context.keepouts, context.boundary)
    stats = []
    lateral = [y + context.robot.rear_axle_to_base[1] for _, y in context.robot.footprint]
    half_width = max(abs(min(lateral)), abs(max(lateral))) + context.clearance_margin
    total_samples = 0
    for row in context.rows:
        context.check_cancelled()
        from shapely.geometry import LineString
        line = LineString(row.centerline)
        step = min(context.sample_step, context.grid.resolution)
        required = line.length / step if step > 0 else math.inf
        if not math.isfinite(required) or required > MAX_ANALYSIS_SAMPLES:
            raise CoverageError('SAMPLE_LIMIT_EXCEEDED', 'Row clearance analysis exceeds sample budget')
        count = max(2, math.ceil(required)+1)
        total_samples += count
        if total_samples > MAX_ANALYSIS_SAMPLES:
            raise CoverageError('SAMPLE_LIMIT_EXCEEDED', 'Task clearance analysis exceeds total sample budget')
        values = []
        for i in range(count):
            if i % 256 == 0:
                context.check_cancelled()
            point = line.interpolate(i/(count-1), normalized=True)
            x, y = world_to_grid(point.x, point.y, context.grid)
            if 0 <= y < context.grid.data.shape[0] and 0 <= x < context.grid.data.shape[1]:
                values.append(float(layers['distance_m'][y, x]))
            else:
                values.append(0.0)
        stats.append({'id': row.id, 'length_m': line.length,
                      'minimum_center_clearance_lower_bound_m': min(values),
                      'narrow_screen': min(values) < half_width,
                      'traversability': 'REQUIRES_FOOTPRINT_VALIDATION'})
    return layers, {'frame_id': 'map', 'reference_point': 'rear_axle_center',
                    'distance_kind': 'conservative_2d_unsigned_clearance',
                    'rows': stats, 'unknown_cells': int(np.sum(context.grid.data == -1)),
                    'eligible_for_execution': False,
                    'input_files': context.input_files}


def write_planning_map(context, output_dir):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    cells = planning_cells(context.grid, context.keepouts, context.boundary)
    pixels = np.where(cells < 0, 205, np.where(cells == 0, 254, 0)).astype(np.uint8)
    image_path = output / 'planning.pgm'
    yaml_path = output / 'planning.yaml'
    if image_path.exists() or yaml_path.exists():
        raise CoverageError('OUTPUT_EXISTS', 'Derived planning maps never overwrite existing files')
    context.check_cancelled()
    with image_path.open('xb') as stream:
        Image.fromarray(np.flipud(pixels)).save(stream, format='PPM')
    payload = yaml.safe_dump({
        'image': image_path.name, 'mode': 'trinary', 'resolution': context.grid.resolution,
        'origin': list(context.grid.origin), 'negate': 0, 'occupied_thresh': 0.65,
        'free_thresh': 0.196}, sort_keys=False)
    with yaml_path.open('x', encoding='utf-8') as stream:
        stream.write(payload)
    return yaml_path
