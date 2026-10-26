"""Strict offline inputs; existing map and platform files remain authoritative."""
from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import stat

import numpy as np
from PIL import Image
import yaml
from shapely.geometry import LineString, Polygon, shape
from shapely.ops import unary_union

from .models import CoverageError, GridMap, PlanningContext, Pose, RobotProfile, WorkRow


MAX_DOCUMENT_BYTES = 16 * 1024 * 1024
MAX_DOCUMENT_NODES = 250_000
MAX_FEATURES = 4096
MAX_ROWS = 1000
MAX_VERTICES = 200_000
MAX_BOUND_FILES = 4096
MAX_SNAPSHOT_BYTES = 8 * 1024 ** 3


def _object(value, name):
    if not isinstance(value, dict):
        raise CoverageError('INVALID_INPUT', f'{name} must be an object')
    return value


def _path(value, base, name):
    if not isinstance(value, str) or not value.strip() or '\x00' in value:
        raise CoverageError('INVALID_INPUT', f'{name} must be a nonempty file path')
    return (Path(base) / value).resolve()


def _file_state(value):
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)


def _check_document(value, *, max_nodes=MAX_DOCUMENT_NODES):
    """Bound alias expansion and reject recursion/nonfinite values before GEOS."""
    nodes = 0
    active = set()
    seen = set()

    def visit(item, depth):
        nonlocal nodes
        nodes += 1
        if nodes > max_nodes or depth > 64:
            raise CoverageError('INPUT_TOO_COMPLEX', 'Document nesting or element limit exceeded')
        if isinstance(item, float) and not math.isfinite(item):
            raise CoverageError('INVALID_INPUT', 'NaN and infinity are not valid input values')
        if not isinstance(item, (dict, list, tuple)):
            return
        identity = id(item)
        if identity in active:
            raise CoverageError('INVALID_INPUT', 'Recursive YAML aliases are not supported')
        if identity in seen:
            return
        active.add(identity)
        for child in item.values() if isinstance(item, dict) else item:
            visit(child, depth + 1)
        active.remove(identity)
        seen.add(identity)
    visit(value, 0)


class UniqueLoader(getattr(yaml, 'CSafeLoader', yaml.SafeLoader)):
    pass


def _mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise CoverageError('INVALID_INPUT', 'Mapping keys must be unique strings')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def read_document(path, *, max_bytes=MAX_DOCUMENT_BYTES, max_nodes=MAX_DOCUMENT_NODES):
    if (type(max_bytes) is not int or type(max_nodes) is not int
            or max_bytes <= 0 or max_nodes <= 0):
        raise CoverageError('INVALID_INPUT', 'Document resource budgets must be positive integers')
    path = Path(path).resolve()
    try:
        if not stat.S_ISREG(path.stat().st_mode):
            raise CoverageError('INVALID_INPUT', f'Input must be a regular file: {path}')
        with path.open('rb') as stream:
            before = os.fstat(stream.fileno())
            if before.st_size > max_bytes:
                raise CoverageError('INPUT_TOO_LARGE', str(path))
            payload = stream.read(max_bytes + 1)
            after = os.fstat(stream.fileno())
        if len(payload) > max_bytes:
            raise CoverageError('INPUT_TOO_LARGE', str(path))
        if _file_state(before) != _file_state(after) or _file_state(after) != _file_state(path.stat()):
            raise CoverageError('STALE', f'Input changed while reading: {path}')
        value = yaml.load(payload.decode('utf-8'), Loader=UniqueLoader)
    except CoverageError:
        raise
    except (OSError, UnicodeError, yaml.YAMLError, RecursionError, ValueError) as exc:
        raise CoverageError('INVALID_INPUT', f'Cannot read {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise CoverageError('INVALID_INPUT', f'{path}: expected an object')
    _check_document(value, max_nodes=max_nodes)
    return value


def file_digest(path):
    h = hashlib.sha256()
    path = Path(path)
    try:
        if not stat.S_ISREG(path.stat().st_mode):
            raise CoverageError('INVALID_INPUT', f'Input must be a regular file: {path}')
        with path.open('rb') as stream:
            before = os.fstat(stream.fileno())
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(chunk)
            after = os.fstat(stream.fileno())
        if _file_state(before) != _file_state(after) or _file_state(after) != _file_state(path.stat()):
            raise CoverageError('STALE', f'Input changed while hashing: {path}')
    except OSError as exc:
        raise CoverageError('INVALID_INPUT', f'Cannot hash {path}: {exc}') from exc
    return 'sha256:' + h.hexdigest()


def assert_current(input_files):
    for path, expected in input_files.items():
        try:
            current = file_digest(path)
        except CoverageError as exc:
            raise CoverageError('STALE', f'Input changed or disappeared: {path}') from exc
        if current != expected:
            raise CoverageError('STALE', f'Input changed or disappeared: {path}')


def _number(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CoverageError('INVALID_INPUT', f'{name} must be finite')
    try:
        value = float(value)
    except (ValueError, OverflowError) as exc:
        raise CoverageError('INVALID_INPUT', f'{name} must fit finite numeric range') from exc
    if not math.isfinite(value):
        raise CoverageError('INVALID_INPUT', f'{name} must be finite')
    if (positive and value <= 0) or (nonnegative and value < 0):
        raise CoverageError('INVALID_INPUT', f'{name} is out of range')
    return float(value)


def _xy(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise CoverageError('INVALID_INPUT', f'{name} needs two coordinates')
    return tuple(_number(v, name) for v in value)


def _pose(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CoverageError('INVALID_INPUT', f'{name} needs [x, y, yaw] in map')
    return Pose(*(_number(v, name) for v in value))


def load_grid(path, *, input_files=None):
    path = Path(path).resolve()
    snapshot = {str(path): file_digest(path)}
    doc = read_document(path)
    if doc.get('mode', 'trinary') != 'trinary':
        raise CoverageError('UNSUPPORTED_MAP', 'Only Nav2 trinary maps are supported')
    resolution = _number(doc.get('resolution'), 'resolution', positive=True)
    origin = _pose(doc.get('origin'), 'origin')
    image_path = _path(doc.get('image'), path.parent, 'image')
    snapshot[str(image_path)] = file_digest(image_path)
    free = _number(doc.get('free_thresh'), 'free_thresh')
    occupied = _number(doc.get('occupied_thresh'), 'occupied_thresh')
    negate = doc.get('negate', 0)
    if not 0 <= free < occupied <= 1 or type(negate) is not int or negate not in (0, 1):
        raise CoverageError('INVALID_MAP', 'Invalid occupancy thresholds or negate')
    try:
        with Image.open(image_path) as image:
            if image.width * image.height > 4_000_000:
                raise CoverageError('MAP_TOO_LARGE', 'Offline authoring accepts at most 4 million cells')
            rgba = np.asarray(image.convert('RGBA'), dtype=float)
    except (OSError, ValueError, Image.DecompressionBombError) as exc:
        raise CoverageError('INVALID_MAP', f'Cannot decode map image: {exc}') from exc
    brightness = rgba[:, :, :3].mean(axis=2) / 255.0
    probability = brightness if doc.get('negate', 0) else 1.0 - brightness
    cells = np.full(probability.shape, -1, dtype=np.int8)
    cells[probability < free] = 0
    cells[probability > occupied] = 100
    cells[rgba[:, :, 3] < 255] = -1
    cells = np.flipud(cells).copy()
    cells.flags.writeable = False
    grid = GridMap(cells, resolution, (origin.x, origin.y, origin.yaw), str(path), str(image_path))
    from .grid import validate_grid
    validate_grid(grid)
    assert_current(snapshot)
    if input_files is not None:
        for filename, digest in snapshot.items():
            if filename in input_files and input_files[filename] != digest:
                raise CoverageError('STALE', f'Input changed before loading map: {filename}')
            input_files[filename] = digest
    return grid


def load_robot_profile(path):
    """Read the existing platform YAML; no authoring-specific robot schema."""
    path = Path(path).resolve()
    digest = file_digest(path)
    doc = read_document(path)
    p = doc.get('platform')
    if not isinstance(p, dict) or p.get('kinematics') != 'ackermann':
        raise CoverageError('PROFILE_UNSUPPORTED', 'Select an existing Ackermann platform profile')
    g = _object(p.get('geometry', {}), 'platform.geometry')
    limits = _object(p.get('limits', {}), 'platform.limits')
    steering = _object(g.get('steering', {}), 'platform.geometry.steering')
    identity = p.get('name')
    if not isinstance(identity, str) or not identity.strip():
        raise CoverageError('INVALID_PROFILE', 'An existing platform.name is required')
    vertices = g.get('navigation_footprint', g.get('footprint', []))
    if not isinstance(vertices, list) or len(vertices) > 1024:
        raise CoverageError('INVALID_PROFILE', 'Footprint must be a bounded coordinate list')
    footprint = tuple(_xy(v, 'navigation_footprint') for v in
                      vertices)
    polygon = Polygon(footprint) if len(footprint) >= 3 else Polygon()
    if polygon.is_empty or not polygon.is_valid or not math.isfinite(polygon.area) or polygon.area <= 0:
        raise CoverageError('INVALID_PROFILE', 'A simple nonzero navigation footprint is required')
    wheelbase = _number(g.get('wheelbase'), 'wheelbase', positive=True)
    radius = _number(g.get('min_turning_radius'), 'min_turning_radius', positive=True)
    reference = g.get('reference_point_assumption', g.get('reference_point', 'unknown'))
    offset = g.get('rear_axle_to_base')
    reference_verified = g.get('reference_point_verified') is True
    if offset is None:
        if reference == 'rear_axle_center':
            offset = (0.0, 0.0)
        elif reference == 'geometric_center':
            # Existing v2 assumption is usable for previews only, never certification.
            offset = (wheelbase / 2.0, 0.0)
            reference_verified = False
        else:
            raise CoverageError('REFERENCE_POINT_UNKNOWN', 'Profile must identify its axle/base relationship')
    offset = _xy(offset, 'rear_axle_to_base')
    def optional(values, key):
        return None if values.get(key) is None else _number(values[key], key, positive=True)
    angle = optional(steering, 'max_equivalent_steering_angle')
    if angle is not None and angle >= math.pi / 2:
        raise CoverageError('INVALID_PROFILE', 'Equivalent steering angle must be below pi/2')
    assert_current({str(path): digest})
    return RobotProfile(
        platform_id=identity, digest=digest,
        wheelbase=wheelbase, min_turning_radius=radius, footprint=footprint,
        rear_axle_to_base=offset,
        geometry_verified=g.get('dimensions_verified_on_vehicle', g.get('verified', False)) is True,
        reference_verified=reference_verified,
        limits_verified=(limits.get('limits_verified_on_vehicle') is True
                         and 'max_forward_velocity' in limits and 'max_reverse_velocity' in limits),
        max_forward_speed=_number(limits.get('max_forward_velocity', 0.2), 'max_forward_velocity', positive=True),
        max_reverse_speed=_number(limits.get('max_reverse_velocity', 0.1), 'max_reverse_velocity', positive=True),
        max_steering_angle=angle,
        max_steering_rate=optional(steering, 'max_steering_rate'),
        max_acceleration=optional(limits, 'max_linear_acceleration'),
        max_deceleration=optional(limits, 'max_linear_deceleration'))


def _geometry(value, identity):
    value = _object(value, f'{identity}.geometry')
    kind = value.get('type')
    if kind not in ('LineString', 'Polygon', 'MultiPolygon'):
        raise CoverageError('INVALID_GEOMETRY', f'{identity}: unsupported geometry {kind}')
    count = 0

    def coordinates(item, depth):
        nonlocal count
        if not isinstance(item, (list, tuple)) or not item:
            raise CoverageError('INVALID_GEOMETRY', f'{identity}: coordinates must be nonempty lists')
        if depth == 0:
            _xy(item, identity)
            count += 1
            if count > MAX_VERTICES:
                raise CoverageError('INPUT_TOO_COMPLEX', f'{identity}: vertex limit exceeded')
            return
        for child in item:
            coordinates(child, depth - 1)
    coordinates(value.get('coordinates'), {'LineString': 1, 'Polygon': 2, 'MultiPolygon': 3}[kind])
    try:
        geometry = shape(value)
    except (ValueError, TypeError, KeyError) as exc:
        raise CoverageError('INVALID_GEOMETRY', f'{identity}: {exc}') from exc
    if (geometry.is_empty or not geometry.is_valid or not np.isfinite(geometry.bounds).all()
            or not math.isfinite(geometry.area) or not math.isfinite(geometry.length)):
        raise CoverageError('INVALID_GEOMETRY', identity)
    return geometry, count


def load_structure(path):
    doc = read_document(path)
    if doc.get('type') != 'FeatureCollection' or doc.get('frame_id') != 'map':
        raise CoverageError('STRUCTURE_FRAME', 'Confirmed GeoJSON must use metric map coordinates')
    if _object(doc.get('review', {}), 'review').get('status') != 'CONFIRMED':
        raise CoverageError('REVIEW_REQUIRED', 'Confirm extracted structure before planning')
    boundaries, rows, keepouts, work_areas, corridors, headlands, ids = [], [], [], [], [], [], set()
    features = doc.get('features')
    if not isinstance(features, list) or len(features) > MAX_FEATURES:
        raise CoverageError('INVALID_STRUCTURE', f'features must be a list of at most {MAX_FEATURES} objects')
    vertex_count = 0
    for feature in features:
        feature = _object(feature, 'feature')
        if feature.get('type') != 'Feature':
            raise CoverageError('INVALID_STRUCTURE', 'Every entry must be a GeoJSON Feature')
        props = _object(feature.get('properties', {}), 'feature.properties')
        if type(props.get('enabled', True)) is not bool:
            raise CoverageError('INVALID_STRUCTURE', 'enabled must be boolean')
        if props.get('enabled', True) is False:
            continue
        identity = props.get('id', feature.get('id', ''))
        if not isinstance(identity, str) or not identity.strip() or identity in ids:
            raise CoverageError('STRUCTURE_ID', 'Every enabled feature needs a unique ID')
        ids.add(identity)
        if props.get('review_status') != 'CONFIRMED':
            raise CoverageError('REVIEW_REQUIRED', identity)
        geometry, count = _geometry(feature.get('geometry'), identity)
        vertex_count += count
        kind = props.get('feature_type')
        if kind == 'aisle':
            if geometry.geom_type != 'LineString' or not geometry.is_simple or geometry.length <= 0:
                raise CoverageError('INVALID_AISLE', identity)
            coordinates = tuple(_xy(v, identity) for v in geometry.coords)
            if any(a == b for a, b in zip(coordinates, coordinates[1:])):
                raise CoverageError('ZERO_LENGTH_SEGMENT', identity)
            corridor = None
            if 'corridor' in props:
                corridor, count = _geometry(props['corridor'], identity + '.corridor')
                vertex_count += count
            if corridor is not None:
                if corridor.geom_type != 'Polygon' or not corridor.is_valid or not corridor.covers(geometry):
                    raise CoverageError('INVALID_CORRIDOR', identity)
                corridors.append(corridor)
            allowed = props.get('allowed_direction', 'bidirectional')
            if allowed not in ('bidirectional', 'along', 'against'):
                raise CoverageError('INVALID_DIRECTION', identity)
            rows.append(WorkRow(identity, coordinates, corridor,
                                ('along', 'against') if allowed == 'bidirectional' else (allowed,)))
            if len(rows) > MAX_ROWS:
                raise CoverageError('INPUT_TOO_COMPLEX', f'Task exceeds {MAX_ROWS} aisles')
        elif kind in ('field_boundary', 'keepout', 'exclusion_zone', 'work_area', 'headland'):
            if geometry.geom_type not in ('Polygon', 'MultiPolygon'):
                raise CoverageError('INVALID_POLYGON', identity)
            {'field_boundary': boundaries, 'keepout': keepouts, 'exclusion_zone': keepouts,
             'work_area': work_areas, 'headland': headlands}[kind].append(geometry)
        else:
            raise CoverageError('UNSUPPORTED_FEATURE', f'{identity}: {kind}')
        if vertex_count > MAX_VERTICES:
            raise CoverageError('INPUT_TOO_COMPLEX', 'Total structure vertex limit exceeded')
    if len(boundaries) != 1:
        raise CoverageError('BOUNDARY_REQUIRED', 'Exactly one confirmed greenhouse boundary is required')
    boundary = boundaries[0]
    for headland in headlands:
        if not boundary.covers(headland):
            raise CoverageError('HEADLAND_OUTSIDE_BOUNDARY', 'Confirmed headland must lie inside the greenhouse')
    for row in rows:
        if not boundary.covers(LineString(row.centerline)):
            raise CoverageError('AISLE_OUTSIDE_BOUNDARY', row.id)
        if row.corridor is not None and not boundary.covers(row.corridor):
            raise CoverageError('CORRIDOR_OUTSIDE_BOUNDARY', row.id)
    work_area = unary_union(work_areas or corridors)
    if not work_area.is_empty and not boundary.covers(work_area):
        raise CoverageError('WORK_AREA_OUTSIDE_BOUNDARY', 'Do not silently clip the requested work area')
    source = _object(doc.get('source', {}), 'source')
    return boundary, rows, keepouts, work_area, source, headlands


def load_request(path):
    path = Path(path).resolve()
    request_digest = file_digest(path)
    doc = read_document(path)
    allowed = {'map_yaml', 'robot_profile', 'structure', 'mode', 'operation_width_m', 'start', 'end',
               'backend', 'allow_reverse', 'heading_rad', 'clearance_margin_m', 'sample_step_m',
               'connection_timeout_s', 'planning_timeout_s', 'backend_options', 'map_binding',
               'site_root', 'publication'}
    if set(doc) - allowed:
        raise CoverageError('UNKNOWN_OPTION', ', '.join(sorted(set(doc) - allowed)))
    def resolve(value):
        return _path(value, path.parent, 'input')
    files = {}
    total_bytes = 0
    def bind(file):
        nonlocal total_bytes
        file = Path(file).resolve()
        if str(file) not in files:
            try:
                total_bytes += file.stat().st_size
            except OSError as exc:
                raise CoverageError('INVALID_INPUT', f'Cannot read {file}: {exc}') from exc
            if len(files) >= MAX_BOUND_FILES or total_bytes > MAX_SNAPSHOT_BYTES:
                raise CoverageError('INPUT_TOO_LARGE', 'Source snapshot file count or byte limit exceeded')
        digest = file_digest(file)
        if str(file) in files and files[str(file)] != digest:
            raise CoverageError('STALE', f'Input changed while loading request: {file}')
        files[str(file)] = digest
        return file
    files[str(path)] = request_digest
    bind(path)
    map_path = bind(resolve(doc.get('map_yaml')))
    profile_path = bind(resolve(doc.get('robot_profile')))
    structure_path = bind(resolve(doc.get('structure')))
    grid = load_grid(map_path, input_files=files)
    robot = load_robot_profile(profile_path)
    boundary, rows, keepouts, work_area, source, headlands = load_structure(structure_path)
    if source.get('annotation_path'):
        annotation = bind(_path(source['annotation_path'], structure_path.parent, 'source.annotation_path'))
        if files[str(annotation)] != source.get('annotation_sha256'):
            raise CoverageError('STALE', 'Confirmed structure no longer matches its source annotation')
    elif source.get('annotation_sha256'):
        raise CoverageError('INVALID_INPUT', 'An annotation digest requires its source path')
    for key, source_path in (('map_yaml_sha256', grid.yaml_path), ('map_image_sha256', grid.image_path)):
        if key in source and source[key] != files[source_path]:
            raise CoverageError('STALE', f'Confirmed structure {key} does not match selected map')
    mode = doc.get('mode', 'rows')
    if mode not in ('rows', 'area'):
        raise CoverageError('INVALID_MODE', str(mode))
    if mode == 'rows' and not rows:
        raise CoverageError('ROWS_REQUIRED', 'Select at least one aisle')
    width = doc.get('operation_width_m')
    width = None if width is None else _number(width, 'operation_width_m', positive=True)
    if mode == 'area' and (width is None or work_area.is_empty):
        raise CoverageError('AREA_INPUT_REQUIRED', 'Area mode requires width and confirmed work area/corridors')
    start = _pose(doc.get('start'), 'start')
    end = _pose(doc.get('end', doc.get('start')), 'end')
    backend = doc.get('backend', 'ros')
    if backend not in ('ros', 'reference'):
        raise CoverageError('BACKEND_UNSUPPORTED', str(backend))
    reverse = doc.get('allow_reverse', True)
    if type(reverse) is not bool:
        raise CoverageError('INVALID_INPUT', 'allow_reverse must be boolean')
    options = doc.get('backend_options', {})
    if not isinstance(options, dict):
        raise CoverageError('INVALID_INPUT', 'backend_options must be an object')
    site_root = str(resolve(doc['site_root'])) if doc.get('site_root') else ''
    # Site/profile authoritative validation occurs at publication, not on user-supplied hash claims.
    if site_root:
        if not Path(site_root).is_dir():
            raise CoverageError('INVALID_INPUT', 'site_root must be an existing directory')
        for file in Path(site_root).rglob('*'):
            if file.is_file():
                bind(file)
    if doc.get('publication'):
        publication_path = bind(resolve(doc['publication']))
        publication = read_document(publication_path)
        for key in ('site_schema', 'profile_snapshot'):
            if publication.get(key):
                bind(_path(publication[key], publication_path.parent, key))
    binding = _object(doc.get('map_binding', {}), 'map_binding')
    connection_timeout = _number(doc.get('connection_timeout_s', 15), 'connection_timeout_s', positive=True)
    planning_timeout = _number(doc.get('planning_timeout_s', 120), 'planning_timeout_s', positive=True)
    if connection_timeout > 300 or planning_timeout > 3600:
        raise CoverageError('INVALID_TIMEOUT', 'Connection and whole-job budgets are limited to 300 and 3600 seconds')
    sample_step = _number(doc.get('sample_step_m', min(0.05, grid.resolution / 2)), 'sample_step_m', positive=True)
    if sample_step < 1e-6:
        raise CoverageError('INVALID_SAMPLE_STEP', 'sample_step_m must be at least one micrometre')
    context = PlanningContext(
        grid=grid, robot=robot, boundary=boundary, rows=rows, work_area=work_area,
        keepouts=keepouts, start=start, end=end, mode=mode, backend=backend,
        operation_width=width, allow_reverse=reverse,
        heading=None if doc.get('heading_rad') is None else _number(doc['heading_rad'], 'heading_rad'),
        clearance_margin=_number(doc.get('clearance_margin_m', 0.0), 'clearance_margin_m', nonnegative=True),
        sample_step=sample_step,
        connection_timeout=connection_timeout,
        planning_timeout=planning_timeout,
        input_files=files, map_binding=binding, request_path=str(path),
        profile_path=str(profile_path), site_root=site_root, structure_path=str(structure_path),
        backend_options=options, headlands=headlands)
    assert_current(files)
    return context
