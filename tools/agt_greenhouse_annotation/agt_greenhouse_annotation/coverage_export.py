"""Explicit adapter from reviewed topology to the existing coverage GeoJSON input.

Crop-row references remain evidence. Only separately reviewed aisle corridors
are exported as travel centerlines; this module does not plan or publish Routes.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile

from shapely import affinity
from shapely.geometry import LineString, Polygon, mapping, shape

from .structure_artifacts import freeze_accepted_proposals, frozen_proposal_document
from .validator import validate_topology


def reviewed_corridor(centerline, width_m, allowed_direction, *, confirmed=False):
    """Record an operator-reviewed constant-width corridor, never infer approval."""
    if confirmed is not True:
        raise ValueError('Traversable aisle corridor needs explicit manual review')
    if allowed_direction not in ('bidirectional', 'along', 'against'):
        raise ValueError('Select an explicit allowed aisle direction')
    width = float(width_m)
    line = LineString(centerline)
    if (not math.isfinite(width) or width <= 0 or line.is_empty or
            not line.is_valid or not line.is_simple or line.length <= 0):
        raise ValueError('Corridor needs finite positive width and a valid centerline')
    corridor = line.buffer(width / 2, cap_style=2, join_style=2)
    if corridor.geom_type != 'Polygon' or not corridor.is_valid:
        raise ValueError('Review a single valid corridor polygon')
    return {'status': 'CONFIRMED', 'semantic_role': 'traversable_aisle',
            'allowed_direction': allowed_direction, 'corridor': mapping(corridor),
            'width_m': width, 'width_model': 'operator_reviewed_constant_width',
            'centerline_xy': [list(point) for point in line.coords]}


def structure_document(topology, *, map_from_reference=None,
                       transform_review_confirmed=False, headland_ids=()):
    """Convert reviewed geometry; validation is delegated to coverage IO below."""
    annotation = topology.get('annotation', {})
    if annotation.get('status') != 'frozen' or annotation.get('manual_review_confirmed') is not True:
        raise ValueError('Freeze explicitly reviewed topology before coverage export')
    frame = topology.get('source', {}).get('map_frame')
    if not isinstance(frame, str) or not frame:
        raise ValueError('Source coordinate frame is required')
    transform = None
    if map_from_reference is not None:
        try:
            transform = [float(v) for v in map_from_reference]
        except (ValueError, TypeError) as exc:
            raise ValueError('Map transform must be finite [dx, dy, yaw]') from exc
        if len(transform) != 3 or not all(math.isfinite(v) for v in transform):
            raise ValueError('Map transform must be finite [dx, dy, yaw]')
        if transform_review_confirmed is not True:
            raise ValueError('Map transform requires explicit manual review')
        if frame == 'map' and any(v != 0 for v in transform):
            raise ValueError('Already-map coordinates cannot receive a nonidentity transform')
    elif frame != 'map':
        raise ValueError('Non-map source requires an explicit reviewed map transform')

    features = []

    def converted(geometry):
        if transform is not None:
            dx, dy, yaw = transform
            geometry = affinity.rotate(geometry, yaw, origin=(0, 0), use_radians=True)
            geometry = affinity.translate(geometry, xoff=dx, yoff=dy)
        return mapping(geometry)

    def add(identity, kind, geometry, **properties):
        if not isinstance(identity, str) or not identity.strip():
            raise ValueError(f'{kind} needs an explicit reviewed identity')
        features.append({'type': 'Feature', 'geometry': converted(geometry),
                         'properties': {'id': identity, 'feature_type': kind,
                                        'review_status': 'CONFIRMED',
                                        'source_annotation_id': identity, **properties}})

    boundary = topology.get('coverage_boundary', {})
    if boundary.get('confidence') != 'confirmed':
        raise ValueError('Draw and confirm one greenhouse boundary for coverage')
    add(boundary.get('id'), 'field_boundary', Polygon(boundary.get('polygon', [])))
    for aisle in topology.get('aisles', []):
        review = aisle.get('diagnostics', {}).get('coverage_review', {})
        if (aisle.get('confidence') != 'confirmed' or review.get('status') != 'CONFIRMED'
                or review.get('semantic_role') != 'traversable_aisle'):
            raise ValueError(f'{aisle.get("id")}: review traversable corridor and direction explicitly')
        line = LineString(aisle.get('centerline', []))
        if [list(point) for point in line.coords] != review.get('centerline_xy'):
            raise ValueError(f'{aisle.get("id")}: corridor review is stale after centerline edits')
        corridor = shape(review.get('corridor', {}))
        add(aisle.get('id'), 'aisle', line,
            corridor=converted(corridor), allowed_direction=review.get('allowed_direction'))
    # topology.rows may describe crop ridges. Never treat them as drive paths.
    if not any(feature['properties']['feature_type'] == 'aisle' for feature in features):
        raise ValueError('No explicitly reviewed traversable aisles; crop-row references cannot be drive paths')
    selected = list(headland_ids)
    if len(selected) != len(set(selected)):
        raise ValueError('Duplicate selected headland identity')
    by_id = {item['id']: item for item in topology.get('headlands', [])}
    for identity in selected:
        item = by_id.get(identity)
        if item is None or item.get('confidence') != 'confirmed':
            raise ValueError(f'{identity}: selected headland must be manually confirmed')
        add(identity, 'headland', Polygon(item['polygon']))
    source = {'original_frame': frame}
    if transform is not None:
        source.update(map_from_reference=transform, transform_review='CONFIRMED')
    if selected:
        source['headland_annotation_ids'] = selected
    return {'type': 'FeatureCollection', 'frame_id': 'map',
            'review': {'status': 'CONFIRMED'}, 'features': features, 'source': source}


def _coverage_io():
    try:
        from agt_coverage_authoring import io
    except ImportError as exc:
        raise ValueError('Install agt_coverage_authoring to validate/export coverage structure') from exc
    return io


def _validate_document(document):
    io = _coverage_io()
    with tempfile.TemporaryDirectory(prefix='agt-coverage-structure-check-') as directory:
        candidate = Path(directory) / 'structure.geojson'
        candidate.write_text(json.dumps(document, allow_nan=False), encoding='utf-8')
        io.load_structure(candidate)


def _destination(path, *, protected_roots=(), input_files=()):
    """Reject writes into immutable inputs, including symlink aliases."""
    requested = Path(path).expanduser().absolute()
    if requested.is_symlink() or requested.exists():
        raise FileExistsError('Exports are immutable; choose a new filename, not an existing path or symlink')
    resolved = requested.resolve()
    if resolved in {Path(file).resolve() for file in input_files}:
        raise ValueError('Output cannot replace an input')
    for root in protected_roots:
        if resolved.is_relative_to(Path(root).resolve()):
            raise ValueError('Output must be outside immutable source and frozen annotation directories')
    # A frozen topology owns its directory and fixed-name sidecars. This also
    # rejects writing a new nested version underneath an older frozen version.
    io = _coverage_io()
    for parent in resolved.parents:
        for manifest_path in parent.glob('*.manifest.json'):
            document = io.read_document(manifest_path)
            if document.get('annotation', {}).get('status') == 'frozen':
                raise ValueError('Output must be outside existing frozen annotation directories')
    return resolved


def export_coverage_structure(topology_path, map_yaml, output, *, package=None,
                              expected_inputs=None, **options):
    """Export a new GeoJSON bound to the frozen annotation and selected map bytes."""
    io = _coverage_io()
    topology_path, map_yaml = (Path(p).expanduser().resolve() for p in (topology_path, map_yaml))
    snapshot = dict(expected_inputs or {})
    io.assert_current(snapshot)
    snapshot[str(topology_path)] = io.file_digest(topology_path)
    topology = io.read_document(topology_path)
    manifest_path = topology_path.with_suffix('.manifest.json')
    snapshot[str(manifest_path)] = io.file_digest(manifest_path)
    manifest = io.read_document(manifest_path)
    for name in manifest.get('sidecar_hashes', {}):
        sidecar = (topology_path.parent / name).resolve()
        if not sidecar.is_relative_to(topology_path.parent):
            raise ValueError('Annotation sidecar must stay inside its version directory')
        snapshot[str(sidecar)] = io.file_digest(sidecar)
    grid = io.load_grid(map_yaml, input_files=snapshot)
    source_root = topology.get('source', {}).get('map_package')
    if not isinstance(source_root, str) or not source_root.strip():
        raise ValueError('Frozen annotation must reference its source map package')
    output = _destination(output, protected_roots=(source_root, topology_path.parent),
                          input_files=snapshot)
    validation = validate_topology(topology, package, topology_path,
                                   topology_path.parent / 'keyframe_topology_labels.csv',
                                   verify_map_files=True)
    if not validation['valid']:
        raise ValueError('Frozen source validation failed: ' + '; '.join(validation['errors']))
    document = structure_document(topology, **options)
    document['source'].update(annotation_path=str(topology_path),
                              annotation_sha256=snapshot[str(topology_path)],
                              map_yaml_sha256=snapshot[str(map_yaml)],
                              map_image_sha256=snapshot[grid.image_path])
    _validate_document(document)
    io.assert_current(snapshot)
    content = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.coverage-', suffix='.tmp', dir=output.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(content)
        io.assert_current(snapshot)
        os.link(temporary, output)  # Atomic no-replace publication, including racing writers.
    finally:
        Path(temporary).unlink(missing_ok=True)
    return output


def freeze_and_export_coverage(topology, rows, aisles, topology_path, map_yaml, output,
                               *, package, **options):
    """Preflight first, then use the existing immutable topology freeze/export."""
    rows, aisles = list(rows), list(aisles)
    io = _coverage_io()
    snapshot = {}
    io.load_grid(Path(map_yaml), input_files=snapshot)
    topology_path = _destination(topology_path, protected_roots=(package.path,), input_files=snapshot)
    output = _destination(output, protected_roots=(package.path, topology_path.parent), input_files=snapshot)
    candidate = frozen_proposal_document(topology, rows, aisles)
    document = structure_document(candidate, **options)
    _validate_document(document)
    io.assert_current(snapshot)
    freeze_accepted_proposals(topology, rows, aisles, topology_path, package=package)
    return export_coverage_structure(topology_path, map_yaml, output, package=package,
                                     expected_inputs=snapshot, **options)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topology', required=True, help='Existing frozen reviewed topology YAML')
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-from-reference', type=float, nargs=3, metavar=('DX', 'DY', 'YAW'))
    parser.add_argument('--confirm-transform-review', action='store_true')
    parser.add_argument('--headland', action='append', default=[], help='Explicit confirmed headland ID')
    args = parser.parse_args(argv)
    try:
        path = export_coverage_structure(args.topology, args.map_yaml, args.output,
            map_from_reference=args.map_from_reference,
            transform_review_confirmed=args.confirm_transform_review, headland_ids=args.headland)
    except (ValueError, OSError) as exc:
        parser.exit(2, str(exc) + '\n')
    print(path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
