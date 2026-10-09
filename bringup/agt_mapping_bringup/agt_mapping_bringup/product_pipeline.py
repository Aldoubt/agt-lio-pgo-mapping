"""Small product composition contract; no ROS import and no new SLAM implementation.

The product config selects a sensor input adapter, an existing LIO frontend
profile and optional *independent* offline processors. This module does not
launch drivers, publish TF, write active_map, or perform review/publication.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .backend_registry import BACKEND_IDS, BackendSelectionError, resolve_backend


class ProductPipelineError(ValueError):
    """An unsupported or inconsistent product configuration."""


@dataclass(frozen=True)
class SensorInput:
    id: str
    lidar_type: str
    imu_type: str
    bag: bool
    live: bool
    note: str


# The catalog describes verified adapters, not arbitrary installed ROS drivers.
# Adding a sensor requires a producer and metadata/deskew/timestamp acceptance.
SENSOR_INPUTS: dict[str, SensorInput] = {
    'mid360_custom': SensorInput(
        'mid360_custom', 'livox_ros_driver2/msg/CustomMsg',
        'sensor_msgs/msg/Imu', True, True,
        'Only currently verified sensor family; driver is owned outside this module.'),
}

# Running a frontend requires *session launch + source export* acceptance.
# The existing backend_registry also contains research profiles; the verified
# operator session must not automatically treat them as runnable.
VERIFIED_SESSION_FRONTENDS = frozenset({'fast_livo2_lio'})

@dataclass(frozen=True)
class Processor:
    id: str
    status: str
    entrypoint: str | None
    description: str


# Modes are intentionally conservative:
# manual: usable by an operator through an existing independent entrypoint;
# research: experimental tools exist but not a reviewed production workflow;
# proposed: no product executor is integrated.
PROCESSORS: dict[str, Processor] = {
    'occupancy_review': Processor(
        'occupancy_review', 'manual', 'scripts/review_mapping_output.sh',
        'PCD to PGM and non-destructive 2D edit/review; run after source export.'),
    'relocalization_study': Processor(
        'relocalization_study', 'manual', 'agt_mapstudio_workflow',
        'Offline Query Set / Top-K / GICP study; does not change runtime localization.'),
    'spatial_confidence': Processor(
        'spatial_confidence', 'research', None,
        'Existing spatial confidence evidence is not a verified generic publisher.'),
    'terrain_traversability': Processor(
        'terrain_traversability', 'research', None,
        'Ground-relative research and raster exports need source-specific acceptance.'),
    'dynamic_cleanup_3d': Processor(
        'dynamic_cleanup_3d', 'proposed', None,
        'No accepted provenance-preserving 3D static-map cleanup executor yet.'),
    'route_authoring': Processor(
        'route_authoring', 'proposed', None,
        'READY Route editor and publication adapter are not yet implemented.'),
    'formal_publish': Processor(
        'formal_publish', 'research', None,
        'Use the existing reviewed Site publisher, not a new publishing schema.'),
}


@dataclass(frozen=True)
class ProductPipeline:
    sensor_id: str
    lidar_topic: str
    imu_topic: str
    frontend_id: str
    processors: tuple[str, ...]
    map_store: str | None
    source_path: str

    def as_dict(self) -> dict[str, Any]:
        """Return a CLI-friendly plan without changing any runtime state."""
        selected = SENSOR_INPUTS[self.sensor_id]
        return {
            'config_path': self.source_path,
            'sensor': {
                'id': selected.id, 'lidar_type': selected.lidar_type,
                'imu_type': selected.imu_type, 'lidar_topic': self.lidar_topic,
                'imu_topic': self.imu_topic, 'live_supported': selected.live,
                'bag_supported': selected.bag,
            },
            'frontend': {
                'backend_id': self.frontend_id,
                'session_status': 'verified' if self.frontend_id in VERIFIED_SESSION_FRONTENDS
                                  else 'not_verified',
            },
            'optional_processors': [
                {
                    'id': name, 'execution': 'manual', 'status': PROCESSORS[name].status,
                    'entrypoint': PROCESSORS[name].entrypoint,
                } for name in self.processors
            ],
            'map_store': self.map_store,
            'note': 'Optional processors are NOT launched by the mapping session.',
        }


def _mapping(value: Any, key: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProductPipelineError(f'{key} must be a YAML mapping')
    return value


def _keys(value: dict[str, Any], allowed: set[str], key: str) -> None:
    unknown = set(value) - allowed
    if unknown:
        raise ProductPipelineError(f'{key} contains unknown keys: {sorted(unknown)}')


def _topic(value: Any, key: str) -> str:
    from .live_source import TOPIC_PATTERN
    if value == 'auto' or (isinstance(value, str) and TOPIC_PATTERN.fullmatch(value)):
        return value
    raise ProductPipelineError(f'{key} must be auto or an absolute ROS topic: {value!r}')


def load_product_pipeline(path: str | Path) -> ProductPipeline:
    """Validate a lightweight operator YAML without probing ROS or devices.

    The YAML is NOT a replacement for backend source hashes, robot extrinsics,
    Site 1.0, READY Route, or the immutable map artifact manifests.
    """
    source = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise ProductPipelineError(f'cannot read product pipeline {source}: {exc}') from exc
    document = _mapping(document, 'root')
    _keys(document, {'schema_version', 'sensor', 'frontend', 'processors', 'map_store'}, 'root')
    if type(document.get('schema_version')) is not int or document['schema_version'] != 1:
        raise ProductPipelineError('schema_version must be exactly 1')

    sensor = _mapping(document.get('sensor'), 'sensor')
    _keys(sensor, {'id', 'lidar_topic', 'imu_topic'}, 'sensor')
    sensor_id = sensor.get('id')
    if sensor_id not in SENSOR_INPUTS:
        raise ProductPipelineError(
            f'sensor {sensor_id!r} has no verified input adapter; '
            'add a tested driver/topic/format adapter first')
    lidar_topic = _topic(sensor.get('lidar_topic', 'auto'), 'sensor.lidar_topic')
    imu_topic = _topic(sensor.get('imu_topic', 'auto'), 'sensor.imu_topic')
    if lidar_topic != 'auto' and lidar_topic == imu_topic:
        raise ProductPipelineError('lidar and IMU topics must differ')

    frontend = _mapping(document.get('frontend'), 'frontend')
    _keys(frontend, {'backend_id'}, 'frontend')
    frontend_id = frontend.get('backend_id')
    if frontend_id not in BACKEND_IDS:
        raise ProductPipelineError(f'unknown frontend backend_id: {frontend_id!r}')
    if frontend_id not in VERIFIED_SESSION_FRONTENDS:
        raise ProductPipelineError(
            f'frontend {frontend_id!r} is registered but lacks a verified operator '
            'session/paired-source exporter; explicitly implement and test an adapter')
    try:
        # Reuse the existing pinned source/profile contract; do not duplicate it.
        resolve_backend(frontend_id, check_sources=False)
    except BackendSelectionError as exc:
        raise ProductPipelineError(str(exc)) from exc

    processors = document.get('processors', {})
    processors = _mapping(processors, 'processors')
    _keys(processors, set(PROCESSORS), 'processors')
    enabled = []
    for name, mode in processors.items():
        if mode not in ('disabled', 'manual'):
            raise ProductPipelineError(
                f'processors.{name}: only disabled/manual are allowed; '
                'implicit post-processing is not yet supported')
        if mode == 'manual':
            item = PROCESSORS[name]
            if item.status != 'manual' or not item.entrypoint:
                raise ProductPipelineError(
                    f'processors.{name} is {item.status}: no accepted manual product entrypoint')
            enabled.append(name)

    store = document.get('map_store')
    if store is not None:
        store = _mapping(store, 'map_store')
        _keys(store, {'root'}, 'map_store')
        root = store.get('root')
        if not isinstance(root, str) or not root.strip():
            raise ProductPipelineError('map_store.root must be a nonempty directory string')
        # This is a reference only, not a request to create/activate a map.
        map_store = str((source.parent / root).resolve()) if not Path(root).is_absolute() else str(
            Path(root).expanduser().resolve())
    else:
        map_store = None
    return ProductPipeline(
        sensor_id=str(sensor_id), lidar_topic=lidar_topic, imu_topic=imu_topic,
        frontend_id=str(frontend_id), processors=tuple(sorted(enabled)),
        map_store=map_store, source_path=str(source))


def capability_report() -> dict[str, Any]:
    """Expose registered versus truly runnable backends without starting ROS."""
    return {
        'sensor_inputs': [
            {'id': s.id, 'lidar_type': s.lidar_type, 'imu_type': s.imu_type,
             'bag': s.bag, 'live': s.live, 'note': s.note}
            for s in SENSOR_INPUTS.values()
        ],
        'frontends': [
            {'id': name, 'session_status': 'verified' if name in VERIFIED_SESSION_FRONTENDS
             else 'registered_not_verified'}
            for name in sorted(BACKEND_IDS)
        ],
        'processors': [
            {'id': p.id, 'status': p.status, 'entrypoint': p.entrypoint,
             'description': p.description}
            for p in PROCESSORS.values()
        ],
        'backend_authority': 'agt_mapping_bringup.backend_registry.resolve_backend',
        'no_automatic_fallback': True,
    }


def apply_to_cli(args: Any, product: ProductPipeline) -> None:
    """Apply explicit product selection without surprising CLI overrides."""
    if args.backend is not None and args.backend != product.frontend_id:
        raise ProductPipelineError(
            f'--backend {args.backend!r} conflicts with product frontend {product.frontend_id!r}')
    args.backend = product.frontend_id
    for name, configured in (('lidar_topic', product.lidar_topic),
                             ('imu_topic', product.imu_topic)):
        existing = getattr(args, name)
        if existing != 'auto' and configured != 'auto' and existing != configured:
            raise ProductPipelineError(f'--{name.replace("_", "-")} conflicts with product sensor')
        if existing == 'auto':
            setattr(args, name, configured)
    if args.live and not SENSOR_INPUTS[product.sensor_id].live:
        raise ProductPipelineError(f'sensor {product.sensor_id} has no accepted live owner')
