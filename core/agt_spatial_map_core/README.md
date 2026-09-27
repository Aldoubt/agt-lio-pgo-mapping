# agt_spatial_map_core · Phase 1

Single-session spatial *evidence*, not a cross-session stability probability. This
C++17 package owns the `SpatialVoxelEvidence` and `ConfidenceParameters` contract;
Map Studio and future localization consumers must reuse it instead of defining
incompatible confidence meanings. It does not modify the mapping master map.

- `VoxelKey`: `floor(map_frame_xyz / voxel_size)`, with the existing temporal
  filter's float32 division semantics, including negative coordinates.
- `point_count`: finite patch points within the map-frame voxel.
- `observed_keyframes`: **distinct** nonempty `poses_timed.txt` records observing
  the voxel. Indices are zero-based records, not timestamps.
- `keyframe_span`: `last_keyframe - first_keyframe`, not elapsed seconds.
- `auto_confidence`: single-session repeated-observation evidence; never call
  it `stability_probability` or `P_stable`.
- `geometry_score`: exactly 1.0 in V1; geometry estimation is deferred.

The config in `config/spatial_confidence.yaml` defines:

```
observation_score = 1 - exp(-observed_keyframes / observation_reference)
span_score = min(1, keyframe_span / keyframe_span_reference)
persistence_score = observation_score^alpha * span_score^(1 - alpha)
auto_confidence = persistence_score * geometry_score
```

`AUTO` sets final confidence to auto confidence; `FORCE_HIGH` sets 1,
`FORCE_LOW` sets the configured low value (or the explicitly supplied manual
value), and `IGNORE` sets 0. Automatic recalculation does not change an
existing manual mode/value. Defaults are 0.20 m, N0=4, S0=3, alpha=0.7,
threshold=0.60, low value=0.05. Only one-session evidence is represented.

`SpatialEvidenceBuilder::build(map_package, parameters, &stats)` consumes
body-frame `patches/*.pcd` plus optimized `poses_timed.txt`, transforms each
finite point into the map frame using `T_map_body`, and accumulates map-frame
centroids, point counts, *distinct* keyframe observations, first/last indices,
and scores. It rejects missing, duplicate, or unsafe patch references and
invalid poses. It does not validate artifact checksums on its own: callers
must validate the parent package first (the export CLI does this).

## Offline derivative export

After sourcing ROS Humble and the existing mapping workspace overlay, build
`agt_spatial_map_core` with `colcon` and run:

```sh
ros2 run agt_spatial_map_core agt_spatial_map_export \
  --map-package /absolute/path/to/optimized_pgo/map_package \
  --output-dir /separate/existing-parent/spatial_confidence_v1 \
  --config /path/to/spatial_confidence.yaml
```

A navigation release without `poses_timed.txt` and `patches/` is **not** an
input. The executable invokes the existing
`agt_mapping_artifacts.validation.verify_artifact` twice (before reading and
before publication); it fails closed if the Python package is not in the
sourced overlay. It also rejects nonempty destinations, symlinks, path
traversal, unsafe patches and invalid config values. A sibling staging folder
is atomically renamed to the destination only after all files and checksums
are complete. The original verified parent is never written. A nonempty
existing output is never overwritten; an empty directory may be replaced.

Optional `--manual-overrides /path/to/v1.yaml` has this input schema:

```yaml
schema_version: 1
coordinate_system: voxel_index
voxel_size: 0.2
overrides:
  - {key: [5, -2, 0], mode: FORCE_LOW, value: 0.02}
  - {key: [6, -2, 0], mode: IGNORE}
```

The output contains **exactly** `confidence_voxels.pcd`, `stable_map.pcd`,
`confidence_metadata.yaml`, `manual_overrides.yaml`, and `checksums.sha256`.
The confidence PCD uses map-frame centroid `x y z`, exact-integer float64
`voxel_x y z` (range ±2^53), integer point/observation/first/last/span counts,
observation/persistence/geometry/auto/final float32 scores, and independent
integer override mode/has-value plus float32 manual value. The metadata
supplies field names, formulas, parameter snapshot, source manifest/checksum
SHA-256, geometry=deferred, source counts and stable selection semantics.
`stable_map.pcd` is one map-frame XYZI centroid **per selected voxel**; its
intensity is final confidence for visualization, *not* measured LiDAR
reflectance. `FORCE_HIGH` may include, `FORCE_LOW` and `IGNORE` always exclude,
and `AUTO` uses the configured final-confidence threshold. If no voxel passes,
export fails without publishing: PCL cannot read a zero-point stable PCD.
No online master map or legacy PCD→PGM result is replaced.

## Legacy compatibility boundary

`test_temporal_filter_compat` builds the **unchanged** source of
`agt_pcd2grid_exporter/TemporalPersistenceFilter.cpp` into a regression-only
test target. With identical voxel size, minimum independent observations and
minimum keyframe-index span, the legacy retained *voxel keys and point
counts* match those selected from the new evidence on a rotated/transformed
patch fixture (including many duplicate points within a keyframe). The new
continuous confidence threshold is a different criterion; this test does
**not** assert that arbitrary confidence thresholds are equivalent to the
legacy hard-count/span filter. Neither that filter nor traversability is
modified or redirected to use the new builder.

Phase 1 deliberately does not contain a Qt UI, terrain semantics, ray-carving
statistics, change detection, or cross-session stability estimates.
