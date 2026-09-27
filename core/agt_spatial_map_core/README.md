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

Phase 1 deliberately does not contain a Qt UI, terrain semantics, ray-carving
statistics, change detection, or cross-session stability estimates.
