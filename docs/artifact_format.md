# Mapping Artifact Format v0.1

## Directory

```text
output/map_package/
├── map.pcd
├── poses.txt
├── poses_timed.txt
├── patches/
├── metadata.yaml
├── calibration.yaml
├── manifest.yaml
└── checksums.sha256
```

`checksums.sha256` contains SHA-256 digests for every artifact file except itself. `manifest.yaml` contains the corresponding digest map for provenance.

## Pose semantics

`frontend_pose` means `T_local_mapping_body`: direct LIO odometry before backend optimization. `optimized_map_pose` means `T_map_mapping_body`: a pose after PGO correction. These are distinct quantities and must never be substituted for one another.

In Commit 5, external PGO correction is not yet connected to the backend contract. The exporter records:

```yaml
backend_status:
  optimized: false
```

and describes the exported pose as frontend-derived. A consumer must reject such an artifact where an optimized global map is required.

## Dense-map limitation

The v0.1 backend contract carries keyframe poses but not keyframe cloud payloads. Therefore `map.pcd` and per-keyframe `patches/*.pcd` are valid zero-point PCD placeholders, with `dense_map.available: false` and `patches.available: false` in metadata. This is intentional structural evidence, not a PGO fused map and not an input to localization or Nav2.

Future dense reconstruction must consume explicit keyframe clouds, set `optimized: true` only after verified PGO correction, and preserve the same checksummed artifact layout.
