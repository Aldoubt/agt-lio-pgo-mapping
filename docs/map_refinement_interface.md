# Map Refinement Interface

Phase 2 defines an offline file interface. The source `map_package` is read
only and the output is a new `refined_map_package`.

## Rule schema

```yaml
version: 1
operations:
  - type: remove_polygon
    points: [[x, y], [x, y], [x, y]]
  - type: remove_box
    min: {x: 0, y: 0, z: 0}
    max: {x: 1, y: 1, z: 2}
  - type: forbidden_zone
    polygon: [[x, y], [x, y], [x, y]]
```

`remove_polygon` and `remove_box` remove matching points from the refined
`map.pcd`. `forbidden_zone` preserves the 3D point cloud and marks the region
occupied in `nav_map.pgm`; it is therefore a navigation restriction rather
than a destructive point-cloud edit. An empty operation list is valid.

The reader accepts ASCII and common binary float32 PCD input. Refined PCD
output is written as ASCII with the original field names retained.

## Output contract

The output retains `map.pcd`, `poses.txt`, `poses_timed.txt`, `patches/`,
`calibration.yaml` when present, and `metadata.yaml`. It adds:

```text
refinement.yaml
filter_report.yaml
nav_map.pgm
nav_map.yaml
manifest.yaml
checksums.sha256
```

`manifest.yaml` records `package_kind: refined_mapping_source`, the parent
package path, and the parent manifest SHA-256. The checksum file covers every
other output file and is suitable for `sha256sum -c`.

## Dynamic-filter evidence interface (deferred)

The first dynamic-filter implementation is offline-only and does not change
FAST-LIO2 or PGO interfaces. Its future input is a YAML evidence index plus
keyframe cloud files:

```yaml
version: 1
source_map_package: /path/to/map_package
keyframes:
  - id: 42
    timestamp: 1720000000.123
    pose: [x, y, z, qw, qx, qy, qz]
    cloud: patches/42.pcd
```

The future filter will produce voxel-level `observation_count`, `frame_count`,
`persistence_ratio`, and `height_variance`, then write static/dynamic PCDs and
`filter_report.yaml`. Phase 2 only reserves this contract and reports the
filter as deferred.