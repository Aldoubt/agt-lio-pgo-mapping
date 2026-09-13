# Interface Contract

## Status

This is a proposed collaboration contract. It defines stable data semantics for future implementations; it does not create ROS messages, services or code in the current initialization phase.

## Input

| Input | Required fields / semantics | Requirement |
| --- | --- | --- |
| Point cloud | Timestamp, frame ID, XYZ, intensity when available, and per-point timing when required by the selected LIO frontend | Required |
| IMU | Timestamp, angular velocity and linear acceleration in a declared IMU frame | Required for LIO |
| Pose | Timestamped local odometry or optional seed pose; pose frame and child frame must be explicit | Optional input; required between pipeline stages as specified |
| Calibration | LiDAR-to-IMU rigid transform, transform direction, time offset, sensor model and frame names | Required |

### Input invariants

- Timestamps use ROS time and are monotonic within each stream; replay mode must declare simulated time.
- Point cloud and IMU time bases must be documented and synchronizable.
- Calibration is immutable for a mapping session. Any change produces a new artifact lineage.
- Raw input is retained or reproducibly referenced; downstream filtering cannot overwrite acquisition evidence.

## Output

| Output | Required content |
| --- | --- |
| `map.pcd` | Dense fused point cloud in the declared global map frame. |
| `trajectory` | `poses.txt` and `poses_timed.txt`; each record identifies a keyframe patch, timestamp and optimized `T_map_mapping_body`. |
| `patches/` | One body-frame PCD per exported keyframe; filename must match the trajectory record. |
| `metadata.yaml` | Artifact schema/version, frame semantics, calibration, source topics, algorithm versions, parameters, checksums and lineage. |

The canonical artifact directory is:

```text
<artifact_root>/
├── map.pcd
├── patches/
│   └── <keyframe-id>.pcd
├── poses.txt
├── poses_timed.txt
├── metadata.yaml
└── manifest.yaml
```

## Metadata minimum contract

`metadata.yaml` must contain at least:

```yaml
format_version: 1
frames:
  map_frame: map
  body_frame: mapping_body
  lidar_frame: lidar
  imu_frame: imu
calibration:
  lidar_to_imu_transform_direction: T_imu_lidar
  time_offset_sec: 0.0
inputs:
  lidar_topic: /example/lidar
  imu_topic: /example/imu
processing:
  lio_backend: fastlio2
  optimizer: pgo
  parameters_sha256: "<sha256>"
outputs:
  map: map.pcd
  patches_dir: patches
  poses: poses.txt
  poses_timed: poses_timed.txt
```

The transform direction shown above is an example only. Each artifact must state its actual convention; consumers must never infer it from a backend name.

## Compatibility and ownership

- A consumer validates schema version, required files, checksums and frame semantics before use.
- A navigation system may derive PGM, OctoMap, relocalization indexes or terrain products, but must retain the parent artifact identifier.
- `map.pcd` is a mapping artifact, not an authorization to activate a navigation map.
- The framework does not publish the navigation runtime `map -> odom` transform. That responsibility remains with the navigation localization system.
