# Architecture

## System boundary

`agt_mapping_framework` owns offline/online mapping computation and reproducible map artifacts. `agt_navigation_v3` owns robot operation and consumes approved artifacts. Neither repository may silently take ownership of the other's responsibilities.

| Mapping Framework | Navigation System (`agt_navigation_v3`) |
| --- | --- |
| MID360/IMU data adaptation and calibration validation | Sensor lifecycle, robot bringup and hardware safety |
| LIO, keyframes, PGO, HBA and dense reconstruction | Local odometry consumption and localization manager |
| PCD, trajectory, patch and metadata export | `map -> odom` ownership and global relocalization |
| Dynamic/semantic/terrain research pipelines | Nav2, costmaps, route execution, HMI and base control |
| Artifact provenance and experiment reproducibility | Map approval, activation, deployment and rollback |

## Complete mapping pipeline

```text
┌──────────────────────────────────────────────────────────────────┐
│                         Mapping Framework                        │
│                                                                  │
│  MID360 + built-in IMU                                           │
│    |  Livox CustomMsg / PointCloud2 + sensor_msgs/Imu            │
│    v                                                             │
│  LiDAR-Inertial Odometry                                         │
│    |  FAST-LIO2 or Batch-LIO                                    │
│    |  deskewed body cloud, local odometry, local trajectory      │
│    v                                                             │
│  Keyframe generation                                             │
│    |  cloud snapshot + timestamp + T_local_mapping_body          │
│    v                                                             │
│  Pose Graph Optimization                                         │
│    |  loop constraints + optimized T_map_mapping_body            │
│    v                                                             │
│  Dense map reconstruction                                       │
│    |  transform/fuse keyframe patches; optional offline HBA      │
│    v                                                             │
│  Map artifacts                                                   │
│    |- map.pcd                                                    │
│    |- trajectory: poses.txt, poses_timed.txt                     │
│    |- patches/<keyframe>.pcd                                     │
│    `- metadata.yaml, manifest/checksums, optional derivatives    │
└───────────────────────────────┬──────────────────────────────────┘
                                │ approved, versioned artifacts
                                v
┌──────────────────────────────────────────────────────────────────┐
│                Navigation System: agt_navigation_v3              │
│  artifact registry/approval -> relocalization assets ->           │
│  localization manager (sole map -> odom owner) -> Nav2 -> robot  │
└──────────────────────────────────────────────────────────────────┘
```

## Processing rules

- Raw LiDAR and IMU are acquisition evidence. Filters for dynamic objects must be explicit branches and must not silently change the front-end input.
- LIO publishes local state only. PGO uses time-aligned body clouds and odometry to select keyframes and estimate global optimized poses.
- HBA is optional offline refinement over exported `patches/` and trajectory; its output must identify its parent PGO artifact and must not overwrite it.
- Dense fusion uses only the optimized pose associated with each keyframe. A map must retain the source patches and trajectory required to reproduce the fusion.
- Navigation-only derivatives such as PGM, OctoMap, costmaps and relocalization indexes are downstream products. They cannot replace the source `map.pcd`.

## Frame and ownership boundary

- Mapping pose semantics: `T_map_mapping_body`; all artifact metadata declares map frame, body frame, LiDAR frame and LiDAR-IMU extrinsic direction.
- The framework may publish mapping visualization transforms in an isolated mapping session, but it does not own navigation runtime `map -> odom`.
- `agt_navigation_v3` performs the one explicit conversion from `mapping_body` to the robot navigation frame, then owns navigation localization and TF publication.

## Reference baseline

The initial reference configuration is audited from `agt_navigation_v3`: Livox conversion, MID360 support, FAST-LIO2/Batch-LIO setup, PGO/HBA configuration, point-cloud preprocessing and terrain/static-confidence utilities. FAST-LIO2, PGO, HBA and Batch-LIO remain external, version-pinned dependencies; no source has been migrated into this repository.
