# Baseline v0.1: Minimal Mapping Pipeline

## Goal

Baseline v0.1 defines the smallest reproducible ROS 2 mapping job that turns an existing MID360 rosbag into a versioned 3D map artifact:

```text
MID360 rosbag -> LiDAR adapter -> LIO frontend -> PGO backend -> Map artifact exporter
```

The baseline establishes a reference experiment for later research. It deliberately excludes navigation deployment, runtime localization, HMI, RTK and robot control.

## Input data

The primary regression input is the existing bag:

```text
/home/yangxuan/ros2_ws/experiments/data/rosbag/bunker_mid360_mapping_20260901_205036
```

The alternate bag `bunker_mid360_mapping_20260901_211105` may be used only as a second regression case after the primary case passes.

Required recorded streams, resolved by launch parameters rather than hard-coded assumptions:

| Stream | Message type | Required use |
| --- | --- | --- |
| MID360 LiDAR | `livox_ros_driver2/msg/CustomMsg`, or an adapter-supported `sensor_msgs/msg/PointCloud2` | LIO scan input |
| MID360 IMU | `sensor_msgs/msg/Imu` | LIO inertial input |
| TF/static calibration | TF stream or immutable calibration YAML | LiDAR/IMU frame validation |

The regression setup must run `ros2 bag info` first and record the exact input topics, message counts, duration and bag metadata hash in the result directory.

## Data flow

```text
rosbag replay (/clock)
       |
       v
agt_mid360_adapter
  CustomMsg / PointCloud2 + Imu
       |
       v
agt_fastlio2_bringup
  local odometry + body-frame cloud
       |
       v
agt_pgo_bringup
  keyframes + loop constraints + optimized poses
       |
       v
agt_map_artifact_exporter
  map.pcd, patches/, trajectory, metadata.yaml, manifest.yaml
```

The adapter preserves timestamps and raw point ordering/timing fields required by the chosen frontend. PGO receives synchronized body-frame clouds and local odometry. The exporter owns no optimization: it serializes the PGO optimized state and provenance into an immutable artifact directory.

## Proposed ROS 2 package structure

Baseline v0.1 adds package definitions only after human migration approval. The following package graph is sufficient to build independently with `colcon build --packages-select ...`; it has no dependency on `agt_navigation_v3` packages.

| Package | Responsibility | Direct dependencies |
| --- | --- | --- |
| `agt_mapping_interfaces` | `MappingStatus`, `MappingArtifact`, export service/action definitions | `ament_cmake`, `rosidl_default_generators`, `builtin_interfaces`, `geometry_msgs` |
| `agt_mid360_adapter` | Normalize MID360 LiDAR/IMU input and validate calibration | `rclcpp`, `sensor_msgs`, `livox_ros_driver2`, `tf2_ros`, `yaml-cpp` |
| `agt_fastlio2_bringup` | Parameterized FAST-LIO2 frontend launch/config adapter | `ament_cmake`, `launch`, `launch_ros`, `fastlio2`, `agt_mid360_adapter` |
| `agt_pgo_bringup` | PGO launch/config adapter and export-service binding | `ament_cmake`, `launch`, `launch_ros`, `pgo`, `agt_mapping_interfaces` |
| `agt_map_artifact_exporter` | Validate/export PGO map, patches, trajectory, metadata and hashes | `rclcpp`, `pcl_conversions`, `yaml-cpp`, `ament_index_cpp`, `agt_mapping_interfaces` |
| `agt_mapping_baseline_bringup` | One replay-oriented composition launch and regression test entry point | `ament_cmake`, `launch`, `launch_ros`, above five packages, `rosbag2_transport` |

```text
agt_mapping_baseline_bringup
  ├── agt_mid360_adapter
  ├── agt_fastlio2_bringup ──> fastlio2
  ├── agt_pgo_bringup ──────> pgo
  └── agt_map_artifact_exporter

agt_mapping_interfaces <── agt_pgo_bringup, agt_map_artifact_exporter
```

`fastlio2`, `pgo`, `livox_ros_driver2` and their transitive libraries are external, version-pinned dependencies. They must be resolved through `rosdep` and a future lockfile/`.repos` manifest; their source is not vendored into this repository.

### Launch relationship

```text
baseline_replay.launch.py
  ├── bag replay process (use_sim_time:=true)
  ├── mid360_adapter.launch.py
  ├── fastlio2.launch.py
  ├── pgo.launch.py
  └── artifact_export.launch.py
```

The launch accepts `bag_path`, `lidar_topic`, `imu_topic`, `calibration_path`, `output_dir`, `frontend_config`, `pgo_config` and `autostart_export`. It must start exactly one LIO frontend and one PGO backend. Export is requested only after bag playback ends and PGO drains its queue.

## Run steps

These are target-state steps; they are not executable until the approved packages and external dependencies are present.

1. Source ROS 2 Humble and the framework workspace overlay.
2. Resolve the bag topic/type inventory with `ros2 bag info <bag_path>`; record it in the result directory.
3. Validate the MID360 LiDAR/IMU calibration and the configured input topic mapping.
4. Build only the baseline packages with `colcon build --packages-select agt_mapping_interfaces agt_mid360_adapter agt_fastlio2_bringup agt_pgo_bringup agt_map_artifact_exporter agt_mapping_baseline_bringup`.
5. Run `baseline_replay.launch.py` with `use_sim_time:=true` and a newly created, empty output directory.
6. Wait for replay completion and PGO completion, then invoke the artifact exporter once.
7. Run the artifact validator and write a regression report with the artifact manifest hash.

## Output artifacts

```text
<output_dir>/
├── map.pcd
├── patches/
│   └── <keyframe-id>.pcd
├── poses.txt
├── poses_timed.txt
├── metadata.yaml
├── manifest.yaml
├── checksums.sha256
└── regression_report.yaml
```

`metadata.yaml` follows [interface_contract.md](interface_contract.md). `manifest.yaml` binds the artifact to source bag identity, topic mapping, calibration hash, frontend/PGO versions, configuration hashes and output checksums.

## Rosbag regression test

Test name: `test_mid360_baseline_replay`.

| Phase | Assertion |
| --- | --- |
| Preconditions | The selected bag exists, contains a supported LiDAR stream and IMU stream, and the output directory is empty. |
| Replay | All nodes use simulated time; exactly one frontend and one PGO backend run; no required node exits with failure. |
| Trajectory | `poses.txt` and `poses_timed.txt` exist, parse successfully, contain at least one keyframe, and have matching patch identifiers. |
| Map | `map.pcd` exists, is non-empty and PCL can load it. |
| Metadata | `metadata.yaml` parses and contains format version, frames, calibration, input topics, processing versions/config hashes and output paths. |
| Integrity | `checksums.sha256` exists, covers `map.pcd`, both trajectory files, metadata and every patch, and verifies without mismatch. |
| Report | `regression_report.yaml` records bag path, bag metadata hash, topic mapping, exit status, keyframe count, point count and manifest hash. |

The v0.1 acceptance criterion is artifact completeness and reproducibility, not navigation performance or field readiness. Numerical trajectory/map-quality thresholds will be frozen only after a clean reference run is reviewed.

## Acceptance criteria

Baseline v0.1 is accepted when the primary bag produces one validated artifact directory and the regression test passes all assertions above on a clean workspace. A rerun with identical bag, dependencies, calibration and parameters must produce identical metadata/configuration hashes and a valid checksum manifest. Differences in floating-point map bytes, if any, must be explicitly reported rather than silently accepted.
