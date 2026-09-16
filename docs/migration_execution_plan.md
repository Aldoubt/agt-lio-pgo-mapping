# Migration Execution Plan: Baseline v0.1

## Status and guardrails

This is an implementation plan derived from `migration_manifest_v0.1.yaml`; it is not migration authorization. No source file has been copied in the current repository state. Each commit below requires review, builds independently, and keeps `agt_navigation_v3` operational until the new baseline acceptance passes.

Phase 1 has one scope only:

```text
MID360 rosbag -> LiDAR adapter -> FAST-LIO2 -> PGO -> artifact exporter
```

It excludes Batch-LIO, HBA execution, dynamic filtering, OctoMap/PGM, Nav2, localization, HMI, RTK and robot runtime.

## Phase 1 packages

| New package | Responsibility | Source reference | Initial dependency boundary |
| --- | --- | --- | --- |
| `agt_mapping_interfaces` | Mapping status/artifact messages and export API | Map artifact portions of `agt_robot_interfaces` | ROSIDL only; no navigation/HMI messages |
| `agt_mid360_adapter` | MID360 message adaptation, topic mapping and calibration validation | `sensor/agt_livox_tools` | `rclcpp`, `sensor_msgs`, `livox_ros_driver2`, TF, YAML |
| `agt_fastlio2_backend` | FAST-LIO2 launch/config binding for a normalized MID360 input | `mapping/agt_mapping_bringup` | External `fastlio2`, launch packages, adapter |
| `agt_pgo_backend` | PGO launch/config binding and save-map request adapter | `mapping/agt_mapping_bringup` | External `pgo`, mapping interfaces, launch packages |
| `agt_map_artifact_exporter` | Output directory validation, PGO export orchestration, metadata and checksums | PGO output contract and mapping scripts | PCL/PCD, YAML, mapping interfaces |
| `agt_mapping_bringup` | User-facing composition launch `mapping_v0.launch.py` | `agt_mapping_bringup` launch intent only | The five packages above and rosbag replay support |

The baseline workspace is independently buildable with:

```bash
colcon build --packages-select \
  agt_mapping_interfaces agt_mid360_adapter agt_fastlio2_backend \
  agt_pgo_backend agt_map_artifact_exporter agt_mapping_bringup
```

External packages (`fastlio2`, `pgo`, `livox_ros_driver2`) are resolved at fixed commits through a future dependency lockfile. Their source must not be copied into this repository.

## Phase 1 migration content

All paths below are planned copies after approval. Their current state is `copy_now: false` and `modify_now: false` in the manifest.

| Component | Planned source files from `agt_navigation_v3` | Destination | Action after approval |
| --- | --- | --- | --- |
| MID360 adapter | `sensor/agt_livox_tools/{CMakeLists.txt,package.xml,src/livox_format_bridge.cpp,launch/livox_format_bridge.launch.py,config/*.yaml}` | `sensor/agt_mid360_adapter/` | Copy as initial implementation; rename package/node only after a compatibility test proves topic/message preservation. |
| FAST-LIO2 binding | `mapping/agt_mapping_bringup/{CMakeLists.txt,package.xml,launch/mapping_mode.launch.py,config/fastlio2_mid360.yaml,scripts/mid360_imu_preflight.py}` | `backends/agt_fastlio2_backend/` | Copy configuration as a frozen reference; extract only the single-frontend launch behavior. |
| PGO binding | `mapping/agt_mapping_bringup/{launch/mapping_mode.launch.py,config/pgo_mid360.yaml,launch/hba_refine.launch.py}` | `backends/agt_pgo_backend/` | Copy only PGO topic/config intent; do not bring HBA into v0.1 execution. |
| Exporter reference | `mapping/agt_mapping_bringup/{scripts/save_mapping_debug.sh,README.md}` | `exporters/agt_map_artifact_exporter/` | Use as behavioral reference for `/pgo/save_maps`; implement a framework-owned exporter rather than copying a debug shell script unchanged. |
| Metadata reference | `mapping/agt_mapping_bringup/config/mapping_mode.yaml`; `interfaces/agt_robot_interfaces/{action/GenerateMapPackage.action,msg/MapPackage.msg}` | `interfaces/agt_mapping_interfaces/` | Extract only neutral artifact fields; do not inherit navigation activation, HMI or package-management semantics. |

## Required refactoring

| Area | Required change | Reason |
| --- | --- | --- |
| Package names | Replace source package names with the six v0.1 package names above. | The framework must not depend on similarly named packages in `agt_navigation_v3`. |
| Launch composition | Replace `mapping_mode.launch.py` with `mapping_v0.launch.py`; launch exactly one frontend and one PGO instance. | The source launch contains optional navigation/OctoMap behavior and temporary YAML overlays. |
| Topic contract | Parameterize LiDAR, IMU, body-cloud and odometry topics with baseline defaults; validate types and frames at startup. | Source defaults mix `/agt/sensors/*` and `/livox/*`. |
| Frame contract | Publish and record `T_map_mapping_body`; prohibit navigation `map -> odom` ownership. | Prevents collision with `agt_navigation_v3` localization ownership. |
| Export interface | Wrap `/pgo/save_maps` behind `agt_map_artifact_exporter`; make output root explicit and require a new empty run directory. | PGO export alone does not ensure framework metadata, checksums or deterministic directory layout. |
| Metadata | Define framework-only schema and hash calibration/configuration/input bag metadata. | Existing MapPackage fields include navigation deployment concerns. |

## Content kept unchanged initially

- The audited MID360 LiDAR/IMU calibration values remain frozen in the copied FAST-LIO2 profile; no online extrinsic estimation is enabled in v0.1.
- FAST-LIO2 and PGO algorithm implementations remain external dependencies at pinned revisions.
- The PGO keyframe/loop configuration in `pgo_mid360.yaml` is retained as a baseline reference until a repeatable rosbag run establishes new thresholds.
- PGO's native exported `map.pcd`, `patches/`, `poses.txt` and `poses_timed.txt` layout remains the source format. The framework adds metadata and integrity files around it.
- The MID360 adapter preserves timestamp and message data; it does not filter, voxelize or dynamically remove points.

## First runnable acceptance

### Command

Target command after the five commits and external dependency installation:

```bash
ros2 launch agt_mapping_bringup mapping_v0.launch.py \
  bag_path:=/home/yangxuan/ros2_ws/experiments/data/rosbag/bunker_mid360_mapping_20260901_205036 \
  output_dir:=/absolute/path/to/output
```

The launch must use simulation time, discover or receive explicit MID360 LiDAR/IMU topic mappings, and create exactly this final result root:

```text
output/
└── map_package/
    ├── map.pcd
    ├── poses.txt
    ├── poses_timed.txt
    ├── patches/
    ├── metadata.yaml
    ├── manifest.yaml
    └── checksums.sha256
```

`output_dir` is a launch argument and must be an empty directory or a unique run directory. The launch must reject a pre-existing non-empty `map_package/` rather than overwrite evidence.

### Acceptance checks

| Check | Pass condition |
| --- | --- |
| Replay input | Bag exists and contains compatible MID360 LiDAR and IMU streams. |
| Pipeline lifecycle | Bag completes; exactly one LIO and one PGO node run; both exit without fatal failure before export. |
| `map.pcd` | Exists below `output/map_package/`, is non-empty and loads as PCD. |
| `poses.txt` | Exists, parses, contains at least one record, and every record references a patch. |
| `metadata.yaml` | Exists and contains schema version, input topics/types, frames, calibration direction/value hash, backend versions, parameter hashes and source bag metadata hash. |
| Integrity | `checksums.sha256` covers `map.pcd`, `poses.txt`, `poses_timed.txt`, `metadata.yaml` and every patch; verification succeeds. |
| Evidence | `manifest.yaml` and regression report identify command arguments, timestamps, node versions and pass/fail results. |

The first acceptance makes no claim about navigation usability, localization accuracy or field readiness.

## First migration commit sequence

### Commit 1 — package skeleton

Create package manifests, build files, resource markers, launch/config placeholders and the independent interface package. Add no copied implementation. Verify the six-package `colcon build --packages-select` command resolves all internal dependencies.

### Commit 2 — sensor adapter

After approval, copy the planned MID360 adapter files. Refactor only package names, parameter names and launch wiring required to eliminate `agt_navigation_v3` dependencies. Add a focused test that confirms input/output timestamp and frame preservation.

### Commit 3 — LIO backend adapter

Copy the frozen FAST-LIO2 launch/config reference and IMU preflight utility. Refactor launch arguments to the framework topic/frame contract, retain one frontend instance, and pin the external FAST-LIO2 revision. Validate a short rosbag replay before proceeding.

### Commit 4 — PGO adapter

Add the PGO configuration/binding and service adapter. Preserve keyframe/loop baseline parameters, bind only normalized body cloud plus LIO odometry, and verify that a save request produces PGO-native map, patch and pose files.

### Commit 5 — artifact exporter

Implement the framework-owned exporter/validator around the PGO output. Generate `metadata.yaml`, `manifest.yaml`, `checksums.sha256` and the regression report, then add `mapping_v0.launch.py` and the full existing-MID360-bag regression test.

Each commit must be reviewable and buildable on its own. Rebase or amend only within the framework repository; do not modify the source repository during this sequence.
