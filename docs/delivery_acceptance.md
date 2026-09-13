# Delivery Acceptance

This document records the acceptance boundary for the baseline release.

## Supported host

- Ubuntu 22.04
- ROS 2 Humble
- Network access during `scripts/bootstrap.sh`
- A user allowed to run `sudo` for ROS/system dependencies

The bootstrap script pins external source revisions in `.repos`, installs `ros-humble-gtsam` and `ros-humble-sophus`, prepares the official Livox ROS 2 manifest, resolves remaining rosdep dependencies, and builds `agt_mapping_bringup` and its transitive dependencies.

## Reproducibility acceptance

The following was verified from a newly cloned workspace, with no source directories from the development workspace used as build inputs:

1. Clone `https://github.com/Aldoubt/agt-lio-pgo-mapping.git`.
2. Import `.repos` into `src/external`.
3. Build `--packages-up-to agt_mapping_bringup`.
4. Confirm `livox_ros_driver2_node`, `lio_node`, `pgo_node`, `mid360_adapter_node`, and `pgo_backend_node` are installed.
5. Source that new workspace and resolve `agt_mapping_bringup`, `fastlio2`, and `pgo` with `ros2 pkg prefix`.
6. Load `mapping_v0.launch.py --show-args`.

Framework unit tests passed in that cloned workspace: 11 tests, 0 failures.

## Mapping regression acceptance

The baseline was run against the MID360 bag `bunker_mid360_mapping_20260901_205036`.

- 333 optimized keyframes and patches
- 444.4 seconds of trajectory coverage
- 673,897 points in `map.pcd`
- maximum consecutive trajectory step: 0.554 m
- maximum implied speed: 0.569 m/s
- `metadata.yaml`: `backend: PGO`, `optimized: true`
- `sha256sum -c checksums.sha256`: passed

The rosbag is intentionally not stored in this repository. A recipient supplies a compatible MID360 bag directory to `scripts/run_mid360_mapping.sh`; use `scripts/verify_map_artifact.sh` before accepting an exported artifact.
