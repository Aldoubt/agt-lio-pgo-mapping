# LIO backend source and benchmark audit

Audit date: 2026-10-02. This records local evidence; it does not promote a
backend based on a repository name or a relative map comparison.

## Repositories and source identity

| Component | Local repository / source | Remote or recorded origin | Revision | Local state |
|---|---|---|---|---|
| Mapping framework | `/home/yangxuan/ros2_ws/src/agt_mapping_framework` | `Aldoubt/agt-lio-pgo-mapping` | `0505094131a40a65ddb3bc260e4d3f3ebbf6931d` | clean at audit; branch `feature/multi-lio-backend-v1` is based on the greenhouse benchmark HEAD |
| Historical LIO benchmark tools | `/home/yangxuan/lio_benchmark_tools` | `Aldoubt/lio_benchmark_tools` | `bdbe60622a6370b20db0940a599f170571ce0414` | clean at audit; branch `main` |
| FAST-LIO2 source used by this framework | `/home/yangxuan/ros2_ws/src/external/fast_lio2_mapping` | `Aldoubt/fast-lio2` | local HEAD `a12f8db1718ef23075b24817bca927cab9cca021`; framework `.repos` pin `7f664a66ae7b3b683dddf7f5322c1ea55a3c8141` | `lidar_processor.cpp` has a pre-existing local modification; preserved |
| LIO-SAM ROS 2 source used in the old benchmark | `/home/yangxuan/lio_benchmark_algorithms/lio_sam_ws/src/LIO-SAM` | `TixiaoShan/LIO-SAM`, branch `ros2` | `08af3f32f01725372d4269838dc44c19c6d9e76b` | `include/lio_sam/utility.hpp` has a pre-existing 6-axis IMU compatibility patch; do not overwrite |
| Point-LIO ROS 2 source used in the old benchmark | `/home/yangxuan/lio_benchmark_algorithms/point_lio_ws/src/point_lio_ros2` | `dfloreaa/point_lio_ros2`, branch `main` | `a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7` | source and matching executable are present |
| FAST-LIVO2 LIO-only source used in the old benchmark | `/home/yangxuan/agt_navigation_v2/third_party/fast_livo2_ros2` | vendored by `Aldoubt/agt_navigation_v2`; benchmark provenance records `1e96f08f992aab57a7b738ad7964edff835caee2` | benchmark-recorded source revision | matching executable is present in `agt_navigation_v2/install` |
| Livox driver | `/home/yangxuan/ros2_ws/src/external/livox_ros_driver2` | `Aldoubt/livox_ros_driver2` | local HEAD `7089e1434933bba684d53f522534db1e7095f498`; framework `.repos` pin `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | local revision differs from pin |

The current ROS 2 workspace contains FAST-LIO2, Batch-LIO, the official-message
driver fork, and a FAST-LIO adapter. It does not contain LIO-SAM, Point-LIO, or
FAST-LIVO2 under `ros2_ws/src`; those three sources and their historical build
workspaces live at the paths above. The exact historical LIO executables exist
outside the framework install. No estimator source was changed by this audit.

## Sensor timing and frames found in source/configuration

The old benchmark bag is
`/home/yangxuan/lio_benchmark_tools/date/mapping_20260719_172810`, not the
current acceptance bag `/home/yangxuan/rosbags/green-house`. Its manifest records
Livox `CustomMsg`, `offset_time:uint32` in nanoseconds relative to the message
header, IMU acceleration in g, and no independent ground truth.

| Backend | Historical input path | Per-point timing | Native output evidence |
|---|---|---|---|
| LIO-SAM no-loop | `custommsg_to_pointcloud2` to `/lio_benchmark/lio_sam_points`; SI-scaled IMU to `/lio_benchmark/imu_si` | converter emits `ring:uint16` and `time:float32` seconds from `offset_time`; LIO-SAM consumes `time`/`t` and uses it for deskew | `/lio_sam/mapping/odometry`, `/lio_sam/mapping/path`; registered raw cloud is published in `odom` frame |
| Point-LIO | same timing-preserving PointCloud2 converter to `/lio_benchmark/points`; direct IMU topic | `time` is converted to `curvature` in milliseconds using `timestamp_unit: 0` (seconds input); source uses per-point curvature in scan/IMU synchronization | `/aft_mapped_to_init`, `/path`; source can publish `/cloud_registered_body` when body-scan publication is enabled |
| FAST-LIVO2 LIO-only | direct `/agt/sensors/lidar/custom` and `/agt/sensors/imu/data` | source consumes `CustomMsg.points[].offset_time` in ns and converts to scan-relative ms; `img_en: 0`, `lidar_en: 1` | `/aft_mapped_to_init`, `/path`; `/cloud_registered_lidar` is the scan cloud in `lidar_link`, `/cloud_registered` is world-frame output |
| FAST-LIO2 | framework native CustomMsg adapter and IMU inputs | driver CustomMsg timing; queue replay audit captures callback/processed stamps | `/fastlio2/lio_odom`, `/fastlio2/body_cloud`, `/fastlio2/lio_path` |

The benchmark LIO-SAM profile sets `loopClosureEnableFlag: false`, uses a
disabled GPS topic, and does not feed GPS messages. It enables the existing
`allow6AxisImu` compatibility patch because this bag carries an invalid IMU
orientation quaternion. This patch estimates roll/pitch from acceleration and
sets initial yaw to zero. That is a recorded compatibility modification to the
estimator source, not an untouched upstream build.

## Historical benchmark evidence

The frozen experiment
`mapping_20260719_172810_full807_gravity_compare_002` used the 807.45-second
bag above, at playback rate 1.0. It did not use the current greenhouse
acceptance bag. Its reports explicitly say `ground_truth_available: false` and
classify trajectory comparisons as relative-to-FAST-LIVO2 diagnostic evidence.
The map-health section marks maps unavailable; the stored PLYs and PNGs are
visual diagnostics, not an accepted map-consistency score. Multiple frozen
exports contain the same run metrics and do not constitute independent replay
repeatability runs.

Verified rows from the latest frozen summary:

| Backend | Run result / trajectory health | Path length / endpoint displacement | Mean CPU / peak RSS | Map quality / repeatability |
|---|---|---|---|---|
| LIO-SAM no-loop | `SUCCESS`, heuristic health pass | 194.21 m / 1.44 m; z range 11.66 m | 88.72% / 488.99 MiB | `NOT_MEASURED`; repeated replay `NOT_MEASURED` |
| Point-LIO | process `SUCCESS`, trajectory health **fail** (`trajectory_short`, `path_divergence`) | 63,649.00 m / 61,991.89 m; z range 15,148.03 m | 29.29% / 446.63 MiB peak (207.03 MiB mean) | `NOT_MEASURED`; repeated replay `NOT_MEASURED` |
| FAST-LIVO2 LIO-only | `SUCCESS`, heuristic health pass | 196.98 m / 1.43 m; z range 0.64 m | 124.29% / 976.18 MiB | `NOT_MEASURED`; repeated replay `NOT_MEASURED` |

CPU is the process monitor's percent-of-core value and can exceed 100% for a
multi-threaded process. The Point-LIO run used the recorded fork/config above;
its lower observed CPU and memory in that one run do not make it a supported
mapping choice because its trajectory failed the benchmark's health gate.
The historical summary contains no per-scan latency P50/P95/max field:
processing latency is `NOT_MEASURED` there. No old result is an absolute
accuracy ranking.

## Current framework implementation at audit

- `agt_mapping_frontend_api` already defines backend-neutral `odometry`,
  `cloud`, and `path` topic names, but its documentation did not define cloud
  frame/content semantics or `T_frontend_body`.
- `agt_fastlio_backend` is the only LIO relay package. `agt_pgo_backend` is an
  optimization bridge; there are no LIO-SAM, Point-LIO, or FAST-LIVO2 adapters
  in this repository.
- The generic PGO artifact exporter and validator require optimized PGO
  metadata. FAST-LIO2 has a separate same-session reference package writer.
  There is not yet one backend-independent map-package writer/validator.
- The live launch is FAST-LIO2 + PGO. A launch selector for multiple LIO
  backends does not yet exist.
- The greenhouse BBS/GICP benchmark is present and consumes validated map
  packages. It must remain downstream of the backend-neutral package boundary.

## Evidence classification

- **VERIFIED FROM LOCAL BENCHMARK:** versions, old dataset fields, one-run CPU/RSS/path metrics, LIO-SAM no-loop parameter flags, 6-axis patch, Point-LIO trajectory-health failure, FAST-LIVO2 image-disabled LIO mode.
- **CURRENT SOURCE CODE:** framework branch/HEAD and `.repos`; available local estimator checkouts; topic names, frame IDs, timing conversions, exporter validation rules.
- **ASSUMPTION:** historical benchmark health heuristics and visualizations are useful for selecting what to validate next, but they are not independent ground truth or multi-run evidence.
- **NOT VERIFIED:** any of the three official backend's map-package acceptance on `/home/yangxuan/rosbags/green-house`; repeated-run consistency on that bag; LIO-SAM false-loop behavior beyond the user's recorded observation; cross-day/cross-growth accuracy; absolute pose accuracy; backend processing-latency percentiles from the old benchmark.

Until the required current-bag acceptance is run, no backend is certified as
the framework default. In particular, the old benchmark does not justify
promoting Point-LIO based on resource measurements alone.
