# Multi-LIO backend acceptance

**Overall status: PARTIAL**  
**Acceptance date:** 2026-10-03 (Asia/Shanghai)  
**Bag:** `/home/yangxuan/rosbags/green-house`  
**Experiment root:** `/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003`

## Repository

- **Branch:** `feature/multi-lio-backend-v1`
- **HEAD:** `0505094131a40a65ddb3bc260e4d3f3ebbf6931d`
- **Workspace state:** modified by this task; the starting worktree was clean. Changes are uncommitted. No external estimator source tree or formal install was changed.
- **Build location:** isolated build/install under `experiments/multi_lio_backend_20261003`; the installed application was not overwritten.

## Existing benchmark evidence

The benchmark source is `/home/yangxuan/lio_benchmark_tools`, remote
`https://github.com/Aldoubt/lio_benchmark_tools.git`, branch `main`, HEAD
`bdbe60622a6370b20db0940a599f170571ce0414`.

| Backend | Existing local evidence |
|---|---|
| LIO-SAM | Historical source/config and one older run; no same-bag repeatability or accepted map artifact |
| Point-LIO | Historical resource run used `dfloreaa/point_lio_ros2` commit `a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7`; a previous trajectory-health check failed |
| FAST-LIVO2 | Historical LIO-only run used the source vendored in `agt_navigation_v2`; this is the run previously confused with FAST-LIO2 |
| FAST-LIO2 | Not included in the old benchmark; separate queue replay audit found same-bag trajectory spread and keeps it legacy/experimental |

Historical metrics and full source/config provenance are in
[LIO_BACKEND_AUDIT.md](LIO_BACKEND_AUDIT.md). The older runs used a different
bag and are not used as current-bag accuracy scores.

## Backend implementations

| Backend | Source used | Current implementation result |
|---|---|---|
| LIO-SAM no-loop | `/home/yangxuan/lio_benchmark_algorithms/lio_sam_ws/src/LIO-SAM`, TixiaoShan `ros2` commit `08af3f32f01725372d4269838dc44c19c6d9e76b`; retained the existing six-axis IMU compatibility patch | **PASS** build, timed MID360 input, normalized frontend output and map export. **FAIL** replay repeatability; not eligible for default |
| Point-LIO | `/home/yangxuan/lio_benchmark_algorithms/point_lio_ws/src/point_lio_ros2`, commit `a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7` | **PASS** build, timed PointCloud2 input, frontend output, map export and one BBS/GICP interface smoke |
| FAST-LIVO2 LIO-only | `/home/yangxuan/agt_navigation_v2/third_party/fast_livo2_ros2`, source commit `1e96f08f992aab57a7b738ad7964edff835caee2` | **PASS** build, raw `CustomMsg`/`offset_time` input, frontend output, map export and one BBS/GICP interface smoke. Verified `img_en=0`, `lidar_en=1`, `imu_en=true`; source selects `ONLY_LIO` |
| FAST-LIO2 | Existing local fork and `.repos` pin differ; see [FAST_LIO2_REGRESSION_STATUS.md](FAST_LIO2_REGRESSION_STATUS.md) | **PARTIAL** source audit only; legacy/experimental, not run in this acceptance |

The registry requires explicit backend IDs. FAST-LIO2 requires an explicit
experimental opt-in. The LIO-SAM profile explicitly disables loop closure,
GPS and external global correction.

## Default selection

**DEFAULT: `DEFAULT_NOT_ESTABLISHED`**

LIO-SAM no-loop passed build, frame/timing conversion and artifact checks, but
failed same-bag repeatability by meter-scale XY and multi-degree yaw differences.
Point-LIO and FAST-LIVO2 each produced coherent-looking parallel greenhouse
rows and valid artifacts, but each has only one current-bag replay and no
absolute ground truth. Neither is promoted to default from a single run.

## Greenhouse bag acceptance

Bag metadata reports **6,230** Livox `CustomMsg` messages, **124,600** IMU
messages, and **622.994 s** duration. All runs used playback rate `1.0`,
`ROS_DOMAIN_ID=126`, `ROS_LOCALHOST_ONLY=1`, no CPU affinity, the same host,
the same bag and each backend's recorded benchmark config/extrinsic. No PGO,
full BBS/GICP standard, loop closure or GPS factor was run.

The adapter's input rates are callback measurements, not estimator accepted
counts. Native estimators do not expose per-scan accepted/drop counters, so
`dropped_lidar`, `dropped_imu` and estimator processing latency remain
`UNKNOWN`. The normalized odometry/cloud pair count is reported separately.
CPU/RSS are whole-launch process-tree measurements, including bag player,
frontend adapter and map exporter. `frontend output latency` is ROS-time stamp
age; `adapter processing latency` covers only message normalization.

| Backend/run | Input LiDAR / IMU median Hz | Frontend output median Hz; paired outputs | Keyframes / map points | Path length / start-end displacement | Artifact | Trajectory topology |
|---|---:|---:|---:|---:|---|---|
| LIO-SAM no-loop run 1 | 10.00 / 175.95 | 4.05 Hz; 2,599 | 959 / 13,602,478 | 522.19 m / 16.65 m | **PASS** | **PARTIAL**: main rows visible; repeated run differs |
| LIO-SAM no-loop run 2 | 10.00 / 176.14 | 4.00 Hz; 2,386 | 934 / 13,216,683 | 524.61 m / 19.25 m | **PASS** | **FAIL repeatability** |
| Point-LIO | 10.00 / 200.00 | 10.00 Hz; 6,165 | 1,180 / 8,477,666 | 527.95 m / 15.89 m | **PASS** | **PARTIAL**: parallel-row topology visible; no absolute GT |
| FAST-LIVO2 LIO-only | 10.00 / 199.00 | 10.00 Hz; 6,009 | 1,162 / 7,651,586 | 527.83 m / 15.75 m | **PASS** | **PARTIAL**: parallel-row topology visible; no absolute GT |

All adapters reported zero transform errors. The lower observed LIO-SAM IMU
callback rate and 4 Hz output rate are recorded observations; exact missing
scan/IMU counts cannot be recovered from the estimator diagnostics.

### LIO-SAM replay repeatability

Runs `runs/lio_sam_noloop` and `runs/lio_sam_noloop_repeat2` use the same source,
config hash, extrinsic, bag, rate and ROS domain. Trajectories are compared
directly in their original map frames; no alignment or registration was
applied. Positions/yaw were interpolated on a common 0.25 s bag-time grid.

| Measure | Result |
|---|---:|
| First XY difference above 0.1 m | 1.75 s |
| First XY difference above 1 m | 33.25 s |
| XY difference median / P95 / max | 1.12 / 3.68 / 3.98 m |
| Yaw difference median / P95 / max | 2.89 / 10.04 / 19.70° |
| Map XY occupancy IoU, 0.5 m cells | 0.734 |
| Keyframe count | 959 vs 934 |
| Repeatability | **FAIL** |

This is a trajectory repeatability failure despite both artifacts passing the
exact map/pose/patch validator. The first run image shows the greenhouse rows;
the overview also contains detached peripheral point clusters, which were not
filtered from the artifact.

Visuals: [all three backend maps](/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003/runs/mapping_backends_compare.png) · [LIO-SAM repeatability comparison](/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003/runs/lio_sam_noloop_repeat2/repeatability_comparison.png) · [LIO-SAM map detail](/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003/runs/lio_sam_noloop/mapping_effect_zoom.png)

### Runtime and latency

| Backend/run | Whole-chain CPU P50 / P95 / max (% of one core) | Whole-chain RSS P50 / P95 / max (MiB) | Frontend output age P50 / P95 / max (ms) | Adapter processing P50 / P95 / max (ms) |
|---|---:|---:|---:|---:|
| LIO-SAM run 2 | 190.2 / 248.9 / 287.3 | 760 / 976 / 1,421 | 469.8 / 691.8 / 1,086.2 | 52.3 / 73.2 / 185.1 |
| Point-LIO | 48.5 / 57.4 / 112.7 | 497 / 574 / 829 | 16.1 / 22.0 / 365.8 | 0.33 / 1.04 / 11.7 |
| FAST-LIVO2 LIO-only | 216.8 / 316.7 / 360.6 | 768 / 1,064 / 1,244 | 25.0 / 50.1 / 124.9 | 23.9 / 41.7 / 109.3 |

The nonzero output-age maxima include startup/initialization. They are not
native estimator processing-time measurements. Source-specific CPU/RSS and
latency analysis is saved as `analysis.json` beside each run.

## Unified frontend contract

**PASS.** Each accepted run published:

- `/mapping/frontend/odometry` (`nav_msgs/msg/Odometry`)
- `/mapping/frontend/cloud` (`sensor_msgs/msg/PointCloud2`), local body cloud
- `/mapping/frontend/path` (`nav_msgs/msg/Path`)
- `/mapping/frontend/status` (`diagnostic_msgs/msg/DiagnosticArray`)

The adapters only normalize timestamps, frame labels and rigid extrinsics; they
do not re-estimate pose, filter points or change timestamps. Livox per-point
timing is retained: the timed `PointCloud2` converter preserves the true line
as `ring`, float seconds as `time`, and original nanoseconds as `offset_time`.
FAST-LIVO2 LIO-only receives the raw `CustomMsg` directly.

## Unified map package

**PASS.** All four current greenhouse exports (two LIO-SAM, one Point-LIO, one
FAST-LIVO2) passed `verify_frontend_map_package`. Each contains the same
backend-independent `map.pcd`, `poses.txt`, `poses_timed.txt`, body-frame
`patches/`, calibration, provenance manifest and checksums. The validator
checks exact map-point reconstruction from patch/pose pairs within 5 µm. The
manifest records `absolute_ground_truth: false`.

## 3D-BBS / GICP compatibility smoke

This section records one or two same-session map-patch queries only. It is an
interface smoke and does not establish held-out relocalization accuracy.

| Backend artifact | BBS / descriptor asset build | Candidate BBS→GICP call |
|---|---|---|
| Point-LIO | **PASS**; 66,788 downsampled points, 1,180 descriptor entries | **PASS** on `patches/8.pcd`: BBS score 0.9996, GICP fitness 0.1398, overlap 1.000. A first `patches/0.pcd` query selected another repeated-row patch and GICP did not converge; that failure is retained in the smoke logs |
| FAST-LIVO2 | **PASS**; 58,108 downsampled points, 1,162 descriptor entries | **PASS** on `patches/8.pcd`: BBS score 0.9994, GICP fitness 0.1919, overlap 0.9997 |
| LIO-SAM no-loop | `NOT_RUN` because replay repeatability failed | `NOT_RUN` |

No backend-specific condition was added to the BBS/GICP asset builders. All
smoke outputs and logs are under each run's `relocalization_smoke/` directory.
No full greenhouse BBS/GICP standard was run.

## Tests and builds

- Isolated `colcon build` of `agt_mapping_frontend_api`, `agt_mid360_adapter`,
  `agt_mapping_frontend_adapter`, `agt_mapping_artifacts`,
  `agt_mapping_bringup` and `agt_map_localization_benchmark`: **PASS**.
- `pytest` over artifact, bringup and benchmark tests: **149 passed, 4 skipped**.
- `colcon test` plus `colcon test-result --verbose`: **161 tests, 0 failures,
  0 errors, 0 skipped**.
- `ros2 launch agt_mapping_bringup mapping.launch.py --show-args`: **PASS**.
- Backend source/profile provenance checks: **PASS** for the three formal
  backends.

## Known limitations

- No absolute ground truth or independent wheel-odometry reference was present
  in the acceptance bag.
- Point-LIO and FAST-LIVO2 have one current-bag run each; multi-run
  repeatability remains `NOT_RUN`.
- Cross-day and cross-growth-season map consistency is `NOT_RUN`.
- LIO-SAM no-loop has measurable same-bag nondeterminism and is not the default.
- Native estimator drop counts and per-scan processing times are unavailable;
  they remain `UNKNOWN`, not zero.
- Repeated greenhouse rows can confuse candidate retrieval; the failed
  Point-LIO `patches/0.pcd` smoke demonstrates this limitation.
- Loop closure, GPS factors and external global corrections were disabled and
  were not evaluated.

## Rollback

The framework branch began at `0505094131a40a65ddb3bc260e4d3f3ebbf6931d` and
has no new commits. To review the baseline without disturbing these uncommitted
changes or experiment outputs, create a separate worktree at that commit. All
bag-derived runs are outside the source tree under
`/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003`.

## Status summary

- **PASS:** isolated build/tests, frontend contract, all four map artifacts,
  Point-LIO and FAST-LIVO2 BBS/GICP interface smoke.
- **FAIL:** LIO-SAM no-loop repeated trajectory consistency.
- **PARTIAL:** current backend selection; no default meets the evidence gate.
- **NOT_RUN:** Point-LIO/FAST-LIVO2 repeatability, LIO-SAM BBS/GICP smoke,
  absolute-GT accuracy, cross-day/cross-season acceptance, full greenhouse
  BBS/GICP standard.
- **BLOCKED:** none.
