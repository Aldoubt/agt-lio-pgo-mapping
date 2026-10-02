# Greenhouse relocalization benchmark v1

This is an **offline-only feasibility benchmark** for the tomato-greenhouse
relocalization design. It does not publish TF, change `map->odom`, start Nav2,
or control a chassis. It invokes the already-installed production native
registration executables from `agt_navigation_v3`.

The benchmark answers two separate questions:

1. **GICP basin:** for a representative row-middle / row-end / headland /
   row-entry query, which `(dx, dy, dyaw)` initial errors still converge to the
   declared reference?
2. **BBS -> GICP:** with no initial pose, does the existing descriptor + CPU
   3D-BBS pipeline return a coarse pose that the same LOCAL GICP backend can
   refine successfully?

The second question is tested directly: the benchmark takes the GLOBAL
backend's `coarse_pose` and calls `map_gicp_tracker` again with that exact pose.
`coarse_seed_gicp_nominal_success` is therefore a direct empirical basin-hit
measurement, not a lookup/interpolation from the sampled heatmap.

## Inputs

Only a checksum-verified reference package and a small scene YAML are required.
Confidence / geometry sidecars from Phase 3B are intentionally not required.

Expected map package:

```text
map_package/
├── map.pcd
├── poses_timed.txt
├── patches/
├── manifest.yaml
└── checksums.sha256
```

The source package is checksum-verified and never modified. Every declared
query window is excluded from the generated target map and descriptor map.
This removes direct query-patch self-overlap, but the first experiment is still
The benchmark records the reference type from `metadata.yaml`. A PGO package is
same-session optimized PGO evaluation; a FAST-LIO2 package labelled
`FASTLIO2_SAME_SESSION_REFERENCE` uses unoptimized frontend poses. Neither is
absolute ground truth or evidence of cross-day generalization.

For an offline FAST-LIO2-only package, use the existing mapping entry point
with `--reference fastlio`. This branch launches the adapter, FAST-LIO2,
frontend relay, and the consistent reference exporter; it does not launch PGO.
It writes `fastlio_reference_package/` with `reference.pgo_applied: false`,
`reference.optimized: false`, and `reference.absolute_ground_truth: false`.

## Scene labels

Copy the example and replace keyframe numbers after inspecting
`poses_timed.txt` / RViz:

```bash
cp benchmarks/agt_map_localization_benchmark/config/greenhouse_scenes.example.yaml \
  ~/ros2_ws/experiments/greenhouse_scenes.yaml
```

Supported scene types are:

- `row_middle`
- `row_end`
- `headland`
- `row_entry`

Optional `rows:` ranges let the benchmark map the native `candidate_patch` back
to a Row ID and report `wrong_row_candidate`. Ranges must not overlap.
An optional top-level `row_id_semantics` value is copied to the run manifest
so provisional trajectory groups are not mistaken for surveyed physical rows.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_map_localization_benchmark
source install/setup.bash
```

The actual GICP / BBS implementation is not in this package. The following
executables must already exist in the sourced install:

```text
agt_global_relocalization_native/map_gicp_tracker
agt_global_relocalization_native/candidate_bbs_gicp_localizer
agt_global_relocalization_native/build_relocalization_assets
agt_global_relocalization_native/build_relocalization_candidates
```

## Recommended first run

Start with the smoke profile. It uses one-frame queries and 27 LOCAL initial
conditions per scene:

```bash
ros2 run agt_map_localization_benchmark agt_greenhouse_relocalization_benchmark \
  --map-package /path/to/mapping_output/map_package \
  --scenes ~/ros2_ws/experiments/greenhouse_scenes.yaml \
  --profile smoke \
  --mode all \
  --run-id tomato_smoke_01
```

Outputs default to:

```text
~/ros2_ws/experiments/greenhouse_relocalization_benchmark/<run-id>/
├── manifest.json
├── summary.json
├── basin/
│   ├── basin.csv
│   ├── targets.json
│   ├── queries/
│   └── targets/
└── global/
    ├── global.csv
    ├── global_target_map.pcd
    ├── assets_manifest.json
    ├── assets/
    ├── descriptor_source/
    └── queries/
```

Plot sampled GICP basin slices:

```bash
ros2 run agt_map_localization_benchmark agt_plot_greenhouse_basin \
  --run ~/ros2_ws/experiments/greenhouse_relocalization_benchmark/tomato_smoke_01
```

The plotting helper imports matplotlib only when invoked. If missing:

```bash
sudo apt install python3-matplotlib
```

## Standard run

After a smoke run proves the map, scene labels and native binaries are valid:

```bash
ros2 run agt_map_localization_benchmark agt_greenhouse_relocalization_benchmark \
  --map-package /path/to/mapping_output/map_package \
  --scenes ~/ros2_ws/experiments/greenhouse_scenes.yaml \
  --profile standard \
  --mode all \
  --run-id tomato_standard_01
```

The standard profile uses 1/3/5-frame queries and a denser grid:

```text
dx   -2.0 .. +2.0 m, step 0.5
dy   -1.5 .. +1.5 m, step 0.5
yaw  -40  .. +40 deg, step 10
```

For faster debugging, override the grid explicitly, for example:

```bash
ros2 run agt_map_localization_benchmark agt_greenhouse_relocalization_benchmark \
  --map-package /path/to/map_package \
  --scenes /path/to/greenhouse_scenes.yaml \
  --mode basin --frames 1 \
  --dx-min -1 --dx-max 1 --dx-step 1 \
  --dy-min -1 --dy-max 1 --dy-step 1 \
  --yaw-min -20 --yaw-max 20 --yaw-step 20 \
  --run-id basin_debug_01
```

## Result fields to inspect first

`basin/basin.csv`:

- `strict_success`: <= 0.2 m 3D translation and <= 2 deg heading error
- `nominal_success`: <= 0.5 m and <= 5 deg
- `loose_success`: <= 1.0 m and <= 10 deg
- final XY / yaw error, native fitness / overlap and Hessian eigenvalues

`global/global.csv`:

- `coarse_xy_error_m`, `coarse_yaw_error_deg`: raw BBS coarse-pose error
- `coarse_seed_gicp_nominal_success`: **direct BBS seed -> LOCAL GICP basin hit**
- `final_nominal_success`: integrated GLOBAL backend final success
- `candidate_patch`, `candidate_row_id`, `wrong_row_candidate`
- BBS score / elapsed time and descriptor diagnostics already exposed by the
  native CLI

The most useful comparison is by `scene_type` in `summary.json`:

```text
row_middle  : basin success / BBS-seed basin-hit / final global success
row_end     : basin success / BBS-seed basin-hit / final global success
headland    : basin success / BBS-seed basin-hit / final global success
row_entry   : basin success / BBS-seed basin-hit / final global success
```

If headland basin-hit is much higher than row-middle basin-hit, that is direct
support for event-triggered relocalization at the row end instead of continuous
global matching in repetitive row interiors.

## Rosbag workflow

Keep rosbag replay and benchmark responsibilities separate:

```text
raw MID360 rosbag
    -> existing run_mid360_mapping.sh --reference fastlio
    -> FAST-LIO2-only fastlio_reference_package
    -> manually label representative keyframes
    -> greenhouse benchmark
    -> basin/global CSV + summary
```

Do not make this benchmark secretly rerun SLAM. Rebuilding the map and
re-evaluating relocalization should remain two reproducible steps.

## Tests

```bash
PYTHONPATH=benchmarks/agt_map_localization_benchmark \
  python3 -m pytest -q benchmarks/agt_map_localization_benchmark/test
```
