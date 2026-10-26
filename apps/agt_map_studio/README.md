# agt_map_studio

Round 1 offline point-cloud viewer for the AGT mapping framework.

## Features

- Load ASCII, binary and binary-compressed PCD files through PCL.
- Keep the complete `PCLPointCloud2` field set in memory, including `intensity` when present.
- Render XYZ points with Qt5/OpenGL.
- Color points by Z height by default: blue is low and red is high.
- WASDQE camera movement; hold Shift for `fast_speed`.
- Left mouse orbit, right mouse pan, wheel zoom.
- Point-size, background, axis, reset-camera and FPS controls.
- `View -> Color by Z Height` toggles height coloring; the legend shows the Z range.
- Save camera/view state to a separate YAML file.
- Non-destructive Round 2A editing: box selection, point-state colors, DeleteBox
  undo/redo and clean-map export.
- `Tools -> Generate Occupancy Map` runs the offline PCD-to-PGM baseline with
  configurable resolution and Z filtering, then shows a 2D preview.
- `File -> Open Occupancy Map` loads a Nav2 `map.yaml` and its relative P2/P5
  PGM into the independent read-only 2D viewer.
- Round 2B-2B refinement editor: erase rectangles, obstacle lines, forbidden
  polygons, sparse undo/redo and traceable refinement export.

This is an offline viewer. It does not create a ROS node, subscribe to topics,
modify a `map_package`, or participate in FAST-LIO2/PGO/mapping execution.

## Build

The current environment has Qt 5.15.3 rather than Qt6, so the CMake file uses
Qt5 automatically and does not install anything.

```bash
cd /home/yangxuan/ros2_ws
colcon build --base-paths src/agt_mapping_framework/apps/agt_map_studio \
  --packages-select agt_map_studio
source install/setup.bash
```

## Run

```bash
ros2 run agt_map_studio map_viewer \
  --pcd /home/yangxuan/ros2_ws/experiments/artifacts/output/mid360_20260901_205036/map_package/map.pcd
```

The executable also accepts a positional path:

```bash
ros2 run agt_map_studio map_viewer /path/to/map.pcd
```

`File -> Save View` writes camera state only; it never writes back to the
source map package.

## Interaction

| input | action |
| --- | --- |
| Left mouse drag | Orbit/rotate camera |
| Right mouse drag | Pan camera |
| Mouse wheel | Zoom |
| W / Down arrow | Move forward/backward |
| A/D | Move left/right |
| Q/E | Move down/up |
| Shift | Fast movement |
| R | Reset camera |
| N/S/D | Navigate/Select/Delete mode |
| Delete | Delete selected points in Delete mode; deleted points remain red |
| Ctrl+Z / Ctrl+Y | Undo / redo the latest delete-box operation |
| 0/1/2 | Isometric / front / top view |
| `+/-` | Increase/decrease point size |

In Select or Delete mode, drag the left mouse button to create an axis-aligned
screen selection. Selected points are yellow. In Delete mode press Delete to
mark them red; the source PCD is never erased or overwritten.

The `Help -> Controls` menu shows the same shortcuts inside the viewer.

`File -> Export Clean Map` asks for a parent directory and creates a new
`clean_map/` directory containing:

- `map.pcd`: only non-DELETED points, with the original PCL fields preserved;
- `edit_history.yaml`: delete-box operations and undo state;
- `metadata.yaml`: source, point counts and exported field names.

`Tools -> Generate Occupancy Map` uses the installed
`agt_pcd2grid_exporter/config/projection.yaml`, displays the effective
parameters for confirmation, and creates a separate `navigation_map/`
directory containing Nav2-compatible `map.pgm`, `map.yaml`, the effective
projection parameters and metadata. It never overwrites the source PCD.

In the 2D occupancy view, left-drag pans, the wheel zooms around the cursor,
`R` resets to a centered 1:1 view, and `F` fits the whole map. The status bar
shows the grid pixel, lower-left grid coordinate and world XY coordinate under
the cursor. Use the occupancy toolbar for the refinement operations described
below.

## Occupancy refinement

The occupancy toolbar provides four modes:

- `View`: pan and zoom only;
- `Erase`: drag a rectangle, confirm, and convert occupied cells to free;
- `Obstacle`: drag a line; `Width (m)` controls its rasterized width and free
  cells become occupied;
- `Forbidden`: click polygon vertices and double-click to finish; the zone is
  stored as a translucent red layer without changing occupancy values. `Esc`
  cancels the in-progress polygon.

`Ctrl+Z` and `Ctrl+Y` undo/redo 2D operations while the 2D page is active.
`File -> Save Map Refinement` writes `map_refinement.yaml`. `File -> Export
Navigation Map` writes a new `navigation_map/` directory containing `map.pgm`,
Nav2-compatible `map.yaml`, `map_refinement.yaml` and `metadata.yaml`. The
original base PGM is never modified.

## Phase 3A: geometry sidecar inspection (read-only)

Open the original optimized PGO package first, then its matching V1 spatial
confidence derivative. Next choose **File → Open Geometry Evidence Sidecar
(read-only)**, or start with `--package <parent> --spatial-confidence <v1>
--spatial-geometry <geometry_sidecar>`. Core validates the sidecar's three
files, parent checksum provenance, the full V1 derivative, exact VoxelKeys
and center/voxel-size correspondence. A failed load retains the previous
Studio model and unsaved V1 edit intent.

**View** adds five read-only geometry coloring modes: PCA normal shape (RGB =
linearity, planarity, scattering), Ht directional Q, Hr rotational Q, Ht
weak-axis, Hr weak-axis (RGB = absolute map x/y/z of the weak eigenvector).
Neutral gray means invalid/insufficient evidence, **not Q=0**. The existing
voxel selection and inspector show local PCA support, covariance eigenvalues,
shape, Ht/Hr separate eigenvalues, weak directions, Q, condition and support
counts. Raw `map.pcd` point indices remain separate from voxel indices.
This view never changes V1 geometry_score/auto_confidence, saved override
intent, stable selection, or any source file; no geometry-edit or map-publish
path is added. Ht and Hr have different units and are not combined into a
probability or location-performance claim.


## Durable local drafts

`Ctrl+S` saves a Studio session and new UUID-named 2D/3D edit files. The
session commit is atomic; an interrupted save leaves its prior referenced
files intact. Opening a session checks the source PCD, base YAML/image and
edit-file checksums, then strictly replays 2D edits in an isolated model before
changing the current view. Undo/redo works on recovered 2D edits. A standalone
2D draft does not inherit the previous session's point cloud or 3D edits.

Saved 3D refinement rules remain explicit **manual reapplication required**;
this implementation does not reconstruct their point selection. These local
files are `DRAFT_NOT_READY_ROUTE` and do not publish or activate a map.

## Offline localization studies

`Tools → Offline studies / 离线重定位研究` opens an independent authoring dialog:

1. Create/open a Project bound to a verified source, block set and optional
   physical topology. Save/rebind records exact dependencies through the CLI.
2. Create manual Query Sets by XY or a map click, or sample frozen reviewed
   physical rows. Inspect resolved source keyframes and snap distances;
   add/enable/disable/delete draft queries, clone or freeze a new revision.
3. Create a Study with explicit accumulation frames and candidate Top-K. Run,
   cancel, validate and export its existing report format. The ROS install
   overlay is explicit and used for both execution and algorithm validation.
4. Filter results by classification/scene/frames and open existing evidence.
   Map markers require the exact loaded source. `STALE`/`INVALID` validation
   clears markers even when the validator process exits with status zero.

The optional `agt_mapstudio_workflow` command is discovered on PATH or through
its installed ROS package. Its absence does not prevent local editing. The
existing backend owns identity validation, query generation and classification;
Studio does not duplicate localization algorithms. A running job retains its
captured Study path. Cancellation allows a configurable cleanup grace (default
10 seconds). Source/dependency changes invalidate previews. Result labels and
same-session references are not independent ground truth or robot execution
acceptance. Current language settings remain owned by MainWindow; the publish
panel shares its existing display catalog.

`View → Rendered points / 显示点数` selects a deterministic display cap (default
500,000; `Full` is available). Z filtering precedes the display sample. All
editing and export retain the complete cloud and original record indices.
Study results and cross-session Comparison are separate layers, with separate
colors and opacity; annotation editing and coverage authoring remain available.

## Focused acceptance

`test_study_workflow_dialog` exercises Qt command lifecycle, argument handling,
missing optional tools, stale validation and fixed running-job identity with a
**test command fixture**, not a localization implementation.
`study_workflow_dialog_smoke PROJECT_YAML SCREENSHOT_PNG` instead drives the
actual installed workflow CLI using a disposable verified Project. It creates
manual queries, freezes them, creates/validates a Study and samples five points
from a reviewed row. It does not run the native localization algorithm.

With `AGT_BUILD_GUI_ACCEPTANCE=ON`, `s1_gui_draft_acceptance` tests real Qt draft
save/reopen, tamper rejection, interrupted saves, unreplayed 3D intent and
standalone 2D isolation. Run GUI/renderer acceptance on the target display;
offscreen tests do not establish OpenGL performance or field behavior.
