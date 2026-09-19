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

## Round 2A boundary

This release implements point selection and non-destructive editing only. It
does not generate PGM/Nav2 maps, run ROS nodes, filter dynamically, or modify
FAST-LIO2/PGO and the mapping pipeline.

Round 2B can add a PCD-to-PGM baseline exporter and a 2D occupancy-map editor.
