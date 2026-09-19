# `agt_map_studio` Round 2A Design

## 1. Scope and audit result

Round 2A upgrades the existing Qt/OpenGL viewer into a non-destructive point-cloud editing tool. The source PCD remains read-only; edits are represented by point state and are exported to a new `clean_map/` directory.

The audit of the current Round 1 implementation found:

| Area | Current implementation | Round 2A insertion point |
| --- | --- | --- |
| Point-cloud cache | `PointCloudViewer::cloud_` stores `LoadedPointCloud`; `LoadedPointCloud::source` keeps the full `pcl::PCLPointCloud2`, while `xyz` is the GPU render cache | Add `source_indices` so render points map back to original PCD records |
| Point fields | `PCDLoader` loads the complete `PCLPointCloud2` and separately extracts XYZ/intensity | Keep this object immutable during editing; export selected source records |
| OpenGL rendering | `PointCloudViewer::initializeGL/paintGL` uploads XYZ to a VBO and renders with a shader | Add a status VBO; shader selects height/selected/deleted color |
| Camera transform | `CameraController` supplies view/projection matrices; viewer uses MVP in `paintGL` | Reuse the same MVP to project screen selection into point candidates |
| Qt events | Viewer handles keyboard, mouse drag and wheel events; `MainWindow` owns menus | Add Navigate/Select/Delete modes, rectangle overlay, toolbar and edit actions |
| GUI state | `MainWindow` owns viewer and menu actions; status bar shows file/points/FPS | Add `SelectionManager` as the edit-state owner and expose edit counts |

No change is required in `backends/`, `core/`, `exporters/`, `interfaces/`, FAST-LIO2, PGO or existing map-package production.

## 2. Point State Layer

The immutable point data and mutable edit state are separate:

```text
LoadedPointCloud
├── source: full PCLPointCloud2, immutable during a session
├── xyz/intensity: render caches
└── source_indices: render index -> original PCD record

SelectionManager
└── status[i]: VISIBLE | SELECTED | DELETED
```

The application never erases points from `xyz`, `source` or `source_indices`. Deleted points remain in the viewer and are rendered red for review. Export filters source records only at the output boundary.

## 3. Selection module

New files:

```text
apps/agt_map_studio/src/selection/
├── SelectionBox.h/.cpp
└── SelectionManager.h/.cpp
```

### `SelectionBox`

Stores the normalized screen-space `QRect` for the current drag. It provides the rectangle bounds used by the viewer's camera projection. The first version is axis-aligned in screen space and does not support a rotated selection box.

### `SelectionManager`

Owns:

- point statuses
- current selected indices
- current selection AABB
- delete command undo/redo stacks
- durable operation history records
- clean-map export

The viewer performs the camera projection for each finite XYZ point. Points whose projected screen coordinate is inside the rectangle are passed to `SelectionManager`, which records their world-space axis-aligned bounding box. Deleted points are excluded from subsequent selection.

This is intentionally a screen-rectangle selection with a world AABB evidence record. It does not pretend that a 2D rectangle uniquely defines a 3D volume behind the camera; the selected point set and its measured AABB are the reproducible selection result.

## 4. Modes and interaction

```text
Navigate (default) -- N
Select              -- S
Delete              -- D
```

In Navigate mode, the existing camera controls remain active. In Select or Delete mode, left-drag draws a screen rectangle and release commits selection. Selected points are yellow.

Delete mode additionally accepts `Delete` to execute a `DeleteBoxCommand` over the current selected set. Deleted points remain visible in red.

Existing camera controls remain available except that `S` is reserved for the required Select mode shortcut. Backward camera movement is also exposed through the Down arrow in Round 2A.

| input | action |
| --- | --- |
| Left drag in Navigate | orbit |
| Right drag | pan |
| Wheel | zoom |
| W / Down | forward / backward |
| S | Select mode |
| A | left |
| D | Delete mode |
| Q / E | down/up |
| N | Navigate mode |
| Shift | fast movement |
| R | reset camera |
| 0 / 1 / 2 | isometric / front / top view |
| Delete | delete current selection in Delete mode |
| Ctrl+Z / Ctrl+Y | undo / redo |

Because `S` and `D` are mode shortcuts, backward movement in Navigate mode uses
the Down arrow; this preserves the requested mode API without making mode
selection ambiguous.

## 5. Command pattern

`DeleteBoxCommand` stores:

- affected point indices
- pre-command statuses
- selection AABB
- operation id and timestamp

`redo` marks affected points `DELETED`; `undo` restores their pre-command statuses. A new delete after undo clears the redo stack. Undo/redo changes state only and does not mutate the source PCD.

## 6. Export contract

`File -> Export Clean Map` creates a new directory selected by the user:

```text
clean_map/
├── map.pcd
├── edit_history.yaml
└── metadata.yaml
```

The exporter copies only records with status `VISIBLE` or `SELECTED` (both are non-deleted) from the original `PCLPointCloud2`. It preserves the original field list, datatype, offsets, point step and all extra fields. `intensity`, `ring` and future fields are not reconstructed from XYZ.

`metadata.yaml` records source path, source point count, visible/deleted counts, field names and export timestamp. `edit_history.yaml` records all delete-box operations and whether an operation is currently undone.

The source `map_package` is never opened for writing and no existing map-package file is overwritten in place.

## 7. GUI changes

Add a toolbar with checkable actions:

```text
[ Navigate ] [ Select ] [ Delete ]
```

Add menu actions:

- Edit / Delete Selected
- Edit / Undo
- Edit / Redo
- File / Export Clean Map
- View / Isometric, Front, Top

Status bar:

```text
Total points | Deleted points | Visible points | Current mode | FPS
```

The OpenGL overlay retains the existing FPS/file display and adds the selection rectangle while a select/delete drag is active.

## 8. Validation plan

Use an existing artifact such as:

```text
/home/yangxuan/ros2_ws/experiments/artifacts/output/mid360_20260901_205036/map_package/map.pcd
```

Required checks:

1. Load and render the source PCD.
2. Switch to Select and drag a rectangle.
3. Verify selected points turn yellow and deleted points remain unchanged.
4. Switch to Delete, press Delete, verify red points and counts.
5. Ctrl+Z restores the prior state.
6. Ctrl+Y reapplies deletion.
7. Export `clean_map/`.
8. Reload `clean_map/map.pcd` and verify exported point count is lower and the deleted records are absent.
9. Run unit tests for state transitions, undo/redo, full-field PCD export and history serialization.

Round 2B remains outside this change: PCD-to-PGM baseline export and 2D occupancy-map editing.
