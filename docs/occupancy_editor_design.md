# Occupancy Map Refinement Editor Design

## Scope and audit

The existing application has an independent `OccupancyViewer` widget with a
cached `QImage`, a `GridMap` occupancy vector, and `MapYamlLoader` for Nav2
YAML/PGM input. The 3D editor has its own `SelectionManager`; its point
commands are not reused for 2D cells.

Round 2B-2B adds a separate refinement document and command module. It does
not modify `backends/`, `core/`, FAST-LIO2, PGO, `agt_navigation_v3`, Nav2 or
the source `map.pgm`.

```text
base map.yaml + map.pgm (read-only)
              |
              v
       RefinementModel
       ├── cell overrides
       ├── forbidden polygon layer
       ├── command undo/redo stacks
       └── operation history
              |
              +--> OccupancyViewer preview
              +--> navigation_map/ export
```

## Data structures

`RefinementModel` retains the base `GridMap` and metadata, then stores only
changed cells in an index-to-occupancy override map. A missing override means
the base cell is active. Forbidden zones are world-coordinate polygons in a
separate vector and never change occupancy values.

Each `CellChange` stores `index`, `before` and `after`. Commands therefore
record only their affected region rather than copying a 2000x2000 grid for
every operation.

## Commands

```text
occupancy/commands/
├── GridCommand.hpp/.cpp       abstract apply/undo command
├── EraseRectangleCommand.hpp/.cpp
├── DrawObstacleCommand.hpp/.cpp
└── ForbiddenPolygonCommand.hpp/.cpp
```

- `EraseRectangleCommand`: world-aligned rectangle; occupied cells become
  free, while free/unknown cells are unchanged.
- `DrawObstacleCommand`: world line plus width in meters; free cells in the
  rasterized capsule become occupied.
- `ForbiddenPolygonCommand`: world polygon; adds/removes a separate forbidden
  layer and leaves the occupancy vector untouched.

`Ctrl+Z` and `Ctrl+Y` operate on the active 2D document when the 2D page is
visible. A new operation clears the redo stack. The durable history includes
geometry, affected cell changes, timestamps and undone state.

## Interaction modes

`OccupancyViewer` has `View`, `Erase`, `Obstacle` and `Forbidden` modes.

- View: left drag pans, wheel zooms.
- Erase: drag a rectangle; release applies an erase command after confirmation.
- Obstacle: drag a line; the toolbar width field is in meters.
- Forbidden: click vertices and double-click to close the polygon; `Esc`
  cancels the in-progress polygon.

Preview colors remain free white, occupied black and unknown gray. Active and
new forbidden polygons are drawn as translucent red overlays.

## Refinement file

`map_refinement.yaml` is traceable and replayable:

```yaml
version: 1
base_map:
  yaml: /path/to/map.yaml
operations:
  - id: 1
    type: erase_rectangle
    geometry:
      min: [x0, y0]
      max: [x1, y1]
    changes:
      - pixel: [px, py]
        before: 100
        after: 0
    undone: false
```

The file records all operations, including undone entries. Loading it onto the
same base map applies only active operations and rebuilds the refinement layer.

## Export

`Export Navigation Map` creates a new directory containing `map.pgm`,
Nav2-compatible `map.yaml`, `map_refinement.yaml` and `metadata.yaml`. The PGM
is generated from the base cells plus active overrides; the source PGM is never
opened for writing or replaced.

## Test plan and Round 2B-2B boundary

Unit tests cover erase, obstacle rasterization, forbidden-layer invariance,
undo/redo, refinement save/load and export. GUI smoke tests use the real
Round 2B-1 map asset. Future work can add polygon editing handles, but must
keep the refinement layer and command boundary independent from the 3D editor.
