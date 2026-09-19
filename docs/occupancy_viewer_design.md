# Occupancy Viewer Foundation Design

## Scope

Round 2B-2A adds a read-only 2D occupancy-map viewer to `agt_map_studio`.
It loads a Nav2-style `map.yaml` and its PGM image, renders the cached grid,
and provides view navigation and coordinate conversion. It does not edit a
cell, write a PGM, launch Nav2, or communicate with ROS.

## Module boundary

```text
apps/agt_map_studio/
└── src/occupancy/
    ├── GridMap.hpp/.cpp
    ├── MapYamlLoader.hpp/.cpp
    └── OccupancyViewer.hpp/.cpp
```

`PointCloudViewer` remains the independent 3D editor. The main window hosts it
and `OccupancyViewer` as separate pages in a `QStackedWidget`; no 2D state or
PGM code is added to the 3D viewer.

## Data model

`GridMap` stores:

- `width`, `height`
- `resolution`
- lower-left `origin_x`, `origin_y`
- row-major `std::vector<int8_t>` cells using `-1`, `0`, `100`
- the source YAML/image paths for display

The PGM is converted once into an occupancy vector and a cached `QImage`. Zoom
and pan repaint the cache; they never reread the PGM.

## YAML and PGM

`MapYamlLoader` reads `image`, `resolution`, `origin`, `occupied_thresh`,
`free_thresh`, `negate` and `mode`. The image path is resolved relative to the
YAML file. Both Netpbm P2 ASCII and P5 binary images are supported, including
16-bit P5 samples.

For a non-negated image, occupancy probability is `(max_pixel - pixel) /
max_pixel`. Values above `occupied_thresh` become occupied, values below
`free_thresh` become free, and the interval between them becomes unknown. This
keeps the standard black-occupied, white-free, gray-unknown convention used by
the baseline exporter.

## View interaction

`OccupancyViewer` draws the cached image with a screen-space transform:

- left mouse drag: pan
- wheel: zoom around the cursor
- `R`: reset to a centered 1:1 view
- `F`: fit the complete map to the widget

The status bar reports the current image pixel and world coordinate when the
cursor is over the map.

## Coordinate contract

The grid origin is the lower-left world corner. API conversions are:

```text
x = origin_x + pixel_x * resolution
y = origin_y + pixel_y * resolution
pixel_x = floor((x - origin_x) / resolution)
pixel_y = floor((y - origin_y) / resolution)
```

PGM rows are top-to-bottom, so loading vertically flips image rows into the
lower-left grid coordinate system. The inverse conversion checks map bounds.

## GUI integration

`File -> Open Occupancy Map` opens a `map.yaml`, loads the referenced PGM and
switches to 2D. `View -> 3D Point Cloud` and `View -> 2D Occupancy Map` switch
pages without sharing edit state. The existing PCD editor menus remain
unchanged and are available on the 3D page.

## Tests and extension boundary

Unit tests cover empty maps, P2/P5 loading, threshold conversion and coordinate
round trips. A 2000x2000-class image is retained as a cached `QImage`; no
per-pixel Qt object is created during interaction.

Round 2B-2B can add line/rectangle/polygon tools and an occupancy command
history. Those edits must be introduced in a separate command/state layer and
must not leak into `PointCloudViewer`.
