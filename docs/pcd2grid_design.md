# PCD to Occupancy Grid Baseline Design

## Scope

Round 2B-1 adds an offline, deterministic PCD-to-occupancy-grid baseline. It
does not modify `backends/`, `core/`, existing map-package production,
FAST-LIO2, PGO or Nav2 runtime. The exporter consumes an already written PCD
and creates a new `navigation_map/` directory.

## Package and integration

```text
exporters/agt_pcd2grid_exporter/
├── include/agt_pcd2grid_exporter/
├── src/
├── config/projection.yaml
└── test/
```

The package provides a small C++ library and an offline CLI. `agt_map_studio`
links to the library and exposes `Tools -> Generate Occupancy Map`; it does not
link the exporter into any ROS navigation node.

## Input and output

Input is a PCD containing finite `x`, `y` and `z` fields. `intensity` and every
other field remain in the loaded source object but are intentionally not used
for the first projection baseline.

Output:

```text
navigation_map/
├── map.pgm
├── map.yaml
├── projection.yaml
└── metadata.yaml
```

`map.yaml` uses Nav2 map-server compatible keys: `image`, `resolution`,
`origin`, `occupied_thresh`, `free_thresh`, `negate` and `mode: trinary`.

## Projection algorithm

For every finite source record accepted by the Z filter:

```text
gx = floor((x - origin_x) / resolution)
gy = floor((y - origin_y) / resolution)
```

With `origin.auto: true`, the origin is the resolution-aligned floor of the
minimum accepted X/Y. Grid dimensions are derived from the maximum accepted
X/Y. Each cell stores only a `hit_count`; no ray tracing or free-space
inference is performed. This makes the baseline easy to explain and avoids
inventing obstacle-free space from an unstructured point cloud.

Occupancy values are:

| hit count | occupancy | PGM byte |
| --- | --- | --- |
| `>= occupied_threshold` | occupied (`100`) | `0` |
| `0`, `empty_cell: unknown` | unknown (`-1`) | `205` |
| `0`, `empty_cell: free` | free (`0`) | `254` |

The default is `unknown`, which is conservative for a surface-only PCD.
There is no obstacle inflation and no Nav2 planner invocation.

## Parameters

`config/projection.yaml` contains:

```yaml
resolution: 0.05
origin:
  auto: true
  x: 0.0
  y: 0.0
z_filter:
  min: -0.3
  max: 1.5
occupied_threshold: 3
empty_cell: unknown
```

`z_filter` is inclusive. `resolution` and `occupied_threshold` must be
positive; invalid or empty filtered inputs are reported as errors.

## Coordinate convention

The grid origin is the lower-left world XY corner. `gy=0` is the lowest world
row. PGM stores rows from top to bottom, so the writer vertically flips rows
without changing the YAML origin. The map yaw is zero and the YAML origin is
`[origin_x, origin_y, 0.0]`.

## Performance and field policy

The PCD source byte buffer is traversed once. Accepted XY pairs are buffered
only until grid dimensions are known, then hit counters are filled in a second
pass over those compact pairs. No Qt object is created per point. The writer
uses a streaming binary PGM output and stores only one byte per grid cell.

## Preview and tests

Map Studio displays the effective resolution and Z limits before running, then
shows a scaled `QImage` preview after the output package is written. Unit
tests cover empty input, single-point projection, multiple points sharing a
cell, Z filtering, PGM/YAML output, and Nav2 key presence. A MID360 artifact is
used for an offline integration run; no real hardware or ROS runtime is
started.

## Round 2B-1 boundary

This exporter is a baseline asset generator only. It does not implement 2D
selection/editing, dynamic filtering, PGM editing, obstacle inflation,
ray-traced free space, Nav2 launch or ROS topic communication. Those belong to
future benchmark/editor work.
