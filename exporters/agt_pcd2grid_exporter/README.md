# agt_pcd2grid_exporter

Offline PCD-to-occupancy-grid baseline for AGT map assets. It projects finite
XYZ records into an XY grid, applies a Z range, counts hits per cell, and
writes a Nav2-compatible `map.pgm` and `map.yaml`.

```bash
ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
  --pcd /path/to/clean_map/map.pcd \
  --config $(ros2 pkg prefix agt_pcd2grid_exporter)/share/agt_pcd2grid_exporter/config/projection.yaml \
  --output /path/to/navigation_map
```

The output also records the effective `projection.yaml` and generation
metadata. Empty cells default to unknown because this baseline does not infer
free space or perform ray tracing.
