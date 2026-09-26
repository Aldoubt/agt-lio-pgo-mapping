# agt_pcd2grid_exporter

Offline mapping-package/PCD to occupancy-grid converter for AGT map assets.
The default package mode reconstructs optimized body-frame patches, removes
3D voxels that are not persistent across separated keyframes, estimates local
ground, classifies obstacle height relative to that ground, and applies small
component removal plus a one-cell morphological close. It writes a
Nav2-compatible `map.pgm` and `map.yaml`.

```bash
ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
  --package /path/to/mapping_package \
  --config $(ros2 pkg prefix agt_pcd2grid_exporter)/share/agt_pcd2grid_exporter/config/projection.yaml \
  --output /path/to/navigation_map
```

`--pcd` remains available for a cloud without patches, but temporal filtering
requires `--package` with `patches/*.pcd` and `poses_timed.txt`. The original
mapping package is never modified. The output records the effective
`projection.yaml` and generation statistics, including temporal retained and
removed point counts.

Ground-supported cells become free, nearby cells receive a conservative
free-space expansion, and cells without evidence remain unknown. This is not
semantic person detection: a moving person normally fails the cross-keyframe
persistence test, while a person standing in one place for several separated
keyframes can remain and must be removed manually or by a future semantic
mapping-stage filter.

## Traversability mode

`config/projection_traversability.yaml` selects `projection_mode: traversability`
(package input only; `--pcd` and the Map Studio point-cloud preview reject it).
Instead of projecting points, it builds the grid from evidence:

- drops robot self-returns inside the footprint column (`robot.base_from_body`,
  `robot.footprint`) and rejects packages whose base tilt exceeds
  `max_keyframe_tilt_deg` (diverged mapping runs);
- grows a ground surface from the optimized trajectory with a bounded step and
  classifies returns against it, so tree canopies are not mistaken for obstacles;
- carves free space along every sensor ray over grown ground; persistent hits
  always win over rays, and porous structure is kept by a keyframe-diverse
  neighbourhood support rule;
- ignores obstacle voxels only ever observed in the near field (people following
  the robot) and forces the padded footprint swept along the trajectory free, so
  the driven path stays plannable.

```bash
ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
  --package /path/to/mapping_package \
  --config $(ros2 pkg prefix agt_pcd2grid_exporter)/share/agt_pcd2grid_exporter/config/projection_traversability.yaml \
  --output /path/to/navigation_map
```

`metadata.yaml` gains a `traversability:` block and `debug/` holds the ground
source, ray-pass and height layers. Update `robot:` when the sensor mount or
footprint changes. Design, evidence and evaluation:
`docs/mcp-traversability-grid-upgrade.md`.
