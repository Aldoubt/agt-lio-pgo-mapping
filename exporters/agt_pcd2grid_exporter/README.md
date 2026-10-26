# agt_pcd2grid_exporter

Offline mapping-package/PCD to occupancy-grid converter for AGT map assets.
The default package mode reconstructs paired body-frame patches and poses, removes
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
- grows a ground surface from the recorded trajectory with a bounded step and
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

## Bound cleanup review output

To retain the temporal decision for inspection, add `--cloud-output clouds` to
the traversability command. `--config` must be explicit. Relative cloud paths
are resolved below `--output`; absolute paths must also be strict descendants
of `--output`. A separate external cloud directory is rejected because two
directories cannot be published as one atomic transaction.

```bash
ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
  --package /path/to/mapping_package \
  --config /path/to/measured_robot_projection.yaml \
  --output /path/to/new_navigation_review \
  --cloud-output clouds
```

The entire new output directory is staged and published once with no replacement
of existing files or even an empty existing directory. Output inside the source
package is rejected. Failed generation removes its staging directory. The
installed `agt_mapping_artifacts` package verifies the existing source manifest
and checksums, checks again before publication, and writes the existing artifact
manifest/checksum format for the complete result. Missing authority dependencies
or unsupported source manifest layouts fail closed.

`metadata.yaml` binds every source file, the selected `--config` file copied as
`input_projection.yaml`, the effective `projection.yaml` including robot geometry,
and the exporter package version. The selected config is rechecked before publish.
This is a review result, not a
READY Site/Route or proof of an executable robot Profile. The bundled example
projection remains Bunker-specific; select measured geometry and extrinsics for
another chassis explicitly.

`clouds/static_map.pcd` retains persistent world observations and all classified
ground; `clouds/removed_candidates.pcd` retains the other candidates. Both carry
XYZ only. The retained cloud can include stable points above the collision band
and is not the exact accepted-point set of the grid. It is not a paired mapping
source and cannot replace only `map.pcd` while leaving original patches behind.
For manual 3D deletions, `agt_map_refinement_core` applies the same map-frame rules
to each paired patch and rebuilds the merged map from retained observations.

The temporal test uses keyframe count/index separation, not elapsed time or a
semantic motion classifier. A static object seen only nearby can be removed;
an object that stays still long enough can remain. Ground retention and the
trajectory footprint sweep remain deliberate traversability policies. Review
these decisions for the scene before using derived maps for route authoring.
