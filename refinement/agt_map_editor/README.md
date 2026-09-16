# agt_map_editor

RViz interactive editor for Phase 2 refinement rules.

The recommended workflow is one command. It opens RViz, loads the source
point cloud, and automatically exports the derived package whenever `SAVE` is
clicked:

```bash
~/ros2_ws/src/agt_mapping_framework/scripts/edit_map.sh \
  ~/ros2_ws/experiments/artifacts/bunker_mid360_v003/map_package \
  ~/ros2_ws/experiments/artifacts/bunker_mid360_v003/refined_map_package
```

The original source package is never modified. The output package may be
regenerated in place because it is a derived artifact.

For manual launch with an existing rule file:

```bash
ros2 launch agt_map_editor map_refinement_editor.launch.py \
  refinement_file:=/path/to/refinement.yaml \
  map_pcd:=/path/to/map_package/map.pcd
```

In RViz, add a `PointCloud2` display with topic
`/map_refinement_editor/map_cloud` and an `InteractiveMarkers` display for topic
`/map_refinement_editor/update`. Drag polygon vertices or the two box corner
handles in the `map` frame. Click `ADD BOX` or `ADD POLYGON` to create a new
operation, then drag its handles. Click the visible `SAVE` marker, or call the
`/map_refinement_editor/save_refinement` service, to write the YAML file.

This MVP edits existing operations. It intentionally does not alter the
source PCD. The one-command launcher calls `agt_map_refinement_core` after
saving to publish a new refined package containing the edited PCD, PGM, YAML,
metadata, manifest, and checksums.