# agt_map_editor

RViz interactive editor for Phase 2 refinement rules.

Start it with an existing rule file:

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
source PCD; run `agt_map_refinement_core` after saving to publish a new refined
package.