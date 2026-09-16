# agt_map_editor

RViz interactive editor for Phase 2 refinement rules.

Start it with an existing rule file:

```bash
ros2 launch agt_map_editor map_refinement_editor.launch.py \
  refinement_file:=/path/to/refinement.yaml
```

In RViz, add the `InteractiveMarkers` display for topic
`/map_refinement_editor/update`. Drag polygon vertices or the two box corner
handles in the `map` frame. Click the visible `SAVE` marker, or call the
`/map_refinement_editor/save_refinement` service, to write the YAML file.

This MVP edits existing operations. It intentionally does not alter the
source PCD; run `agt_map_refinement_core` after saving to publish a new refined
package.