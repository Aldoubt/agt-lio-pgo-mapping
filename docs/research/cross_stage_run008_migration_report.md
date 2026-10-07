# Cross-stage run008 migration report

**Status: PARTIAL — DRAFT research assets are reusable; human ROI/frame review and experiment acceptance remain open.**

The durable project is `/home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/project.json`. It references, and does not copy, the two source map packages, rosbag directories, and run008 results. `migrate_run008.py` refuses to overwrite an existing output directory. Annotation edits save to the persistent `annotations/annotation_v1.json`, not to run008.

The migrated session records are:

| session_id | Source | Acquisition platform | Growth stage | Source map SHA-256 |
|---|---|---|---|---|
| `session_green_house` | `green-house` map package | `HANDHELD`, `USER_DECLARED` from the user's explicit statement | `UNKNOWN` | `d9fb4c866ce4c5f49c0b2a0d447bf47ef532362dd88af8b710a6c39c0e23e0a8` |
| `session_white_tomato_collect_20261006_080252` | white-tomato map package | `VEHICLE`, `USER_DECLARED` by the user's 2026-10-07 clarification | `SPARSE`, `USER_DECLARED` from the prior user description, not inferred from the filename | `ac6e1f97d37c1b4855381dd25b9047cfded36d617fa01c16310dd34ca82c5482` |

The original migration recorded the comparison platform as `UNKNOWN`. The persistent research session manifest has since been amended from the user's explicit statement that this map was collected from a vehicle-mounted LiDAR. This metadata amendment does not modify the source bag, map package, alignment, or any run008 output.

The reused alignment is `alignment_run008_approximate_001`, with direction `T_target_from_source`: the white-tomato map is source, `green-house` is target, and the stored matrix transforms comparison points into the target map. Its status remains `APPROXIMATE` / `DRAFT`; its method and byte hash point to the original run008 alignment YAML. No SLAM, global GICP or alignment update was performed.

The purple greenhouse-boundary candidate and navigation-interior candidate were copied as numeric geometry into the canonical Research Annotation JSON V1 document. Both have `DRAFT` review status, `PROVISIONAL` candidate status, and both source session IDs. They were not promoted to final boundaries or paper truth. The candidate YAML declares `map_reference/camera_init`, while the migrated annotation and alignment contract label the frame `UNVERIFIED/camera_init`. This frame-name relationship still needs manual confirmation. The MapStudio status panel keeps the uncertainty visible; a reviewer should resolve it before freezing or using these coordinates in statistics.

The run008 result contract reports `paper_statistics_eligible: false`. Its ROI revision is `run008-greenhouse_roi.yaml:candidate-001`, which does not match the V1 annotation revision. The reference session growth stage is unknown; the alignment and result are not frozen. The independent validator currently returns PASS for bundle structure and hashes, while its paper-statistics gate returns false with those blockers.

Before this migration, a SHA-256 inventory of every run008 file was captured at `/tmp/agt_run008_before.sha256`. Final verification uses that inventory to confirm all run008 files remain byte-identical. Source PCDs are also re-hashed by the independent validator and checked by the GUI before display.

The GUI interaction smoke test passed on an X11 desktop using a temporary copy of this project: it created a four-vertex polygon, dragged one vertex, deleted one vertex, undid that deletion, saved, exited, and reloaded with the edited map coordinates unchanged. Its result remained ineligible for paper statistics because its status was DRAFT. The screenshot is `evidence/mapstudio_annotation_v1.png` in the persistent project.

No physical ROI boundary, semantic region, fixed structure correspondence, or metric accuracy is confirmed by this migration. The annotation workflow is ready for a person to create/edit polygons and capture stable structure regions using existing 3D selection geometry; correspondence residual analysis and final research acceptance remain follow-on work.
