# MapStudio Annotation Workflow V1

> Current interface instructions are in [MapStudio UX Cleanup V1](mapstudio_ux_cleanup_v1.md). The button names and panel layout below describe the original implementation and are retained as contract/migration history; do not use them as the current UI guide.

## What is reused

The workflow extends the existing Qt `map_viewer` / `PointCloudViewer`. It reuses the viewer's top view, map-coordinate picking, point selection, polygon-prism, box and sphere selection geometry, undo stack, and auxiliary point-cloud rendering. The research annotations remain non-destructive; selecting a region never deletes or rewrites source points.

Existing polygon support was split across other workflows: the 2D occupancy editor has polygon edits; `agt_greenhouse_annotation` edits row centerlines, headlands and scene markers under Topology Schema v1; the PCD viewer has polygon-prism selection for point deletion. None of those was a persistent cross-session ROI project. The new GUI stores cross-stage annotations only as Research Annotation JSON V1. Cross-stage analysis remains an offline consumer of imported candidates and saved evidence; it is not used to alter the saved alignment or run GICP.

The original implementation focused on BEV polygons and reusing 3D selections. The current UX Cleanup V1 GUI also exposes the existing Annotation JSON V1 polyline and point geometries through the type-driven draw workflow; it continues to reuse the same 3D viewer and stable-structure selection geometry.

## Open the two-session research project

Build into a temporary prefix so the normal workspace install is left alone:

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install_mapping_framework/setup.bash
colcon build --base-paths src/agt_mapping_framework/apps/agt_map_studio \
  --packages-select agt_map_studio \
  --build-base /tmp/agt_mapstudio_annotation_build \
  --install-base /tmp/agt_mapstudio_annotation_install
source /tmp/agt_mapstudio_annotation_install/setup.bash
ros2 run agt_map_studio map_viewer --research-project \
  /home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/project.json
```

The same project can be opened from **File → Open Research Annotation Project** or the bilingual **Research Annotation / 研究标注** dock. Before display, MapStudio checks each source PCD SHA-256, session identity, source/target hash binding, exact `T_target_from_source` direction, SE(3), annotation dataset/reference frame, and annotation source hashes. The target session is primary; the source session is transformed with the saved matrix into that target frame. No registration is run. Each view uses a deterministic display-only sample capped at 1.2 million points; the raw PCD bytes remain external and read-only. Use the checkbox and opacity slider to compare the sessions.

The dock displays the annotation reference frame and the primary map frame. In this migrated experiment those names carry explicit uncertainty qualifiers; matching numeric coordinates do not promote the alignment or frame alias to verified truth.

## Create and review annotations

Keep the confirmed greenhouse boundary as the outer polygon. For the follow-up aisle study, generate or draw `Aisle` polygons for row corridors, `Obstacle Region` polygons for obstacle footprints, and `Traversable Region` polygons only where a person has reviewed the ground area as passable. Automatic aisle proposals use the primary/reference point cloud and the saved boundary's BEV footprint; they do not classify obstacles or make traversability decisions. The new labels use Annotation V1's existing `custom` type and `MapStudio UI type` note marker, so the V1 schema stays unchanged. `Navigation Interior` is now an optional legacy analysis ROI; keep the imported run008 candidate only for provenance, and do not treat it as a required navigation map. The existing `Centerline`, `Row Entrance`, and `Aisle End` point annotations remain available for later aisle-level analysis.

### Fast path: draw and revise aisles

In the Annotation workspace, the type selector now defaults to **Aisle** and places **Topology** first. The contextual **Auto Extract Aisle + Ends** button uses the primary/reference cloud plus the saved Greenhouse Boundary to estimate the row direction from supported parallel density ridges, then proposes corridors between adjacent ridges. It displays the estimated direction and score separation; weak or ambiguous directions are rejected. It creates one `DRAFT` / `PROVISIONAL` aisle polygon and two `DRAFT` / `PROVISIONAL` `Aisle End` points per proposal. The Z interval defaults to the Annotation toolbar values; profile resolution, minimum row spacing, estimated row half-width, side clearance, minimum/maximum width and minimum length are configurable in the dialog. The batch records source session, algorithm version, orientation diagnostics, input point counts and parameters in the existing annotation notes. It stays in memory until Save and a single Undo removes the complete batch. Select an aisle in **OBJECTS**; its contextual **Edit Aisle** button enables dragging vertices, and Delete removes the selected object. Inspect and revise every proposal and endpoint before Review; these candidates are not physical ground truth, obstacle classification, or a traversability decision.

Manual annotation remains available: click **Draw Aisle**, outline one corridor with polygon vertices, then press Enter or right-click to finish.

Choose a polygon type and click **绘制 BEV 多边形 / Draw polygon**. MapStudio switches to top view. Click to add map-frame XY vertices, press Enter or right-click to close, Backspace removes the last pending vertex, and Escape cancels. Select an annotation in the list to highlight it. Drag a vertex to move it; click a vertex and use **删除选中顶点 / Delete vertex** to remove it (a polygon keeps at least three vertices). **Delete** removes the selected annotation feature. The annotation Undo/Redo buttons track geometry and feature changes.

For stable structures, click **使用现有 3D 选区 / 3D select**, use the existing toolbar selection mode/tool and optional Z window, choose `Fixed Frame Region`, `Fixed Column Region`, or `Stable Structure ROI`, then click **保存选区为结构 ROI / Capture ROI**. This records the current selection geometry in the annotation document without applying a point deletion. These saved shapes are available for later registration-residual analysis; that metric consumer is outside this release.

Click **保存草稿 / Save DRAFT** to atomically write the persistent annotation JSON path shown by the project. Closing the GUI or switching projects with unsaved edits prompts to save, discard, or cancel. Mark each selected feature `REVIEWED` only after human review; the GUI records reviewer and time. **FROZEN** is available only after every feature is reviewed and permanently locks the current annotation revision. Freezing annotations alone does not make run008 eligible for paper statistics; the alignment and result gates remain independent.

The run008 greenhouse boundary and navigation-interior candidates have already been imported into the persistent annotation document as `DRAFT`/`PROVISIONAL`. The import button is idempotent, verifies the candidate file hash and both source map hashes, and refuses duplicate candidate IDs. It never writes run008.

## Reproducible checks

```bash
cd /home/yangxuan/ros2_ws/src/agt_mapping_framework
PYTHONPATH=tools/research_asset_contract pytest -q tools/research_asset_contract/test
python3 tools/research_asset_contract/validate_bundle.py \
  /home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/project.json
```

For the C++/Qt tests, source `/opt/ros/humble/setup.bash` and `install_mapping_framework/setup.bash`, build into the temporary directories above, then run `ctest --test-dir /tmp/agt_mapstudio_annotation_build/agt_map_studio --output-on-failure` from that sourced shell.

On an active X11 desktop, the GUI interaction smoke test opens a temporary copy of the persistent project and uses XTest to create a BEV polygon, drag a vertex, delete a vertex, undo, save, exit and reload. It does not write to the durable project or run008:

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install_mapping_framework/setup.bash
source /tmp/agt_mapstudio_annotation_install/setup.bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  XDG_RUNTIME_DIR=/run/user/1000 QT_QPA_PLATFORM=xcb QT_OPENGL=desktop \
  PYTHONPATH=src/agt_mapping_framework/tools/research_asset_contract \
  python3 src/agt_mapping_framework/tools/research_asset_contract/gui_smoke_x11.py \
    --mapstudio /tmp/agt_mapstudio_annotation_install/agt_map_studio/lib/agt_map_studio/map_viewer \
    --project /home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/project.json
```

The captured current GUI screenshot is stored at `evidence/mapstudio_annotation_v1.png` inside the persistent research project. It shows the real pair of registered point clouds, imported provisional ROI candidates, coordinate-frame labels and the bilingual annotation controls.

## Migration evidence

See `docs/research/cross_stage_run008_migration_report.md`. The persistent project contains references and hashes, not copies, of source bags, source map packages or run008. Its output bundle is under `/home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/`; it is not stored only under run008.
