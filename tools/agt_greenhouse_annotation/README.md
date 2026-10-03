# AGT greenhouse annotation

Reusable offline GUI for any **validated AGT map_package**, including a new greenhouse/session. Physical rows and headlands are entered and reviewed by a person. The tool generates labels and benchmark scene inputs; it never derives physical row IDs from trajectory segments.

Reference status is always `MANUAL_TOPOLOGY_PLUS_SAME_SESSION_FRONTEND_REFERENCE`, with `absolute_ground_truth: false`. A frontend map provides the coordinate system; the manually drawn topology identifies physical corridors. Its quality is limited by that map and the manual review.

## Build and launch

Existing Ubuntu 22.04 dependencies: PyQt5, Matplotlib, Open3D, NumPy, PyYAML, Shapely. This package adds no web framework, ROS node, or localization implementation.

```bash
source /opt/ros/humble/setup.bash
cd /home/yangxuan/ros2_ws
colcon --log-base /tmp/agt_annotation_log build --packages-select agt_greenhouse_annotation \
  --build-base /tmp/agt_annotation_build \
  --install-base /tmp/agt_annotation_install
source /tmp/agt_annotation_install/setup.bash

ros2 run agt_greenhouse_annotation greenhouse_annotator \
  --map-package /path/to/new_session/map_package \
  --output /path/to/new_session/annotation_v1/greenhouse_topology.yaml
```

Existing production installs are not overwritten. Direct source launch is also available:

```bash
PYTHONPATH=/home/yangxuan/ros2_ws/src/agt_mapping_framework/tools/agt_greenhouse_annotation \
  python3 -m agt_greenhouse_annotation.gui \
  --map-package /path/to/map_package \
  --output /path/to/annotation_v1/greenhouse_topology.yaml
```

Required map inputs are `map.pcd`, `poses_timed.txt`, `patches/`, `metadata.yaml`, and a checksum `manifest.yaml`. The timed pose convention matches AGT export: `patch timestamp x y z qw qx qy qz`. Loading verifies metadata and timed-pose hashes; freeze verifies every declared map-package file. Hashes refer to bytes, not just filenames. `map_package_hash` is SHA256 of the checksum manifest, whose entries commit to map/poses/patches.

The cloud loads in a Qt worker thread. `--voxel 0.2 --max-display-points 120000` controls display reduction; full map bytes and trajectory are preserved. Zoom/pan with the Matplotlib toolbar; disable pan/zoom before drawing. Click in Inspect mode to see nearest keyframe ID, bag timestamp, pose and topology label.

## Manual workflow

1. Choose **Draw row centerline**, left-click vertices along an actual physical travel corridor, and right-click or press Enter to finish. Enter a physical ID, width, allowed travel direction, confidence and notes. IDs start blank; the tool does not invent them. The stored vertex order defines positive s and positive-left d; the direction field records allowed motion without changing those coordinates.
2. Choose **Draw headland polygon**, click its boundary and finish; enter a physical headland ID and confidence. Polygons must be valid and have positive area.
3. Choose **Manual scene marker**, click near the trajectory and select ROW_ENTRY/ROW_MIDDLE/ROW_END/HEADLAND/OTHER. The snapped keyframe/timestamp/pose and corridor label are shown. Optional physical-row/headland associations are explicit manual choices; their default is UNKNOWN.
4. Select an annotation in the list and **Edit details / vertices** to change ID, width, direction, notes, or the JSON vertex list. In **Edit vertices**, drag a selected geometry's vertex. Delete, Ctrl+Z Undo, and Ctrl+Shift+Z Redo are supported. Row-ID edits update existing manual scene associations; deleting geometry clears those associations to UNKNOWN.
5. Save a draft with Ctrl+S. Review `keyframe_topology_labels.csv`, `scene_candidates_for_manual_review.csv`, and validation coverage. Suggested scene candidates are review aids and are never inserted into manual scenes or benchmark inputs.
6. Press **Freeze after manual review** once physical geometry and scene associations are reviewed. Freeze checks all source hashes, asks for explicit manual review confirmation, and makes the saved topology read-only. Benchmark consumers must require `status: frozen` and verify the manifest.
7. To revise a frozen annotation, choose **New draft version** and a new output directory. Each version has its own fixed-name CSV sidecars; snapshots in `annotation_versions/<sha256>/` preserve the YAML and provenance of earlier saves.

Unconfirmed geometry yields UNKNOWN. A pose belongs to a row only if its Euclidean polyline distance is within both half the manually specified width and `max_row_assignment_distance_m`. Beyond endpoints, distance to the endpoint is respected. Near-tied overlapping rows stay UNKNOWN/AMBIGUOUS. An overlapping headland can set zone HEADLAND while retaining a row only when the pose is also inside a confirmed manual row corridor; this is never a forced nearest-row association.

## Validation, freeze and export

```bash
ros2 run agt_greenhouse_annotation greenhouse_annotation_validate \
  /path/to/annotation_v1/greenhouse_topology.yaml --verify-map-files

# This flag is the annotator's assertion that review is complete.
ros2 run agt_greenhouse_annotation greenhouse_annotation_freeze \
  --topology /path/to/annotation_v1/greenhouse_topology.yaml \
  --output /path/to/frozen_v1/greenhouse_topology.yaml \
  --confirm-manual-review

ros2 run agt_greenhouse_annotation greenhouse_annotation_validate \
  /path/to/frozen_v1/greenhouse_topology.yaml --require-frozen --verify-map-files

# Export draft sidecars; frozen artifacts are verified and retained unchanged.
ros2 run agt_greenhouse_annotation greenhouse_topology_export \
  --topology /path/to/frozen_v1/greenhouse_topology.yaml
```

The validator checks unique IDs, finite geometry, row length/width, valid headland polygons, source frame/backend/commits/hashes, manual freeze review, timestamp existence and declared-keyframe correspondence, suspicious corridor overlap, rigid backend transforms, CSV keyframe coverage, label agreement and provenance sidecar hashes. It reports UNKNOWN percentage without inventing missing labels. Default overlap error threshold is 20% of the smaller corridor; keep this controlled parameter in the annotation. Validation writes `annotation_validation.json` and exits 2 on failure.

Saved artifacts:

| Artifact | Meaning |
| --- | --- |
| `greenhouse_topology.yaml` | Manual topology, scene markers, source hashes and backend transforms |
| `greenhouse_topology.geojson` | Geometry in local map metres; explicitly not georeferenced longitude/latitude |
| `greenhouse_topology.manifest.json` | YAML hash, source/backend/tool/rosbag provenance and every generated sidecar hash |
| `keyframe_topology_labels.csv` | Every canonical keyframe with `(row,s,d,psi)` or UNKNOWN; yaw stored in radians |
| `scene_candidates_for_manual_review.csv` | Geometry-based suggestions, explicitly NEEDS_MANUAL_REVIEW |
| `benchmark_scenes_<backend_id>.yaml` | Only manually chosen markers, compatible with the existing greenhouse benchmark |
| `annotation_versions/<sha256>/` | Immutable content-addressed annotation/provenance snapshot |

Benchmark scene export uses the existing runner's pose-list index in `keyframe` and preserves the PCD ID in `map_keyframe_id`. Rows are not converted to trajectory index ranges. OTHER markers, timestamps outside tolerance, and keyframes lacking a five-frame window are listed in `excluded_manual_scenes`; they are not silently moved. An empty annotation produces `NO_RUNNABLE_MANUAL_SCENES` and cannot be frozen. Use a dedicated output directory per annotation version to keep label hashes associated with the right YAML.

## Two backends from one rosbag

Choose one canonical backend and add the other map:

```bash
ros2 run agt_greenhouse_annotation greenhouse_annotator \
  --map-package /path/to/pointlio/map_package \
  --other-map-package /path/to/fastlivo2/map_package \
  --output /path/to/session/annotation_v1/greenhouse_topology.yaml \
  --correspondence-max-dt 0.5
```

The tool confirms the same rosbag source and uses nearest bag timestamps to fit a rigid SE(2) transform from other-backend XY to canonical XY. It records method, matched count, maximum timestamp difference, XY residual median/P95/max, source/target manifest hashes and coordinate frames. A residual is a frontend consistency measure, not ground-truth accuracy. No backend frame identity is assumed. `backend_transforms[backend_id].matrix` is a 4×4 other-to-canonical transform. In addition it writes `other_backend_keyframe_topology_labels.csv`, `backend_scene_correspondence.csv` and that backend's manual benchmark-scene YAML.

If saving/exporting/freezing a two-backend draft outside the GUI, pass `--other-map-package` to retain both backend sidecars. Existing frozen artifacts are verified and retained without rewriting their sidecars or provenance. If a frozen annotation lacks the other transform, create a new draft version, add and review correspondence, then freeze a new artifact.

## Geometry API for analysis

```python
from agt_greenhouse_annotation.topology import load_topology, label_pose, project_to_row
topology = load_topology('/path/to/frozen_v1/greenhouse_topology.yaml')
label = label_pose(topology, x, y, yaw_rad)
# physical_row_id, along_row_s_m, lateral_d_m, relative_heading_deg,
# zone_type, headland_id, label_confidence
```

Map every candidate's actual pose through the matching backend transform before labeling. A candidate patch index or provisional trajectory group is not a physical row label. Unknown query rows keep physical-row Recall/WrongRow/longitudinal metrics unavailable. Cross-backend labels are unavailable without a validated transform.

## Tests and current greenhouse smoke

```bash
cd /home/yangxuan/ros2_ws/src/agt_mapping_framework/tools/agt_greenhouse_annotation
PYTHONPATH=. pytest -q
```

Synthetic tests cover projection, signed d, cumulative s, heading, row/headland/UNKNOWN assignment, unconfirmed geometry, overlap ties, save/load, hashes, immutable freeze, validator failures, timestamp correspondence, backend transforms, manual scenes, and GUI edit/Undo/Redo. Synthetic physical IDs never annotate real data.

The real Point-LIO greenhouse map loads 8,477,666 points and displays 120,000 reduced points with 1,180 timed keyframes. The smoke draft contains **zero physical rows/headlands and 100% UNKNOWN** until a person annotates the actual scene. Loading/saving that draft and full source checksum validation prove tool/data compatibility; they do not claim physical-topology completion or retrieval performance.

For an unattended compatibility screenshot without any invented physical annotations:

```bash
ros2 run agt_greenhouse_annotation greenhouse_annotator \
  --map-package /path/to/map_package \
  --output /path/to/smoke/greenhouse_topology.yaml \
  --smoke-output /path/to/smoke/gui_map_load.png --smoke-save-empty
```

This mode refuses existing geometry. Interactive launch omits both smoke arguments.
