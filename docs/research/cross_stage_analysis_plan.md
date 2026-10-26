# Cross-Stage Greenhouse Mapping Analysis Plan

Status: **PARTIAL — P0 audit and P1/P2 candidate tooling are in progress; P3 is gated on manual ROI review.** This plan records evidence from the current mapping-framework checkout and the two existing map packages. It does not treat the user-provided acquisition narrative as metadata-backed ground truth.

## Scope and input contract

The analysis consumes two immutable map packages and an already saved approximate cross-session transform. It does not replay rosbag, estimate a new alignment, alter the source PCDs, change navigation code, or modify SLAM. Every run checks both packages with the repository's `scripts/verify_map_artifact.sh`, verifies the PCD SHA-256 values against the saved comparison, and records package, pose, manifest, metadata, and bag-metadata identities.

| Role in this experiment | Input | Evidence-backed interpretation |
|---|---|---|
| Sparse candidate | `/home/yangxuan/ros2_ws/experiments/artifacts/output/white_tomato_20261006_collect_080252_mapstudio_20261007/map_package` | User-declared sparse-period capture; the source bag is the 2026-10-06 white-tomato collection. Platform is UNKNOWN in package metadata. |
| Reference candidate | `/home/yangxuan/ros2_ws/experiments/fastlivo_lio_green-house_20261006_retry01/map_package` | Existing `green-house` map package. Its physical stage and acquisition platform are UNVERIFIED from package metadata. |
| Existing transform and exploratory scores | `/home/yangxuan/ros2_ws/experiments/artifacts/output/white_tomato_vs_green_house_20261007/comparison.json` | Approximate transform from sparse-map coordinates into the reference-map coordinates; not an independent ground-truth pose. |

Both PCDs declare the local frame name `camera_init`. These names do not make the frames identical. The experiment names the reference coordinate frame `map_reference/camera_init` and uses only the saved transform to express sparse points there. No shared usable TF or independent surveyed datum was found. The frame relationship therefore remains approximate.

The current comparison JSON reports 0.5 m XY cells, z in `[-2, 6]` m, 8,225 reference cells, 6,919 sparse cells, 5,520 shared cells, and global IoU 0.574. This exploratory IoU includes external background and is not greenhouse-interior agreement. No reviewed centerline or cross-map row-correspondence asset exists for these two exact packages. Existing topology files found in the workspace are draft/unconfirmed or empty; the separate structure-migration worktree is dirty and is not an input or deliverable for this run.

## Reuse and implementation boundary

- Reuse the immutable Site 1.0 map-package contract and its existing `verify_map_artifact.sh` validator; do not introduce another production map-bundle schema.
- Reuse the saved `comparison.json` transform and its recorded registration diagnostics. This run validates the SE(3) matrix and inverse closure, then computes directional nearest-surface diagnostics on voxel-downsampled PCDs without rerunning registration.
- Reuse existing PCD/map editing and topology capabilities only as reviewed input workflows. `apps/agt_map_studio/src/occupancy/commands/` provides occupancy polygon/erase operations; `tools/agt_greenhouse_annotation/agt_greenhouse_annotation/topology.py` and `gui.py` support manual row/headland/topology annotations. They do not provide a confirmed cross-stage semantic greenhouse ROI or automatic centerline correspondence for these two maps.
- Reuse `exporters/agt_pcd2grid_exporter/src/PCDProjector.cpp` and `TraversabilityGridBuilder.cpp` for navigation-map projection and evidence-qualified traversability. These are not the cross-stage ROI/IoU metric engine.
- The checked-in current checkout search found no cross-stage centerline extractor or stable row-correspondence module. Existing annotation assets for this map pair are draft/unconfirmed or empty; the cross-stage line and width stages remain planned.
- Add a small offline analysis package under `tools/cross_stage_analysis/`. Its ROI format is explicitly an analysis candidate format, not a published navigation asset or replacement for Site/Route contracts.
- Keep raw PCDs, packages, and earlier comparison outputs read-only. Write this experiment under `/home/yangxuan/ros2_ws/experiments/artifacts/output/cross_stage_greenhouse_20261007/`.

## Staged work and gates

| Stage | Work and output | Acceptance / stop condition |
|---|---|---|
| P0 — audit and data contract | This plan, experiment YAML, input manifest, package hashes, frame/transform convention, and explicit UNKNOWN fields. | Both package validators pass; saved transform hashes match the package PCDs; no acquisition platform or unverified physical stage is invented. |
| P1 — alignment review | `alignment_transform.yaml`, `alignment_validation.json`, `alignment_validation.md`; proper-rotation/homogeneous-matrix checks, inverse closure, z quantiles, and regional bidirectional nearest-surface residual diagnostics. | Label result `Approximate Cross-Session Alignment`. Residual tiles remain generic spatial diagnostics until fixed structures are manually identified. |
| P2 — ROI proposal | Editable `greenhouse_roi.yaml`, `roi_visualization.png`, and separate `roi_manual_review.png`. The existing purple outline is retained as a PROVISIONAL greenhouse-boundary candidate. `navigation_interior_shrink_m` creates a separately labeled Navigation Interior ROI. Harvested, transition, dense vegetation, exterior and stable-structure selections remain unconfirmed. | The user explicitly authorized provisional P3 in this run. Every unreviewed polygon, stage label, and resulting metric is marked PROVISIONAL and not paper truth. |
| P3 — provisional occupancy comparison | Common-origin/common-extent occupancy layers at 0.5 m and 0.2 m; global, boundary-candidate and Navigation Interior IoU; `metrics.json/.csv`; a 3D overlay/residual diagnostic; Fig01–Fig03 as 4:3, 300 DPI PNG/PDF. | Reuse the exact saved SE(3) transform and verify map hashes. Empty cells remain UNKNOWN; one-sided occupied cells are censored observations, not environmental change. Stable-structure-only residual remains unverified without manual masks for corners, frame edges and columns. |
| P4 — row correspondence (planned) | Review/extract sparse and reference row candidates, stable row IDs, explicit correspondence and Fig04–Fig05. | Require physical-row confirmation; do not pair only by nearest distance. |
| P5 — shift and width (planned) | Three distinct centerline comparisons, row width/clearance profiles, metrics and Fig06–Fig07. | Require reliable along-row correspondences. Robot clearance claims require confirmed footprint and safety margins. |
| P6 — traversability (planned) | Evidence-qualified geometry/robot traversability layers and Fig08. | Unknown ground/obstacle evidence stays unknown; vegetation points are not automatically rigid obstacles. |

The current task runs P0–P3 with explicitly provisional ROIs. The `allow_provisional_roi_metrics` config switch records that authorization. Centerline shifts and width/traversability metrics remain deferred until row-level registration accuracy and physical correspondences are validated.

## Output and reproducibility contract

The run output includes `input_manifest.json`, `alignment_transform.yaml`, `alignment_validation.json/.md`, `greenhouse_roi.yaml`, `roi_visualization.png`, `roi_manual_review.png`, `roi_validation.json`, `metrics.json/.csv`, `stable_structure_residuals.json`, `p3_validation.json`, `paper_figure_manifest.json`, and `figures/Fig01..Fig03` in PNG/PDF. The experiment configuration is stored as `experiment.yaml`; command output and errors are preserved in `execution_log.txt`.

For this provisional P3 run, the command uses a fresh `run008` output directory. Earlier `run001`–`run007` outputs are retained and not overwritten. Run from the repository root:

```bash
OUT=/home/yangxuan/ros2_ws/experiments/artifacts/output/cross_stage_greenhouse_20261007_run008
mkdir -p "$OUT"
cp tools/cross_stage_analysis/config/greenhouse_cross_stage_20261007.yaml "$OUT/experiment.yaml"
PYTHONPATH=tools/cross_stage_analysis python3 -m cross_stage_analysis \
  --config "$OUT/experiment.yaml" --stage all \
  > "$OUT/execution_log.txt" 2>&1
```

Tests use synthetic arrays for transform, grid, ROI erosion, occupancy semantics and output gates. The real-data run verifies package validators and hashes while leaving source maps read-only. Expected status is PROVISIONAL until the ROI, stable structures and map-stage assignments receive manual review.

## Known limitations and deferred work

- The current transform is approximate; its registration fitness and global IoU are exploratory evidence, not accuracy ground truth.
- The two acquisitions' platform, exact temporal order, harvest boundary, fixed greenhouse border, and common vertical datum are not independently verified from package metadata.
- Nearest-surface residuals include moving crops and exterior trees. They cannot yet be called stable-structure residuals.
- PCDs contain point support, not sensor-ray evidence. An empty cell cannot be labeled free.
- No reviewed centerlines, row correspondence, confirmed robot footprint, or calibrated traversability evidence exists for this map pair.
- Fig01–Fig03 and global/ROI IoUs are generated provisionally; they are not final paper evidence until the provisional masks and acquisition stages are reviewed.
