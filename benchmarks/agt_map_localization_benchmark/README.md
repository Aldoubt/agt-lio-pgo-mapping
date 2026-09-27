# Phase 3B: Offline Localization A/B (experimental only)

This independent ament_python package **does not run a ROS node, subscribe to a
sensor, publish a map, publish TF, or modify navigation runtime**. It only
prepares *new* PCDs in a uniquely named experiment directory and invokes the
**unchanged** `agt_global_relocalization_native` executables already used by
manual-seed LOCAL and candidate-led GLOBAL localization. Do not interpret any
output as a production map, field gate, calibrated probability, or absolute GT.

## Source contract

- Source PGO is the single-session optimized `map_package` with 702 body-frame
  patches and `poses_timed.txt`. The `map.pcd` patch-concatenation order is
  checked against the patch poses before it is ever sliced by keyframe. Read
  only: the optimized PGO is **not** rerun or rewritten.
- V1 `confidence_voxels.pcd`, separate geometry_v1 sidecar, and the Phase 2B
  reviewed derivative have verified checksums, parent hashes, voxel keys and
  centroids. The V1 deferred `geometry_score=1` is required.
- Candidate A: raw PGO points from this experiment's **map subset**.
  B: those same raw map-subset points selected by V1 AUTO stable voxel keys.
  C: those same points selected by reviewed stable keys (if verified).
  D: B points whose **valid translation Qt** is at least the predeclared
  25th/50th/75th *unique stable voxel* quantile (not a fused confidence).
  B/C here are split-derived raw-point PCDs, **not byte-identical copies of**
  the immutable full-session `stable_map.pcd` exports. Their masks still use
  full-session evidence; that information leakage is explicitly disclosed.
- For each D, controls draw exactly the same number of points from B with a
  declared deterministic seed: unrestricted RANDOM and 1m-XY-cell
  round-robin VOXEL. Report number of points, 1m XY occupied cells relative
  to A, bounds and footprint. No success-based threshold selection.

## Leakage tiers and references

- Synthetic ground/wall/corner/pole/repeated walls: idealized exact geometry,
  known reference, strong/weak starts; may have exact map/query overlap. Its
  exact synthetic reference is **not** evidence of real-world accuracy.
- Tier0: fixed query center(s) 350 (SMOKE) and a map **containing its own
  keyframe**: `SELF_QUERY/DATA_LEAKAGE_EXPECTED` on every result.
- Tier1: centers 350 (SMOKE) or 175,350 (STANDARD); map contains only
  predeclared `i % 3 != 2`, within 30m of a chosen center, taking **at most
  the 96 nearest** for each center, **excluding all query windows ±2**.
  Candidate maps **and descriptor patches** use only that map subset; each
  query body patch is held out. This bounded evaluation ROI is centered using
  the **known optimized reference pose**: an additional *oracle ROI bias*, not
  a deployable no-seed map selector or full-map search. Reference poses come from
  the *same* PGO optimization, and confidence/geometry normals/labels were
  estimated over **all 702 keyframes**: pose-graph and evidence-label leakage
  remain. Only call the reference `optimized_PGO_pose`, not absolute GT.
- Tier2: no verified same-place independent session; `NOT_RUN`. FULL is not a
  selectable CLI profile and needs explicit user authorization.

For 3/5-frame queries, every neighboring body-frame patch is transformed into
reference body coordinates with `inv(T_map_body_reference) * T_map_body_i`.
LOCAL initial errors add translations in map XYZ; yaw is pre-multiplied in map
Z. Same query PCD, initial pose, CLI parameters and native executable are used
for every A/B/C/D/control comparison. GLOBAL has **no initial pose**; candidate
Polar Context, CPU BBS and GICP use native builders/runtime binaries and only
map-split assets. GLOBAL results are kept separate from LOCAL.

LOCAL explicitly uses full_se3, map/scan leaf 0.25m, max correspondence 1.5m,
map crop radius 12m, half-height 5m, four threads, matching audited manual
seed defaults. The native crop is centered on the *perturbed initial pose*;
geometry diagnostics are anchored at the fixed optimized reference so the
per-query Q does not itself vary with the supplied initial pose. GLOBAL uses
native candidate parameters from the audited `global_relocalization.yaml`.
Every parameter and native executable SHA is in `manifest.json`.

`Qr_mapping_view_median` is a local median of *mapping-era*, per-voxel geometry
sidecar Qr. `Qr_query_conditioned` instead recomputes a local rotation matrix
from **only the candidate map points**, its normal per occupied voxel, and
`g=(p_map-query_reference_body_origin) cross n`. Each voxel contributes its
mean `g*g^T`, and Ht gives one vote per distinct valid normal voxel. They have
different populations/statistics and must **never** be silently equated; the
normal itself remains full-session-derived. Ht and Hr units stay separate.
Insufficient normals produce invalid Q (`null`) rather than artificial zero.
Weak directions have arbitrary sign; alignment uses absolute cosine. Q is a
directional diagnostic, **not P(success)**.

## Run without overwriting existing experiments

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
# package installed to a separate Phase 3B overlay, NOT to ros2_ws/install
source /home/yangxuan/ros2_ws/experiments/agt_map_localization_phase3b_20260927/install/setup.bash
ros2 run agt_map_localization_benchmark agt_map_localization_benchmark \
  --profile smoke --run-id smoke_20260927_01
ros2 run agt_map_localization_benchmark agt_map_localization_benchmark \
  --profile standard --run-id standard_20260927_01 \
  --smoke-run /home/yangxuan/ros2_ws/experiments/agt_map_localization_phase3b_20260927/runs/smoke_20260927_01
```

SMOKE always runs the synthetic suite **first**, then bounded Tier0/Tier1
native tests. STANDARD requires a completed SMOKE on identical source bytes;
it uses two Tier1 centers, 1/3/5-frame queries and all 20 predeclared starts.
GLOBAL (if available) is run only **after all LOCAL registrations**; its
no-initial-pose workload is deliberately bounded. The CLI refuses to reuse a
run-id. If interrupted, it leaves `INCOMPLETE` and `results.jsonl`, never
claims partial statistics were a completed run. Each completed output has
`manifest.json`, `real/<tier>/{split,queries,candidates}.json`, new candidate
PCDs, per-query PCDs, `results.jsonl`, `results.csv`, `summary.json`,
`report.md`, and offline plots. All source checksums are validated before and
after. No Phase 1–3A source is written.

Strict/nominal/loose analytical criteria (3D translation m, absolute z m, yaw
°): **(0.25, 0.20, 5), (0.50, 0.40, 10), (1.00, 0.70, 20)**.
They are **not safety thresholds**. Native `fitness`/`overlap` and poses are
reported when actually returned. The unchanged CLI does not export exact
inlier counts or iteration counts: columns stay `null`. For a successful
GLOBAL result outside loose tolerance, the report counts an incorrect backend
success; without an independent negative session, a negative-session false
relocation *rate* remains `NOT_RUN`. Spearman and empirical Q bins group
repeated starts before comparison and do not calibrate Q to a probability.

## Tests

```bash
PYTHONPATH=benchmarks/agt_map_localization_benchmark \
  python3 -m pytest -q benchmarks/agt_map_localization_benchmark/test
```

Existing core/Studio regression is run via its separate, already isolated
build/install directories; the benchmark does not modify those packages.
