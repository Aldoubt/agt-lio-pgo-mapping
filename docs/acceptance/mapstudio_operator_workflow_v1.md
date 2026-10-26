# MapStudio Operator Workflow v1 acceptance

Status: **SOFTWARE PASS; OPERATOR ACCEPTANCE NOT VERIFIED; OVERALL PARTIAL**. Build, automated checks, project validation, and one real green-house regression Study are recorded below. GUI actions that require visual or human confirmation remain unchecked.

## Software and data evidence

- Worktree branch: `feature/mapstudio-operator-workflow-v1`; implementation base: `b6c1538a05cd07d261022f478d4b61d738b3f9e4`.
- Isolated build: `agt_map_localization_benchmark` and `agt_map_studio` built successfully into `/tmp/mapstudio-operator-install`; the annotation and mapping-artifact packages were also built into that temporary overlay so this checkout's runtime dependencies are available.
- Automated tests: Study/Query Set Python tests **11 passed**; greenhouse annotation/workbench Python tests **101 passed**; MapStudio CTest **17/17 targets passed (72 test cases)**, including language switching, point-cloud render sampling/Z filtering, and Study-result markers; all three relocalization JSON schemas pass Draft 2020-12 meta-schema checks; `git diff --check` passes.
- Project validation returned `valid=true`, `state=CURRENT`, no errors or stale reasons, with one Query Set and one Study.
- Real Study `kf590_regression_1f_topk4` completed 1/1 query with status `SUCCEEDED`; its exact evidence package validates `PASS` and is current. Query KF590 selected rank-1 KF305 and was classified `FALSE_ACCEPT`; measured same-session reference errors are XY 3.764943 m and yaw 1.972562 degrees. GICP converged with fitness 0.171517. This is same-session regression evidence, not independent ground truth; physical row identity remains `UNKNOWN`.
- UI language: a `Language` menu offers 简体中文 and English, remembers the preference, translates curated visible strings in both directions, and preserves combo-box data/enums. Automated widget switching and data-preservation tests pass. Persistence across an actual close/reopen and full visual translation coverage are **NOT VERIFIED** without a display session.
- Study result markers are loaded from each result's Query Set `resolved_position_m` and rendered in the point-cloud viewer by classification: `CORRECT` green, `FALSE_ACCEPT` red, `REJECTED` yellow, `TIMEOUT` purple, and `NO_DATA` gray. Classification, scene, and frame filters update the marker layer. These are empirical labels, not probabilities.
- KF590 report export passed at `/home/yangxuan/ros2_ws/projects/mapstudio/green-house/operator_workflow_v1/reports/kf590_regression_1f_topk4_20261006/`, containing `summary.csv`, `queries.csv`, `candidates.csv`, and `study_manifest.yaml` (plus copied summary YAML). The candidate export preserves extra evidence-producer fields such as BBS/patch diagnostics.
- A headless application launch kept the event loop alive until timeout, but Qt could not create an OpenGL context. This does not count as visual GUI acceptance.
- The point-cloud viewer now applies the Z band to the displayed sample as well as rectangle/polygon selection. Rendering defaults to a deterministic 500,000-point cap (250k/500k/1M/1.5M/3M/Full are selectable); editing and exports retain the full source cloud. Sampler tests verify deterministic caps, Z filtering before the cap, and the Full option. A physical-display FPS comparison is **NOT VERIFIED**.
- Chinese localization now includes the relocalization panel's offline-analysis explanation and the publishing workflow descriptions shown in the supplied screenshots. Widget language-switch tests pass; full visual coverage across every dialog remains **NOT VERIFIED** as stated below.
- First visible use of the structure editor starts asynchronous row/aisle proposal generation in a map-manifest-bound authoring workspace and restores any existing manual reference draft. A visibility-gating regression test passes. Running the same COMPARE backend against the full green-house source (7,799,820 points) completed in 252.45 s with 1.1 GiB peak RSS, yielding 24 GLOBAL_PROFILE rows, 53 LOCAL_TRACKS, and 23 aisle proposals. This is a smoke/proposal result only: physical topology remained empty pending operator review, and the artifacts were written under `/tmp/agt_mapstudio_structure_analysis`, not published map assets. The preview renderer's 500k cap does not reduce offline analysis input size.
- The source map package, block set, annotation candidate, old experiment evidence, and fixed checkout were not modified.

## Green-house project preparation

The currently validated FAST-LIVO2 LIO-only source is:

```text
/home/yangxuan/ros2_ws/experiments/fastlivo_lio_green-house_20261006_retry01/map_package
```

The existing 112-block set is:

```text
/home/yangxuan/ros2_ws/experiments/mapstudio_relocalization_mvp_20261006_01/blocks_k11
```

Both validate against the same source. The source reports 1,180 keyframes and 7,799,820 map points. The current-source annotation candidate is:

```text
/home/yangxuan/ros2_ws/experiments/mapstudio_relocalization_mvp_20261006_01/annotation_current_source/greenhouse_topology.yaml
```

It is schema-valid but contains zero rows/headlands and has `status=draft`, `manual_review_confirmed=false`; coverage is 100% `UNKNOWN`. It is not evidence of physical structure review. Do not freeze or sample row queries until an operator reviews actual row identities in the annotator and saves a new revision.

An initial MapStudio project is prepared outside the experiments tree:

```text
/home/yangxuan/ros2_ws/projects/mapstudio/green-house/operator_workflow_v1/project.yaml
```

It points to the source, unbound 112-block set, and current draft annotation. A manual-only KF590 regression Query Set and a 1-frame, Top-K=4 Study are in the project with `row_id=UNKNOWN`. The Study has run against the existing native localization tools; its evidence is stored within the project and the report is exported under `reports/kf590_regression_1f_topk4_20261006/`. Once an operator freezes the structure, build a new block set bound to that exact annotation before authoring reviewed-row Query Sets.

The earlier KF590 record is available as read-only evidence under:

```text
/home/yangxuan/ros2_ws/experiments/mapstudio_relocalization_mvp_20261006_01/evidence/
```

The new Study record has query KF590, rank-1 KF305, same-session XY error 3.764943 m, yaw error 1.972562 degrees, and `FALSE_ACCEPT`; the application reads these from the evidence record, not UI constants. Physical row identity remains `UNKNOWN`. The Study Browser has not yet been manually shown loading this newly produced evidence.

## Manual GUI checklist

Use a project manifest prepared for the worktree's Operator Workflow build. When opening a project, confirm that source, annotation and block-set state are checked from their recorded identities. The source and block assets above are read-only. Keep any newly edited annotation, blocks, Query Sets, Studies, and report outputs in a separate project/work directory.

- [ ] Launch the feature worktree's `map_viewer` build.
- [ ] Switch `Language` to `简体中文`; confirm menu, toolbar, Project Browser, labels, dialogs, and status fields switch where catalog translations exist.
- [ ] Switch back to `English`; confirm those visible strings return to English.
- [ ] Confirm language choice persists after closing and relaunching.
- [ ] Confirm enum/data values such as `FALSE_ACCEPT`, `UNKNOWN`, `AUTO`, and `GLOBAL_PROFILE`, IDs, file paths, and saved YAML remain unchanged by language switching.
- [ ] Open the green-house project and display the FAST-LIVO2 source cloud and trajectory.
- [ ] Display the current annotation and verify it is clearly marked `DRAFT / NOT REVIEWED`, with physical identities still `UNKNOWN`.
- [ ] Open the existing structure annotator from `Edit in new draft`; visually identify and edit the actual row centerlines, entry/exit, direction, and headlands. Save as a new draft.
- [ ] Validate the edited annotation against this exact source; manually compare the row IDs/geometries to the scene.
- [ ] Confirm Review & Freeze only after the human review. Verify a new frozen annotation revision was created and the prior file is unchanged.
- [ ] Build a new block set bound to that exact frozen annotation, then Save / Rebind the project to those exact revisions.
- [ ] Create a reviewed-row Query Set; check row entry, 25%, middle, 75%, exit, scene, resolved keyframe, timestamp, and snap distance.
- [ ] Create a manual query by clicking the map; confirm the requested click and resolved keyframe are shown separately.
- [ ] Preview markers; disable and delete a query in a draft; confirm the frozen revision does not change.
- [ ] Freeze a Query Set and confirm it is immutable; clone to a draft before further edits.
- [ ] Create a Study using a subset of frames 1/3/5 and Top-K 1–5; confirm existing native GLOBAL/Top-K/GICP only is selected.
- [ ] Run the Study while interacting with the viewer; inspect progress, failure count, cancellation, and persisted `NOT_RUN` results where applicable.
- [ ] Inspect scene/frame summaries, empirical rates, correct@1/3/5 and `N/A` reference handling.
- [ ] Filter to `False Accept`; inspect a result, candidates, GICP convergence/fitness, reference error, and classification independently.
- [ ] Select the Study and confirm the main map shows classification-colored result markers; change classification, scene, and frame filters and confirm the marker layer follows them.
- [ ] Switch candidates and overlay query/candidate clouds; confirm candidate loads are on demand.
- [ ] Export and inspect `summary.csv`, `queries.csv`, `candidates.csv`, and `study_manifest.yaml`.
- [ ] Save and close MapStudio.
- [ ] Reopen the same project and verify the same source identity, frozen annotation, block set, Query Set, Study, and evidence refs are restored.
- [ ] Change or remove a referenced dependency in a disposable copy; confirm the project/Study reports `STALE` or `INVALID` and does not show green/current.

Record operator, date, project digest, annotation revision, block-set revision, Study ID, and any failed step here after manual execution. Until then, every unchecked item remains **NOT VERIFIED**.
