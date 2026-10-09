# MapStudio Operator Workflow v1

Status: bounded software implementation **PASS**; **operator acceptance remains NOT VERIFIED**. See the acceptance record for tested capabilities and manual gates. This workflow composes the existing frontend map-package validator, `agt_greenhouse_annotation` schema v1 and freeze command, keyframe-block validator, and single-query GLOBAL/Top-K/GICP evidence producer. It adds no localization algorithm and does not run robot or navigation runtime behavior.

## Assets and identity

| Asset | Owner / authoritative check | Revision identity |
|---|---|---|
| Source map package | `agt_mapping_artifacts.frontend_package.verify_frontend_map_package` | Existing manifest/checksum/pose digests are recorded independently |
| Structure annotation | `agt_greenhouse_annotation` schema-v1 validator and `greenhouse_annotation_freeze` | Exact YAML byte SHA-256; review confirmation is explicit and freezing writes a new directory |
| Keyframe block set | `agt_mapping_artifacts.keyframe_blocks.verify_keyframe_blocks` | Existing immutable block-set manifest and index digests |
| Query Set | `agt_map_localization_benchmark.study_workflow.validate_query_set` | `content_sha256` is canonical JSON SHA-256 over the complete asset mapping with the self-digest field omitted; frozen revision is create-once |
| Relocalization Study | `agt_map_localization_benchmark.study_workflow.validate_study` | Same content-digest rule; evidence refs additionally record exact evidence-manifest SHA-256 |
| MapStudio project | `agt_map_localization_benchmark.study_workflow.validate_project` | Mutable workspace manifest digest; it stores relative path hints and each dependency's identity/hash |

Query locations store both the requested map-frame point and a resolved source observation: keyframe ID, pose index, timestamp, full map position, snap distance, and reference level. Validation resolves the exact source and recomputes nearest legal keyframe/snap distance. Disabled records remain visible in drafts but are excluded from a frozen Study. Frozen Query Sets cannot be edited; clone to a new draft revision, then freeze into a new file.

Structure sampling uses only rows with `confidence=confirmed` from an annotation with `status=frozen` and `manual_review_confirmed=true`. The Query Set and block set must reference the exact same annotation bytes. No inferred physical row identity is promoted from geometry or localization evidence.

## Study behavior

A Study requires a current frozen Query Set. Supported query windows are a unique subset of 1, 3, and 5 frames; `candidate_top_k` is limited to 1–5. Every enabled query expands to one existing offline query per selected frame count. Jobs update a manifest between queries, preserve completed evidence, classify isolated failures as `NO_DATA`, and mark unstarted work `NOT_RUN` after cancellation. `cancel_requested` is persisted separately from terminal `CANCELED`.

Classification and aggregate denominators are reference-aware. Same-session results remain explicitly `SAME_SESSION_REFERENCE`; they are not independent ground truth. GICP convergence is recorded separately and never determines `CORRECT`. `correct@k` is `N/A` when the reference or eligible candidate rank is unavailable. Rates are empirical count ratios, not probabilities.

The Study Browser's map evidence layer resolves each result to the frozen Query Set's `resolved_position_m` and displays the five supported result classes as colored point markers. Classification, scene, and frame filters constrain both the result tree and marker layer. Marker colors represent observed classifications only, not probability or navigability.

The exporter writes `summary.csv`, `queries.csv`, `candidates.csv`, and `study_manifest.yaml`. Candidate CSV columns retain both the stable common fields and any additional fields supplied by the evidence producer, serializing structured values as JSON. Evidence stays in its existing immutable package format. Project manifests use relative filesystem path hints and content identities; open/validate rechecks dependencies and reports missing or changed references as stale/invalid rather than ready.

## CLI surface

Installed entry point: `agt_mapstudio_workflow`.

```text
query-set-manual
query-set-sample
query-set-validate
query-set-freeze
query-set-clone-draft
query-set-add-manual
query-set-set-enabled
query-set-delete-draft-query
study-create
run-study
study-validate
study-export
project-init
project-validate
project-add-assets
project-set-dependencies
```

The Qt workflow calls the same commands and validators. The GUI does not reimplement source, annotation, block-set, or evidence validation.

## UI language

The menu `Language` provides `简体中文` and `English`, and persists the selection under `QSettings` key `AGT/MapStudio: ui/language`. Translation applies to visible widget/action/menu/dock/table/tree strings and later-shown dialogs. Combo-box item data, enum values, IDs, paths, and serialized assets are unchanged. The in-repo catalog is curated; untranslated free-form diagnostics and arbitrary third-party dialog text can remain in their original language.
