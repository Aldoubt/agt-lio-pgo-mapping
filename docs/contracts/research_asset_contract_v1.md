# Research Asset Contract V1

`agt.research_dataset_manifest/v1` groups acquisition sessions and names one declared reference frame. Each entry binds a stable `session_id` to exactly one session manifest. A session manifest records `acquisition_platform`, `growth_stage`, `reference_frame`, backend, bag metadata, and provenance status. Unknown values stay `UNKNOWN`; `USER_DECLARED` and `FILE_VERIFIED` identify where a non-unknown value came from. No stage or platform is inferred from a filename.

`agt.research_alignment_contract/v1` binds one `source_session_id` to one distinct `target_session_id`. The only supported direction is `T_target_from_source`: column points are transformed by `p_target = T_target_from_source * p_source`. Its `source_hash` and `target_hash` must equal the exact PCD hashes in those two session manifests. The homogeneous transform must be finite and in SE(3). A visually plausible alignment remains `APPROXIMATE`/`DRAFT` until a person reviews it.

`agt.research_annotations/v1` is the single canonical editable ROI/structure format. It binds `dataset_id`, `annotation_version`, `reference_frame`, and the exact `source_hashes` map. Each feature has a unique `annotation_id`, a canonical `annotation_type`, geometry, review status, and one or more exact `source_session_ids`. V1 geometries are `polygon_xy`, `polyline_xy`, `point_xyz`, and a shape-specific `selection_3d` (`aabb`, `polygon_prism`, or `sphere`). Coordinates use metres in the document reference frame. The GUI imports the existing run008 YAML candidate into this JSON form; it does not create a second canonical ROI format. The old topology Schema v1 remains a separate row/headland/scene contract and is not repurposed for cross-stage ROIs.

`agt.research_analysis_configuration/v1` records the dataset, annotation version, alignment ID, analysis parameters, and input hashes. `agt.research_experiment_result/v1` records the exact same dataset/alignment, the annotation version used, and SHA-256 of the complete `analysis_configuration.json` byte stream. Its `result_hashes` map covers run008 output files by relative path; it does not hash the result record itself.

Digest coverage is deliberately non-self-referential:

- `source_hash`: raw bytes of one source `map.pcd`.
- `package_manifest_hash` and `bag_metadata_hash`: raw bytes of the source package manifest and bag `metadata.yaml` respectively.
- Alignment `provenance_hash`: raw bytes of the original run008 alignment YAML. The source/target PCD hashes are separate fields.
- Annotation `imported_from.sha256`: raw bytes of the imported run008 ROI YAML.
- Experiment `analysis_configuration_hash`: raw bytes of the complete canonical analysis configuration JSON.
- `result_hashes`: raw bytes of each listed run008 result file, excluding the result contract itself.
- Dataset, session, alignment, annotation, analysis, and result manifests do not contain their own digest. `project.json` links them by path, so no manifest hashes itself.

Review lifecycle is `DRAFT` → `REVIEWED` → `FROZEN`. Editing geometry or adding/removing a feature returns the affected feature and annotation document to `DRAFT`. Freezing requires every feature to be `REVIEWED` and a reviewer identity; frozen annotations are immutable in the GUI. A result is eligible for formal paper statistics only if the annotation and every feature are `FROZEN`, the alignment status and review status are `FROZEN`, the result uses the exact annotation version and is itself `FROZEN`, and every session's growth stage is known. The independent validator reports each failed gate.

The independent, GUI-free validator is `tools/research_asset_contract/validate_bundle.py`. It checks the JSON Schema, session IDs, exact PCD hashes, transform direction and SE(3), annotation source hashes, analysis/result identity, configuration digest, and the paper-statistics gate. MapStudio repeats the critical source/session/alignment/annotation checks before displaying maps. The schema and validator can be used without Qt or ROS.
