# Map Refinement Layer Design Review

Status: Phase 2 approved and implemented as an offline-first refinement
package. Existing mapping code and the original package format remain
unchanged.

## Scope and repository baseline

The audit was performed in the current workspace at
`/home/yangxuan/ros2_ws/src/agt_mapping_framework`. The source repository
documentation identifies this project as the ROS 2 Humble mapping framework
for MID360 -> FAST-LIO2 -> PGO -> validated map artifacts. The rosbag used by
the baseline acceptance is external to the repository; the named fixture is
`bunker_mid360_mapping_20260901_205036`.

The existing published map package contains the following files:

```text
map_package/
├── map.pcd
├── poses.txt
├── poses_timed.txt
├── patches/
├── calibration.yaml
├── metadata.yaml
├── manifest.yaml
└── checksums.sha256
```

The requested refinement output is therefore a derived package. The source
package must remain immutable and retain its original validation evidence.

## 1. Current data flow

The current runtime and export path is:

```text
MID360 adapter
  -> FAST-LIO2 frontend
  -> frontend odometry/cloud/path topics
  -> PGO backend
  -> backend map_pose/keyframes/status topics
  -> mapping artifact exporter
  -> agt_mapping_artifacts.ArtifactWriter
  -> map_package/
```

The architecture document also describes an optional dense reconstruction
stage between PGO and map artifacts. In the implemented exporter path,
`MappingArtifactExporter` listens for a backend status containing
`"optimized":true`, extracts `artifact_source`, and calls
`ArtifactWriter.write_optimized_pgo()`.

`ArtifactWriter.write_optimized_pgo()` copies `map.pcd`, `poses.txt`,
`poses_timed.txt`, and `patches/`, then writes `calibration.yaml`,
`metadata.yaml`, `manifest.yaml`, and `checksums.sha256`. The separate
`MapPackageExporter` is the validated release/export path: it requires an
optimized PGO source, copies the source files atomically into
`<map_root>/<site>/<version>`, and creates the package metadata and integrity
files there.

The current scripts expose two separate operations:

* `run_mid360_mapping.sh` runs the mapping workflow and leaves the final
  `output/map_package` under the run directory.
* `verify_map_artifact.sh` checks required files, checks that metadata declares
  optimized PGO, and runs `sha256sum -c checksums.sha256`.

No current operation mutates an existing map package in place.

## 2. PCD export entry points

There are two framework-owned export entry points:

1. The ROS exporter node in
   `exporters/agt_mapping_exporter/agt_mapping_exporter/exporter_node.py`
   reacts to the optimized backend status and delegates file writing to
   `ArtifactWriter`.
2. The installed `agt_mapping_artifacts export_map_package` command invokes
   `MapPackageExporter.export()`. The shell wrapper
   `scripts/export_validated_map_package.sh` uses this command for the
   versioned release path.

The actual dense PCD is expected to have already been produced by the PGO
artifact source. The framework does not currently fuse keyframe clouds in the
exporter itself.

## 3. PGM generation audit

No PCD-to-PGM or occupancy-grid implementation was found in `core/`,
`backends/`, `exporters/`, `interfaces/`, `scripts/`, or `docs/`. The existing
contract explicitly assigns PGM, OctoMap, costmaps, and other navigation-only
derivatives to the downstream navigation repository.

The refinement request changes that boundary only for the new refinement
output. The least disruptive design is to add a refinement-owned, Nav2-
compatible PCD-to-occupancy-grid exporter and keep the original mapping
package unchanged. The generated `nav_map.pgm` and `nav_map.yaml` should be
derived files with parent-package provenance, not replacements for
`map.pcd`.

## 4. Manifest and checksum logic

The existing integrity conventions are:

* SHA-256 values are written as `digest  relative/path` lines in
  `checksums.sha256`.
* `MapPackageExporter._write_integrity_files()` hashes all files except
  `manifest.yaml` and `checksums.sha256`, writes the checksum file, then writes
  a YAML manifest containing the same digest map, required files, and package
  kind.
* `verify_map_artifact.sh` validates the required files and executes
  `sha256sum -c checksums.sha256`; it does not independently compare the
  digest map in `manifest.yaml` with the checksum file.

The refinement pipeline should reuse this convention, write all output files
into a staging directory, generate the manifest and checksum file only after
all content is complete, and publish with an atomic rename. It must include
`map.pcd`, `nav_map.pgm`, `nav_map.yaml`, `refinement.yaml`, metadata, and any
filter report/evidence files that are part of the package contract. The source
package hash and manifest hash should be recorded as lineage fields.

There is a small implementation inconsistency to resolve before reuse:
`ArtifactWriter.write()` writes a manifest and then calls `write_checksums()` a
second time, which causes the final checksum file to include `manifest.yaml`
while the digest map embedded in the earlier manifest does not include that
final manifest digest. The new refinement writer should use one deterministic
policy and add a regression test for both checksum-file verification and
manifest/checksum agreement. This can be fixed locally in the new package
without changing the existing package format.

## 5. Current interface design

The backend interface package `agt_mapping_backend_api` currently defines:

* `Keyframe.msg`: timestamp, pose, and a string `cloud_reference`.
* `KeyframeArray.msg`: a header and an array of `Keyframe` values.

The exporter also consumes `PoseStamped` map pose and `std_msgs/String`
backend status. The public file contract defines optimized poses as
`T_map_body`; keyframe patch clouds are declared to be in the body frame.

The current API does not provide a per-frame point-cloud payload or a frame
index that can be joined to raw clouds by the refinement process. Therefore a
first version of dynamic filtering cannot be safely implemented as a live
backend modification. It must consume an offline evidence set that contains
`map.pcd`, `poses_timed.txt`, keyframe cloud references/payloads, or an
explicitly documented precomputed voxel observation stream.

## 6. Proposed insertion point

The new flow should be downstream of validation and upstream of navigation
deployment:

```text
validated source map_package (read-only)
  -> refinement.yaml rule parsing
  -> optional voxel persistence evidence/filter
  -> static point selection and rule-layer edits
  -> refined map.pcd
  -> Nav2 occupancy conversion
  -> refined_map_package/ manifest + checksums
```

Recommended ownership:

* `refinement/agt_map_refinement_core`: package orchestration, source package
  validation, rule application, lineage, atomic output, and integrity files.
* `refinement/agt_dynamic_filter`: offline voxel persistence analysis and
  `static_map.pcd`, `dynamic_candidate.pcd`, evidence, and report outputs.
* `refinement/agt_map_editor`: schema and rule operations such as
  `remove_polygon`, `remove_box`, and `forbidden_zone`.
* `interfaces/agt_map_refinement_interfaces`: the
  `GenerateRefinedMap.srv` ROS service contract.
* `agt_map_exporter` (under `refinement/` or a clearly separate package):
  Nav2-compatible PGM/YAML generation from the refined PCD.

These packages should be offline-first and plugin-oriented. The service and
CLI should call the same core pipeline rather than duplicate transformation
logic.

## 7. Compatibility and non-compatibility risks

### High risk

* **Dynamic-filter evidence gap.** Existing backend messages carry poses and
  cloud references, not the keyframe point data required to calculate
  observation count, frame count, persistence ratio, or height variance.
  The implementation must define an input evidence format and fail clearly
  when it is absent.
* **PCD semantics.** The filter and editor must preserve the source PCD
  fields, coordinate frame, and point precision where possible. A PCD with
  missing or unsupported fields must not silently become an invalid map.
* **Nav2 conversion semantics.** PGM generation needs explicit resolution,
  origin, occupied/free/unknown thresholds, z filtering, and frame metadata.
  A 3D map cannot be converted correctly without those parameters.

### Medium risk

* **Integrity ordering.** Manifest and checksum generation must have one
  deterministic policy, and the refined package must not overwrite the source
  package.
* **Forbidden-zone meaning.** A forbidden zone is a navigation derivative
  concept. It should be represented in `refinement.yaml` and reflected in the
  generated occupancy grid, while the filtered 3D PCD behavior must be
  explicitly defined rather than inferred.
* **Empty refinement behavior.** An empty rule file should preserve point
  data byte-for-byte when no dynamic filter is requested, while still
  producing a new package, lineage metadata, and fresh integrity files.
* **Service lifecycle.** A ROS service should report validation and output
  errors in its response and must not block the existing mapping runtime.

### Low risk

* Adding independent ament packages and a shell CLI does not require changes
  to FAST-LIO2, PGO, or the existing map package layout.
* Keeping the source package read-only and publishing to a new output path
  preserves rollback and reproducibility.

## 8. Proposed implementation plan after confirmation

1. Freeze the refinement package contract: input requirements, rule schema,
   PCD field policy, dynamic-filter evidence format, occupancy-grid parameters,
   and manifest lineage fields.
2. Add independent ROS 2 packages and the `GenerateRefinedMap.srv` interface;
   verify a selective `colcon build` before touching transformation logic.
3. Implement shared source validation, deterministic staging, output
   publication, and manifest/checksum generation in the refinement core.
4. Implement rule operations and focused tests for empty input, polygon
   deletion, box deletion, and forbidden-zone export.
5. Implement the offline voxel persistence filter with explicit thresholds,
   evidence YAML, and static/dynamic PCD outputs. Do not change frontend or
   PGO input streams.
6. Implement PCD-to-PGM/YAML conversion with Nav2-compatible metadata and
   tests for resolution, origin, and occupied/free/unknown behavior.
7. Add the CLI and ROS service as thin callers of the same pipeline.
8. Run the requested acceptance checks against the supplied
   `bunker_mid360_mapping_20260901_205036` artifact when it is available;
   otherwise run synthetic PCD fixtures and report the missing external
   fixture explicitly.

## 9. Phase 2 decision record

The following decisions were confirmed before Phase 2 implementation:

1. The current repository checkout remains the target workspace and repository
  identity is unchanged.
2. Refined packages retain calibration, poses, patches, and the other source
  mapping assets for compatibility.
3. Dynamic filtering is offline-first; Phase 2 defines its evidence interface
  only and does not modify FAST-LIO2, PGO, or their ROS interfaces.
4. `nav_map.pgm` and `nav_map.yaml` are refinement-owned derivatives. The
  existing mapping exporter and original `map_package` format are unchanged.

Phase 2 implementation is located under `refinement/agt_map_refinement_core`,
with examples in `map_refinement_examples/` and the shell entry point in
`scripts/apply_map_refinement.sh`. Focused tests pass and selective `colcon
build --packages-select agt_map_refinement_core` passes. The real external
baseline artifact is not present in this checkout, so validation against
`bunker_mid360_mapping_20260901_205036` remains pending artifact availability.
