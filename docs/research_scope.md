# Research Scope

## Purpose

This document defines the initial research boundary for collaboration. It is intentionally algorithm-focused: a research result must preserve reproducibility, frame semantics and artifact provenance before it is considered for navigation deployment.

## Current baseline

The reference baseline is known from the audited `agt_navigation_v3` environment. It is not yet implemented in this repository.

- **FAST-LIO2 / Batch-LIO frontend**: alternative LiDAR-Inertial Odometry frontends for MID360 LiDAR and IMU input.
- **PGO backend**: keyframe selection, loop closure and pose-graph optimization over synchronized front-end clouds and odometry.
- **HBA offline refinement**: optional refinement that consumes exported keyframe patches and poses after PGO.
- **PCD artifact generation**: export of dense optimized `map.pcd`, keyframe patches, trajectories and metadata.

Baseline experiments should use frozen rosbag input, calibration, package commits and parameter files. A result must identify the exact parent artifact and experiment configuration.

## Potential research

### Dynamic object removal

Develop point-level, object-level or temporal-persistence methods that reduce moving people, vehicles and vegetation artifacts. The method must report removed-point statistics and preserve the unfiltered evidence path for comparison.

### Semantic filtering

Associate semantic labels with points or patches, then use an explicit policy to retain, suppress or separately layer classes. Semantic model version, class mapping and confidence thresholds belong in `metadata.yaml`.

### Traversability map

Derive ground, elevation, slope, roughness and obstacle layers from the optimized map. The output may support navigation, but its robot geometry and cost policy remain downstream, navigation-specific configuration.

### Incremental map update

Support revisiting a site, localizing a new session against an approved artifact, detecting spatial changes and publishing a versioned delta or successor map. Original artifacts remain immutable; updates must retain lineage and rollback information.

## Evaluation expectations

- Compare against the frozen baseline on the same rosbag and calibration.
- Measure trajectory consistency, map quality, loop closure precision/recall where labels exist, runtime and memory.
- Record failure cases rather than filtering them from aggregate results.
- Export a complete artifact set for every reported result.
- Keep navigation field acceptance separate from mapping research validation.
