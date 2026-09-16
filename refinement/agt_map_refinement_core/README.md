# agt_map_refinement_core

Offline Phase 2 refinement pipeline for an existing validated `map_package`.
It applies `refinement.yaml` rules, retains the original mapping assets, and
publishes a new package with a Nav2-compatible PGM/YAML derivative.

```bash
ros2 run agt_map_refinement_core apply_map_refinement \
  --map-package /path/to/map_package \
  --refinement /path/to/refinement.yaml \
  --output /path/to/refined_map_package
```

Phase 2 supports ASCII and common binary float32 PCD input, writing refined
maps as auditable ASCII PCD, plus `remove_polygon`, `remove_box`, and
`forbidden_zone`. Dynamic filtering is represented in `filter_report.yaml` as
deferred until its offline evidence interface is implemented.