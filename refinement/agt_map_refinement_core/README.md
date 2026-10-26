# agt_map_refinement_core

Offline Phase 2 refinement pipeline for an existing validated `map_package`.
It applies `refinement.yaml` rules without changing the original mapping package,
and publishes a new package with a Nav2-compatible PGM/YAML preview.

```bash
ros2 run agt_map_refinement_core apply_map_refinement \
  --map-package /path/to/map_package \
  --refinement /path/to/refinement.yaml \
  --output /path/to/refined_map_package
```

ASCII and common binary float32 PCD inputs are supported; output retains the
input encoding unless explicitly selected. `remove_polygon`, `remove_box`,
`remove_sphere`, and `remove_height_band` are evaluated in map coordinates on
every paired body-frame patch. The derived `map.pcd` is rebuilt from those
retained patch observations, preserving non-coordinate fields such as intensity.
This can change sampling density relative to a downsampled parent map. Empty
patches retain their original pose entry. The patch and both pose file sets must
match; source integrity and rule bytes are checked again before publication.

The existing `refined_mapping_source` manifest/checksum format binds the parent
and all derived files. Publication uses a staged directory and an atomic
no-replace rename; existing outputs and paths inside the source are rejected.
The derived package explicitly records its refinement identity; it does not
claim to be an unmodified frontend package or a READY Site/Route.

`forbidden_zone` affects the navigation preview without deleting observations.
Poses remain unchanged, so downstream trajectory footprint sweep may still mark
the driven area free. A deletion is not a keepout rule: use the relevant map
editing/review operation for that intent. Automatic recognition of moving people
is not implemented by this module; `filter_report.yaml` keeps dynamic filtering
marked deferred.
