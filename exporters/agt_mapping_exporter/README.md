# agt_mapping_exporter

Subscribes to backend keyframes, map pose and status. On
`artifact_export_requested`, it writes `output_dir/map_package/` with PCD,
trajectory, patch, metadata, calibration, manifest and checksum files.

The current backend does not provide point-cloud patches or an optimized PGO
correction. The exporter therefore writes valid zero-point PCD placeholders and
records `backend_status.optimized: false`; these artifacts are structural test
evidence, not dense maps for navigation or localization.
