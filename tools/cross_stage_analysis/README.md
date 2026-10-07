# Cross-stage greenhouse analysis (P0–P3)

This tool audits and freezes an existing cross-session transform, prepares provisional greenhouse and Navigation Interior ROIs, computes observed-occupancy IoU at 0.5 m and 0.2 m, and generates Fig01–Fig03. It never estimates a transform or replays a rosbag. Unreviewed ROIs and stage/platform labels remain `PROVISIONAL`; all resulting figures and metrics are unsuitable as final paper truth until manual review.

Run from this directory with the experiment-specific config:

```bash
python3 -m cross_stage_analysis --config /path/to/experiment.yaml --stage all
```

Set `PYTHONPATH` to this directory when invoking it from elsewhere. The existing `verify_map_artifact.sh` remains the map-package validator. The transform file's contract is column-vector form: `target_point = T_target_from_source * source_point`. `--stage all` performs P0–P3 when `allow_provisional_roi_metrics: true`; `--stage occupancy` resumes P3 from an existing ROI package. The inward buffer is controlled by `navigation_interior_shrink_m`.

The standalone `--stage roi` also verifies both map packages and checks their PCD hashes against the saved transform provenance before it reads the clouds. Each experiment should use a fresh output directory; generated runs are not silently overwritten.

Observed occupied cells are represented only from point support. Empty cells remain UNKNOWN. No ray evidence means no FREE classification. A cell occupied in only one map is “one-sided observed occupancy,” not an environmental change. The alignment figure includes a 3D overlay and Navigation Interior surface residuals; these remain all-surface proxies because physical corners, frame edges and columns have no reviewed point masks.

Tests:

```bash
PYTHONPATH=. pytest -q test
```
