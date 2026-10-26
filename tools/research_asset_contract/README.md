# Research Asset Contract V1

This package owns the machine-readable contract and migration adapter used by
MapStudio's cross-session annotation workflow. `Annotation JSON V1` is the
canonical editable source. The existing `greenhouse_roi.yaml` remains a
read-only import format and a generated adapter for `cross_stage_analysis`;
it is never the sole or canonical research record.

The package uses the JSON Schema in
`docs/contracts/research_asset_contract_v1.schema.json` plus semantic checks
for hashes, transform direction, session identity, and paper-statistics gates.
It has no ROS, GUI, SLAM, or registration dependency.

```bash
python3 -m pytest tools/research_asset_contract/test
python3 tools/research_asset_contract/migrate_run008.py \
  --run008 /home/yangxuan/ros2_ws/experiments/artifacts/output/cross_stage_greenhouse_20261007_run008 \
  --output /home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1
python3 tools/research_asset_contract/validate_bundle.py \
  /home/yangxuan/ros2_ws/experiments/research_assets/cross_stage_greenhouse_20261007_v1/project.json
```

The migration refuses to overwrite an existing asset directory. It references
raw bags, source map packages, and run008 in place, recording SHA-256 values;
it does not copy or edit them.
