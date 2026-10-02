# agt_mapping_bringup

Launch composition and lifecycle package for offline and live MID360 mapping.

## FAST-LIO2-only offline reference

For a benchmark reference that must not use PGO, pass `--reference fastlio` to
`scripts/run_mid360_mapping.sh`. This mode launches the sensor adapter,
FAST-LIO2, its frontend relay, and `fastlio_reference_exporter`; it does not
launch `pgo_node`, `agt_pgo_backend`, or the PGO artifact exporter. The
exporter pairs `/mapping/frontend/cloud` with `/mapping/frontend/odometry`,
then builds `map.pcd`, `patches/`, and both pose files from those same pairs.
It validates every patch-to-map transform and writes the package metadata as
`FASTLIO2_SAME_SESSION_REFERENCE`, with absolute ground truth explicitly
unavailable.

Example for a freshly built workspace overlay:

```bash
scripts/run_mid360_mapping.sh /path/to/green-house /path/to/new/mapping-run \
  --no-rviz --rate 1.0 --reference fastlio \
  --setup /home/yangxuan/ros2_ws/install/setup.bash
```

The output package is `fastlio_reference_package/` under the mapping run
directory. The default mapping workflow remains the PGO mode.
