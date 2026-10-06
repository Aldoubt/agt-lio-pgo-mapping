# agt_mapping_bringup

The operator entry point defaults to the user-selected `fast_livo2_lio` profile: FAST-LIVO2 LIO-only with LiDAR+IMU, `img_en=0`, and no PGO/loop-closure chain. FAST-LIO2 is disabled in the backend selection policy.

## Offline bag workflow

```bash
scripts/run_mid360_mapping.sh /path/to/green-house /path/to/new/mapping-run \
  --no-rviz --rate 1.0
```

The CLI checks the rosbag, resolves the default profile, and launches `mapping_v0.launch.py`. It sources the FAST-LIVO2 workspace recorded by the profile before the mapping overlay, waits for the native frontend, adapter, and exporter consumers, then replays only the selected LiDAR and IMU topics. A successful replay requests a paired body-cloud/odometry export and verifies the resulting `map_package/` using the existing `verify_frontend_map_package` validator. Cancellation or replay failure never triggers export. The rosbag is read-only; output must be new or empty.

FAST-LIVO2 logs are retained under the ROS log directory. The recorded trajectory is a same-session frontend reference, not independent ground truth. The package keeps paired keyframe patches and poses so downstream block building does not require PGO.

The current verified session CLI supports `fast_livo2_lio`. Other registered frontend profiles remain inspectable/selectable in the lower-level backend launch, but do not silently fall back through the session CLI. `--backend fast_lio2_legacy` and legacy `--reference pgo|fastlio` options are rejected.

## Live MID360 capture

The live capture entry points use the same FAST-LIVO2 LIO-only exporter and record the raw LiDAR/IMU stream under `<OUTPUT>/raw_bag`. Live hardware capture is not part of the `green-house` offline replay acceptance.

## Verify a source package

The session automatically verifies before marking `session.json` as `completed`. To verify later:

```bash
source /opt/ros/humble/setup.bash
: "${FAST_LIVO2_WS:?Set FAST_LIVO2_WS to the FAST-LIVO2 workspace root}"
: "${ROS_WS:?Set ROS_WS to the ROS 2 mapping workspace root}"
source "$FAST_LIVO2_WS/install/setup.bash"
source "$ROS_WS/install/setup.bash"
source "$ROS_WS/install_mapping_framework/setup.bash"
python3 -c 'from agt_mapping_artifacts.frontend_package import verify_frontend_map_package; print(verify_frontend_map_package("/path/to/run/map_package"))'
```

The validator checks package checksums, pose/patch correspondence, map reconstruction, and point counts. It does not validate map quality, localization correctness, or navigation safety.
