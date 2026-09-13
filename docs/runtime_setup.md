# MID360 PGO Runtime Setup

Use [`scripts/bootstrap.sh`](../scripts/bootstrap.sh) from the repository root. It is the supported installation path: it imports the exact revisions in `.repos`, installs ROS dependencies and builds only the packages needed by the baseline. Do not import the same `.repos` into another directory in the workspace, because duplicate ROS package names cannot be built by colcon.

Run the MID360 bag regression:

```bash
./scripts/run_mid360_mapping.sh /path/to/mid360_mapping_bag /data/agt_mapping_runs/mid360_run
```

RViz opens with the FAST-LIO2 world-frame cloud, frontend trajectory and odometry. It is a live LIO visualization, not a claim that the PGO map is already optimized. Keep the launch running until the export request has completed.

`auto_export:=true` is the default: three seconds after rosbag playback exits, the launch requests the PGO export service. To inspect the backend manually instead, pass `auto_export:=false` and use the service command below.

When replay has ended and PGO has drained, request real PGO export:

```bash
ros2 service call /mapping/backend/export_artifact std_srvs/srv/Trigger '{}'
```

Inspect the final artifact directory:

```text
/data/agt_mapping_runs/mid360_20260901_205036/map_package/
```

Use it only if `metadata.yaml` has `backend: PGO` and `backend_status.optimized: true`.
