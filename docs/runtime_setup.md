# MID360 PGO Runtime Setup

External FAST-LIO2, PGO and their `interface` service package are pinned in `.repos`; import/build them beside this repository rather than copying their source. Do not run `vcs import` again when those repositories already exist in `src/`, because that creates duplicate ROS package names.

```bash
cd ~/ros2_ws/src/external
vcs import < ../agt_mapping_framework/.repos
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src/agt_mapping_framework src/external/fast_lio2_mapping \
  --packages-up-to agt_mapping_bringup
source install/setup.bash
```

Run the MID360 bag regression:

```bash
ros2 launch agt_mapping_bringup mapping_v0.launch.py \
  bag_path:=/home/yangxuan/ros2_ws/src/rosbag/bunker_mid360_mapping_20260901_205036 \
  output_dir:=/data/agt_mapping_runs/mid360_20260901_205036 \
  start_rviz:=true
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
