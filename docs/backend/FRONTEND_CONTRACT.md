# Unified LIO frontend contract

The contract is implemented by `agt_mapping_frontend_adapter` and declared in
`agt_mapping_frontend_api`:

| Topic | Type | Meaning |
|---|---|---|
| `/mapping/frontend/odometry` | `nav_msgs/msg/Odometry` | `T_frontend_body`, with the backend's map/odom-like frame as parent and the canonical body frame as child |
| `/mapping/frontend/cloud` | `sensor_msgs/msg/PointCloud2` | One current scan/keyframe cloud expressed in the same local body frame |
| `/mapping/frontend/path` | `nav_msgs/msg/Path` | Native path after any declared rigid body-frame conversion; if a backend has no path publisher, a sampled path is formed from normalized odometry |
| `/mapping/frontend/status` | `diagnostic_msgs/msg/DiagnosticArray` | Backend ID/revision, input and output rates, adapter latency, stamps, and estimator fields marked `UNKNOWN` when not exposed |

The frontend cloud and odometry are approximate-time paired (maximum slop is
profiled, currently 50 ms). Their original message stamps remain unchanged.
The map exporter records each selected cloud and the exact normalized pose
paired with that cloud. It does not query or branch on a backend topic.

## Input timing

MID360's raw input is
`/agt/sensors/lidar/custom` (`livox_ros_driver2/msg/CustomMsg`) and IMU input is
`/agt/sensors/imu/data` (`sensor_msgs/msg/Imu`). LIO-SAM and Point-LIO consume
`/mapping/sensor/deskew_cloud`, which keeps the real `line` as `ring`, the
header-relative `offset_time` in exact nanoseconds, and `time` in seconds for
the existing estimators. It stably sorts points by offset and applies the same
invalid-tag/nonfinite filtering as the historic `lio_benchmark_tools`
converter. The plain XYZI adapter topic `/mapping/sensor/cloud` still drops
per-point timing and must not feed a deskewing estimator.

FAST-LIVO2 receives native `CustomMsg` directly so its own offset-time path is
preserved. LIO-SAM alone gets a copied IMU message with acceleration converted
from g to m/s² and covariance scaled by the square of that factor. Point-LIO
and FAST-LIVO2 keep the historic raw-g input/configuration. FAST-LIO2 remains
explicitly experimental and receives the existing validated CustomMsg relay.

## Frames

The estimator pose is not recomputed or smoothed. Adapters only apply the
profiled static `T_body_lidar` and, for LIO-SAM's registered world cloud, the
inverse of the paired native pose before expressing the cloud in body
coordinates. The normalized odometry child frame is set to the profile's body
frame. Profiles record the map, body, LiDAR, and IMU names and the exact
translation/quaternion used. The cloud header frame is always the body frame.

Processing time reported by an estimator is `UNKNOWN` unless the estimator
exports it. Diagnostics separately measure adapter compute time and
frontend-output delay against simulated bag time; these are not represented as
estimator processing time or drop counts.
