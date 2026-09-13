# agt_mapping_backend_api

Defines the optimization backend contract:

| Direction | Topic / service | Type |
| --- | --- | --- |
| Input | `/mapping/frontend/cloud` | `sensor_msgs/msg/PointCloud2` |
| Input | `/mapping/frontend/odometry` | `nav_msgs/msg/Odometry` |
| Output | `/mapping/backend/map_pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/mapping/backend/keyframes` | `agt_mapping_backend_api/msg/KeyframeArray` |
| Output | `/mapping/backend/status` | `std_msgs/msg/String` |
| Trigger | `/mapping/backend/export_artifact` | `std_srvs/srv/Trigger` |

The API fixes names and message types only. It does not prescribe an optimizer or map artifact format.

Each `Keyframe` carries timestamp, pose and `cloud_reference`. Before a PGO export succeeds, no keyframe is publishable as optimized because its cloud reference is unavailable.
