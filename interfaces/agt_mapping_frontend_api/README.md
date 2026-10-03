# agt_mapping_frontend_api

Defines the backend-neutral mapping frontend topic contract. A frontend publishes:

- `<output_namespace>/odometry` — `nav_msgs/msg/Odometry`
- `<output_namespace>/cloud` — `sensor_msgs/msg/PointCloud2`
- `<output_namespace>/path` — `nav_msgs/msg/Path`
- `<output_namespace>/status` — `diagnostic_msgs/msg/DiagnosticArray`

The default namespace is `/mapping/frontend`.

`odometry` is the backend's frontend pose in its map/odom-like parent frame;
`child_frame_id` names the normalized body frame. The pose is `T_frontend_body`.
`cloud` is the current scan/keyframe's local cloud expressed in that same body
frame, paired with the native odometry by timestamp. Its header stamp is kept
unchanged. The adapter may apply only declared rigid frame transforms; it does
not smooth, refilter, or estimate pose. Each profile records the body/LiDAR
extrinsic. `path` contains the same normalized body poses. `status` records the
backend identity and source revision, observed rates, adapter latency, last
sensor/output stamps, and `UNKNOWN` for estimator drop or processing counts the
native process does not expose.
