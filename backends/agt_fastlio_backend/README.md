# agt_fastlio_backend

Adapter for an externally supplied FAST-LIO2 node. It subscribes to the existing
FAST-LIO2 output topics and republishes their messages unchanged through the
mapping frontend contract:

| FAST-LIO2 input | Default framework output |
| --- | --- |
| `/fastlio2/lio_odom` | `/mapping/frontend/odometry` |
| `/fastlio2/body_cloud` | `/mapping/frontend/cloud` |
| `/fastlio2/lio_path` | `/mapping/frontend/path` |

Use `fastlio_backend.launch.py` to configure `input_odom_topic`,
`input_cloud_topic`, optional `input_path_topic`, and `output_namespace`.
The package does not contain, build, configure or modify FAST-LIO2 source.
