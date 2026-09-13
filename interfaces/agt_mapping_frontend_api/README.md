# agt_mapping_frontend_api

Defines the backend-neutral mapping frontend topic contract. A frontend publishes:

- `<output_namespace>/odometry` — `nav_msgs/msg/Odometry`
- `<output_namespace>/cloud` — `sensor_msgs/msg/PointCloud2`
- `<output_namespace>/path` — `nav_msgs/msg/Path`

The default namespace is `/mapping/frontend`. The API does not define an LIO implementation or modify message content.
