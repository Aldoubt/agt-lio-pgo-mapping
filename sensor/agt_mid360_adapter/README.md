# agt_mid360_adapter

ROS 2 Humble adapter from `livox_ros_driver2/msg/CustomMsg` to
`sensor_msgs/msg/PointCloud2`.

`mid360_adapter_node` subscribes to `/livox/lidar` and publishes
`/mapping/sensor/cloud` by default. It preserves the input message timestamp,
uses the input `frame_id` unless the optional `frame_id` parameter explicitly
overrides it, and converts Livox `reflectivity` to the standard float32
`intensity` field.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input_topic` | `/livox/lidar` | Input CustomMsg topic. |
| `output_topic` | `/mapping/sensor/cloud` | Output PointCloud2 topic. |
| `frame_id` | `""` | Output frame override; empty preserves input frame. |

The package intentionally contains no filtering, deskewing, LIO, PGO or navigation integration.
