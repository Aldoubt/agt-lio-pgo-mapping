#pragma once

#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace agt_mid360_adapter
{

sensor_msgs::msg::PointCloud2 customMsgToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & frame_id_override = "");

// Deskew-capable output. Keeps the real Livox line as `ring`, exports `time`
// in seconds relative to the CustomMsg header, and retains the exact uint32
// nanosecond `offset_time` for audits. Points are stably sorted by time.
sensor_msgs::msg::PointCloud2 customMsgToTimedPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & frame_id_override = "");

}  // namespace agt_mid360_adapter
