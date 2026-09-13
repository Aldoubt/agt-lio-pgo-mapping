#pragma once

#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace agt_mid360_adapter
{

sensor_msgs::msg::PointCloud2 customMsgToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & frame_id_override = "");

}  // namespace agt_mid360_adapter
