#include "agt_mid360_adapter/custommsg_to_pointcloud.hpp"

#include <cstdint>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace agt_mid360_adapter
{

sensor_msgs::msg::PointCloud2 customMsgToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & frame_id_override)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  if (!frame_id_override.empty()) {
    output.header.frame_id = frame_id_override;
  }
  output.height = 1;
  output.width = static_cast<uint32_t>(input.points.size());
  output.is_bigendian = false;
  output.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(input.points.size());

  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
  for (const auto & point : input.points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    *intensity = static_cast<float>(point.reflectivity);
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return output;
}

}  // namespace agt_mid360_adapter
