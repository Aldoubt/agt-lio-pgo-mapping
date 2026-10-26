#include "agt_mid360_adapter/custommsg_to_pointcloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace agt_mid360_adapter
{

namespace
{
constexpr uint32_t kTimedPointStep = 26;

template<typename T>
void writeValue(std::vector<uint8_t> & data, size_t offset, T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

sensor_msgs::msg::PointField makeField(
  const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}
}  // namespace

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

sensor_msgs::msg::PointCloud2 customMsgToTimedPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & input,
  const std::string & frame_id_override)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  if (!frame_id_override.empty()) {
    output.header.frame_id = frame_id_override;
  }
  std::vector<size_t> selected;
  selected.reserve(input.points.size());
  for (size_t index = 0; index < input.points.size(); ++index) {
    const auto & point = input.points[index];
    const auto tag = static_cast<uint8_t>(point.tag & 0x30U);
    if ((tag == 0x00U || tag == 0x10U) && std::isfinite(point.x) &&
      std::isfinite(point.y) && std::isfinite(point.z))
    {
      selected.push_back(index);
    }
  }
  std::stable_sort(selected.begin(), selected.end(), [&](size_t lhs, size_t rhs) {
    return input.points[lhs].offset_time < input.points[rhs].offset_time;
  });

  output.height = 1;
  output.width = static_cast<uint32_t>(selected.size());
  output.fields = {
    makeField("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    makeField("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    makeField("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    makeField("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
    makeField("ring", 16, sensor_msgs::msg::PointField::UINT16),
    makeField("time", 18, sensor_msgs::msg::PointField::FLOAT32),
    makeField("offset_time", 22, sensor_msgs::msg::PointField::UINT32),
  };
  output.is_bigendian = false;
  output.point_step = kTimedPointStep;
  output.row_step = output.point_step * output.width;
  output.is_dense = true;
  output.data.resize(output.row_step);
  for (size_t index = 0; index < selected.size(); ++index) {
    const auto & point = input.points[selected[index]];
    const size_t offset = index * kTimedPointStep;
    writeValue(output.data, offset + 0, point.x);
    writeValue(output.data, offset + 4, point.y);
    writeValue(output.data, offset + 8, point.z);
    writeValue(output.data, offset + 12, static_cast<float>(point.reflectivity));
    writeValue(output.data, offset + 16, static_cast<uint16_t>(point.line));
    writeValue(output.data, offset + 18, static_cast<float>(point.offset_time) * 1.0e-9F);
    writeValue(output.data, offset + 22, point.offset_time);
  }
  return output;
}

}  // namespace agt_mid360_adapter
