#include <gtest/gtest.h>

#include <cstring>

#include "agt_mid360_adapter/custommsg_to_pointcloud.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

TEST(CustomMsgToPointCloud, PreservesHeaderCoordinatesAndIntensity)
{
  livox_ros_driver2::msg::CustomMsg input;
  input.header.stamp.sec = 123;
  input.header.stamp.nanosec = 456;
  input.header.frame_id = "mid360";
  livox_ros_driver2::msg::CustomPoint first;
  first.x = 1.0F;
  first.y = -2.0F;
  first.z = 3.5F;
  first.reflectivity = 42;
  input.points.push_back(first);
  livox_ros_driver2::msg::CustomPoint second;
  second.x = -4.0F;
  second.y = 5.0F;
  second.z = 6.0F;
  second.reflectivity = 255;
  input.points.push_back(second);

  const auto output = agt_mid360_adapter::customMsgToPointCloud2(input);
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.header.frame_id, "mid360");
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 2U);

  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(output, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(output, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(output, "intensity");
  EXPECT_FLOAT_EQ(*x, 1.0F);
  EXPECT_FLOAT_EQ(*y, -2.0F);
  EXPECT_FLOAT_EQ(*z, 3.5F);
  EXPECT_FLOAT_EQ(*intensity, 42.0F);
  ++x; ++y; ++z; ++intensity;
  EXPECT_FLOAT_EQ(*x, -4.0F);
  EXPECT_FLOAT_EQ(*y, 5.0F);
  EXPECT_FLOAT_EQ(*z, 6.0F);
  EXPECT_FLOAT_EQ(*intensity, 255.0F);
}

TEST(CustomMsgToPointCloud, OverridesFrameOnlyWhenConfigured)
{
  livox_ros_driver2::msg::CustomMsg input;
  input.header.frame_id = "mid360";
  EXPECT_EQ(agt_mid360_adapter::customMsgToPointCloud2(input).header.frame_id, "mid360");
  EXPECT_EQ(
    agt_mid360_adapter::customMsgToPointCloud2(input, "mapping_lidar").header.frame_id,
    "mapping_lidar");
}

TEST(CustomMsgToPointCloud, TimedConversionPreservesDeskewFieldsAndSortsOffsets)
{
  livox_ros_driver2::msg::CustomMsg input;
  input.header.frame_id = "livox_frame";
  input.header.stamp.sec = 3;
  input.points.resize(3);
  input.points[0].x = 1.0F;
  input.points[0].line = 2;
  input.points[0].reflectivity = 42;
  input.points[0].offset_time = 20000000U;
  input.points[0].tag = 0x10;
  input.points[1].x = 2.0F;
  input.points[1].line = 1;
  input.points[1].reflectivity = 43;
  input.points[1].offset_time = 10000000U;
  input.points[1].tag = 0x00;
  input.points[2].x = 3.0F;
  input.points[2].line = 3;
  input.points[2].offset_time = 30000000U;
  input.points[2].tag = 0x20;  // Filtered by the historical LIO adapter rule.

  const auto output = agt_mid360_adapter::customMsgToTimedPointCloud2(input);
  ASSERT_EQ(output.width, 2U);
  ASSERT_EQ(output.point_step, 26U);
  ASSERT_EQ(output.header.stamp.sec, 3);
  ASSERT_EQ(output.fields.size(), 7U);
  EXPECT_EQ(output.fields[4].name, "ring");
  EXPECT_EQ(output.fields[5].name, "time");
  EXPECT_EQ(output.fields[6].name, "offset_time");
  uint16_t first_ring = 0;
  float first_time = 0.0F;
  uint32_t first_offset = 0;
  std::memcpy(&first_ring, output.data.data() + 16, sizeof(first_ring));
  std::memcpy(&first_time, output.data.data() + 18, sizeof(first_time));
  std::memcpy(&first_offset, output.data.data() + 22, sizeof(first_offset));
  EXPECT_EQ(first_ring, 1U);
  EXPECT_FLOAT_EQ(first_time, 0.01F);
  EXPECT_EQ(first_offset, 10000000U);
}
