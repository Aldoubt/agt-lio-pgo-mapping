#include <gtest/gtest.h>

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
