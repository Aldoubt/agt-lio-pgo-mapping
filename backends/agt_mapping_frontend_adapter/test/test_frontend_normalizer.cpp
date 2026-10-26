#include <gtest/gtest.h>

#include <cstring>

#include "agt_mapping_frontend_adapter/frontend_normalizer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using agt_mapping_frontend_adapter::RigidTransform;

TEST(FrontendNormalizer, BodyCloudAndOdometryUseContractFrames)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "aft_mapped";
  odom.pose.pose.orientation.w = 1.0;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "body";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  {
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    *x = 1.0F;
  }

  const auto normalized_odom = agt_mapping_frontend_adapter::normalizeOdometry(
    odom, "body", "robot_body", RigidTransform{});
  const auto normalized_cloud = agt_mapping_frontend_adapter::normalizeCloud(
    cloud, odom, "body", "robot_body", "lidar", RigidTransform{});
  EXPECT_EQ(normalized_odom.header.frame_id, "odom");
  EXPECT_EQ(normalized_odom.child_frame_id, "robot_body");
  EXPECT_EQ(normalized_cloud.header.frame_id, "robot_body");
  EXPECT_EQ(normalized_cloud.header.stamp, cloud.header.stamp);
}

TEST(FrontendNormalizer, FrontendFrameCloudIsReturnedInBodyCoordinates)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = 10.0;
  odom.pose.pose.orientation.w = 1.0;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "odom";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  {
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    *x = 11.0F;
  }
  const auto output = agt_mapping_frontend_adapter::normalizeCloud(
    cloud, odom, "frontend", "body", "lidar", RigidTransform{});
  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x");
  EXPECT_NEAR(*x, 1.0F, 1e-5);
  EXPECT_EQ(output.header.frame_id, "body");
}

TEST(FrontendNormalizer, NoLoopCloudTransformKeepsTheConfiguredLeverArm)
{
  RigidTransform body_from_lidar;
  body_from_lidar.x = 1.0;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = 5.0;
  odom.pose.pose.orientation.w = 1.0;
  const auto normalized = agt_mapping_frontend_adapter::normalizeOdometry(
    odom, "frontend", "body", body_from_lidar);
  EXPECT_NEAR(normalized.pose.pose.position.x, 4.0, 1e-9);
  EXPECT_EQ(normalized.child_frame_id, "body");
}
