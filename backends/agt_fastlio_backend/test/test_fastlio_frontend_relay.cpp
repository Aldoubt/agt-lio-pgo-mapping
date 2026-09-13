#include <gtest/gtest.h>

#include "agt_fastlio_backend/frontend_relay.hpp"
#include "agt_mapping_frontend_api/frontend_topics.hpp"

TEST(FastLioFrontendRelay, MapsConfiguredNamespaceToFrontendTopics)
{
  const auto topics = agt_mapping_frontend_api::makeFrontendTopics("mapping/test_frontend/");
  EXPECT_EQ(topics.odometry, "/mapping/test_frontend/odometry");
  EXPECT_EQ(topics.cloud, "/mapping/test_frontend/cloud");
  EXPECT_EQ(topics.path, "/mapping/test_frontend/path");
}

TEST(FastLioFrontendRelay, PreservesOdometryTimestampFrameAndPayload)
{
  nav_msgs::msg::Odometry input;
  input.header.stamp.sec = 10;
  input.header.stamp.nanosec = 20;
  input.header.frame_id = "lidar";
  input.child_frame_id = "body";
  input.pose.pose.position.x = 1.5;
  const auto output = agt_fastlio_backend::relayOdometry(input);
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.header.frame_id, "lidar");
  EXPECT_EQ(output.child_frame_id, "body");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 1.5);
}

TEST(FastLioFrontendRelay, PreservesCloudAndPathHeaders)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 30;
  cloud.header.frame_id = "body";
  cloud.width = 7;
  const auto output_cloud = agt_fastlio_backend::relayCloud(cloud);
  EXPECT_EQ(output_cloud.header.stamp, cloud.header.stamp);
  EXPECT_EQ(output_cloud.header.frame_id, "body");
  EXPECT_EQ(output_cloud.width, 7U);

  nav_msgs::msg::Path path;
  path.header.stamp.sec = 40;
  path.header.frame_id = "lidar";
  const auto output_path = agt_fastlio_backend::relayPath(path);
  EXPECT_EQ(output_path.header.stamp, path.header.stamp);
  EXPECT_EQ(output_path.header.frame_id, "lidar");
}
