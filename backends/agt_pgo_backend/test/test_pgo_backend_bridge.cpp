#include <gtest/gtest.h>

#include "agt_mapping_backend_api/backend_topics.hpp"
#include "agt_pgo_backend/pgo_bridge.hpp"

TEST(PgoBackendBridge, DeclaresStableFrontendAndBackendTopics)
{
  EXPECT_STREQ(agt_mapping_backend_api::kFrontendCloud, "/mapping/frontend/cloud");
  EXPECT_STREQ(agt_mapping_backend_api::kFrontendOdometry, "/mapping/frontend/odometry");
  EXPECT_STREQ(agt_mapping_backend_api::kMapPose, "/mapping/backend/map_pose");
  EXPECT_STREQ(agt_mapping_backend_api::kKeyframes, "/mapping/backend/keyframes");
  EXPECT_STREQ(agt_mapping_backend_api::kStatus, "/mapping/backend/status");
}

TEST(PgoBackendBridge, MockFrontendOdometryProducesPoseAndKeyframe)
{
  agt_pgo_backend::PgoBridge bridge(0.5);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = 42;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 1.0;

  EXPECT_TRUE(bridge.acceptOdometry(odometry));
  const auto pose = bridge.mapPose(odometry);
  EXPECT_EQ(pose.header.stamp, odometry.header.stamp);
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  ASSERT_EQ(bridge.keyframes().poses.size(), 1U);
  EXPECT_EQ(bridge.keyframes().poses.front().header.stamp, odometry.header.stamp);

  odometry.pose.pose.position.x = 1.2;
  EXPECT_FALSE(bridge.acceptOdometry(odometry));
  odometry.pose.pose.position.x = 1.6;
  EXPECT_TRUE(bridge.acceptOdometry(odometry));
  EXPECT_EQ(bridge.keyframes().poses.size(), 2U);
}

TEST(PgoBackendBridge, ArtifactTriggerPublishesRequestState)
{
  agt_pgo_backend::PgoBridge bridge(0.5);
  EXPECT_EQ(bridge.artifactTriggerStatus(), "idle");
  bridge.triggerArtifactExport();
  EXPECT_EQ(bridge.artifactTriggerStatus(), "artifact_export_requested");
}
