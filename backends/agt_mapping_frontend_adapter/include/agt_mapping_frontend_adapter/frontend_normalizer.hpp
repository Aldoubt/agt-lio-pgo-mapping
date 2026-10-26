#pragma once

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace agt_mapping_frontend_adapter
{

struct RigidTransform
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string & target, const std::string & source,
  const RigidTransform & value, const builtin_interfaces::msg::Time & stamp);

nav_msgs::msg::Odometry normalizeOdometry(
  const nav_msgs::msg::Odometry & input, const std::string & cloud_frame_mode,
  const std::string & body_frame, const RigidTransform & body_from_lidar);

sensor_msgs::msg::PointCloud2 normalizeCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const nav_msgs::msg::Odometry & odometry,
  const std::string & cloud_frame_mode, const std::string & body_frame,
  const std::string & lidar_frame, const RigidTransform & body_from_lidar);

nav_msgs::msg::Path normalizePath(
  const nav_msgs::msg::Path & input, const std::string & cloud_frame_mode,
  const std::string & body_frame, const RigidTransform & body_from_lidar);

}  // namespace agt_mapping_frontend_adapter
