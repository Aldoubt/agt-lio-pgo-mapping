#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace agt_fastlio_backend
{

nav_msgs::msg::Odometry relayOdometry(const nav_msgs::msg::Odometry & message);
sensor_msgs::msg::PointCloud2 relayCloud(const sensor_msgs::msg::PointCloud2 & message);
nav_msgs::msg::Path relayPath(const nav_msgs::msg::Path & message);

}  // namespace agt_fastlio_backend
