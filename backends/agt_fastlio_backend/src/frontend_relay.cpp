#include "agt_fastlio_backend/frontend_relay.hpp"

namespace agt_fastlio_backend
{

nav_msgs::msg::Odometry relayOdometry(const nav_msgs::msg::Odometry & message) { return message; }
sensor_msgs::msg::PointCloud2 relayCloud(const sensor_msgs::msg::PointCloud2 & message) { return message; }
nav_msgs::msg::Path relayPath(const nav_msgs::msg::Path & message) { return message; }

}  // namespace agt_fastlio_backend
