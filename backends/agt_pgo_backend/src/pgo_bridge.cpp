#include "agt_pgo_backend/pgo_bridge.hpp"

#include <cmath>

namespace agt_pgo_backend
{

PgoBridge::PgoBridge(double keyframe_distance_m)
: keyframe_distance_m_(keyframe_distance_m)
{
}

bool PgoBridge::acceptOdometry(const nav_msgs::msg::Odometry & odometry)
{
  const auto & position = odometry.pose.pose.position;
  if (has_keyframe_) {
    const auto & previous = last_keyframe_pose_.position;
    const double dx = position.x - previous.x;
    const double dy = position.y - previous.y;
    const double dz = position.z - previous.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < keyframe_distance_m_) {
      return false;
    }
  }
  geometry_msgs::msg::PoseStamped keyframe;
  keyframe.header = odometry.header;
  keyframe.pose = odometry.pose.pose;
  keyframes_.header = odometry.header;
  keyframes_.poses.push_back(keyframe);
  last_keyframe_pose_ = odometry.pose.pose;
  has_keyframe_ = true;
  return true;
}

geometry_msgs::msg::PoseStamped PgoBridge::mapPose(const nav_msgs::msg::Odometry & odometry) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  return pose;
}

const nav_msgs::msg::Path & PgoBridge::keyframes() const { return keyframes_; }
const std::string & PgoBridge::artifactTriggerStatus() const { return artifact_trigger_status_; }
void PgoBridge::triggerArtifactExport() { artifact_trigger_status_ = "artifact_export_requested"; }

}  // namespace agt_pgo_backend
