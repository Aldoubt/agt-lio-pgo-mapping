#pragma once

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace agt_pgo_backend
{

class PgoBridge
{
public:
  explicit PgoBridge(double keyframe_distance_m);
  bool acceptOdometry(const nav_msgs::msg::Odometry & odometry);
  geometry_msgs::msg::PoseStamped mapPose(const nav_msgs::msg::Odometry & odometry) const;
  const nav_msgs::msg::Path & keyframes() const;
  const std::string & artifactTriggerStatus() const;
  void triggerArtifactExport();

private:
  double keyframe_distance_m_;
  bool has_keyframe_{false};
  geometry_msgs::msg::Pose last_keyframe_pose_;
  nav_msgs::msg::Path keyframes_;
  std::string artifact_trigger_status_{"idle"};
};

}  // namespace agt_pgo_backend
