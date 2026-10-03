#include "agt_mapping_frontend_adapter/frontend_normalizer.hpp"

#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace agt_mapping_frontend_adapter
{
namespace
{
tf2::Transform transformFromPose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

tf2::Transform bodyFromLidar(const RigidTransform & value)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(value.x, value.y, value.z));
  transform.setRotation(tf2::Quaternion(value.qx, value.qy, value.qz, value.qw).normalized());
  return transform;
}

geometry_msgs::msg::Pose poseFromTransform(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose result;
  result.position.x = transform.getOrigin().x();
  result.position.y = transform.getOrigin().y();
  result.position.z = transform.getOrigin().z();
  result.orientation = tf2::toMsg(transform.getRotation());
  return result;
}

void requireMode(const std::string & mode)
{
  if (mode != "body" && mode != "lidar" && mode != "frontend") {
    throw std::invalid_argument("cloud_frame_mode must be body, lidar, or frontend");
  }
}
}  // namespace

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string & target, const std::string & source, const RigidTransform & value,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped output;
  output.header.stamp = stamp;
  output.header.frame_id = target;
  output.child_frame_id = source;
  output.transform.translation.x = value.x;
  output.transform.translation.y = value.y;
  output.transform.translation.z = value.z;
  output.transform.rotation.x = value.qx;
  output.transform.rotation.y = value.qy;
  output.transform.rotation.z = value.qz;
  output.transform.rotation.w = value.qw;
  return output;
}

nav_msgs::msg::Odometry normalizeOdometry(
  const nav_msgs::msg::Odometry & input, const std::string & cloud_frame_mode,
  const std::string & body_frame, const RigidTransform & body_from_lidar)
{
  requireMode(cloud_frame_mode);
  nav_msgs::msg::Odometry output = input;
  if (cloud_frame_mode == "frontend") {
    // LIO-SAM publishes the registered scan and its pose in the frontend
    // frame as a LiDAR pose. Convert T_frontend_lidar to T_frontend_body.
    const tf2::Transform frontend_from_lidar = transformFromPose(input.pose.pose);
    const tf2::Transform body_from_lidar_transform = bodyFromLidar(body_from_lidar);
    output.pose.pose = poseFromTransform(frontend_from_lidar * body_from_lidar_transform.inverse());
  }
  output.child_frame_id = body_frame;
  return output;
}

sensor_msgs::msg::PointCloud2 normalizeCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const nav_msgs::msg::Odometry & odometry,
  const std::string & cloud_frame_mode, const std::string & body_frame,
  const std::string & lidar_frame, const RigidTransform & body_from_lidar)
{
  requireMode(cloud_frame_mode);
  sensor_msgs::msg::PointCloud2 lidar_cloud = cloud;
  if (cloud_frame_mode == "frontend") {
    if (cloud.header.frame_id != odometry.header.frame_id) {
      throw std::invalid_argument("frontend cloud and odometry parent frames differ");
    }
    const tf2::Transform frontend_from_lidar = transformFromPose(odometry.pose.pose);
    const auto lidar_from_frontend_msg = tf2::toMsg(frontend_from_lidar.inverse());
    geometry_msgs::msg::TransformStamped lidar_from_frontend;
    lidar_from_frontend.header = cloud.header;
    lidar_from_frontend.header.frame_id = lidar_frame;
    lidar_from_frontend.child_frame_id = cloud.header.frame_id;
    lidar_from_frontend.transform = lidar_from_frontend_msg;
    tf2::doTransform(cloud, lidar_cloud, lidar_from_frontend);
  } else if (cloud_frame_mode == "lidar") {
    lidar_cloud = cloud;
  } else {
    sensor_msgs::msg::PointCloud2 output = cloud;
    output.header.frame_id = body_frame;
    return output;
  }

  const auto body_from_lidar_msg = makeTransform(
    body_frame, lidar_frame, body_from_lidar, cloud.header.stamp);
  sensor_msgs::msg::PointCloud2 output;
  tf2::doTransform(lidar_cloud, output, body_from_lidar_msg);
  output.header.stamp = cloud.header.stamp;
  output.header.frame_id = body_frame;
  return output;
}

nav_msgs::msg::Path normalizePath(
  const nav_msgs::msg::Path & input, const std::string & cloud_frame_mode,
  const std::string & body_frame, const RigidTransform & body_from_lidar)
{
  requireMode(cloud_frame_mode);
  nav_msgs::msg::Path output = input;
  if (cloud_frame_mode == "frontend") {
    const tf2::Transform body_from_lidar_transform = bodyFromLidar(body_from_lidar);
    for (auto & pose : output.poses) {
      const tf2::Transform frontend_from_lidar = transformFromPose(pose.pose);
      pose.pose = poseFromTransform(frontend_from_lidar * body_from_lidar_transform.inverse());
    }
  }
  for (auto & pose : output.poses) {
    pose.header.frame_id = input.header.frame_id;
  }
  return output;
}

}  // namespace agt_mapping_frontend_adapter
