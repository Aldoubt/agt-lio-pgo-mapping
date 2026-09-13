#include <memory>
#include <string>

#include "agt_mapping_backend_api/backend_topics.hpp"
#include "agt_pgo_backend/pgo_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace agt_pgo_backend
{

class PgoBackendNode : public rclcpp::Node
{
public:
  PgoBackendNode() : Node("pgo_backend_node"), bridge_(declare_parameter<double>("keyframe_distance_m", 0.5))
  {
    const auto cloud_topic = declare_parameter<std::string>(
      "input_cloud_topic", agt_mapping_backend_api::kFrontendCloud);
    const auto odometry_topic = declare_parameter<std::string>(
      "input_odometry_topic", agt_mapping_backend_api::kFrontendOdometry);
    map_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      agt_mapping_backend_api::kMapPose, rclcpp::QoS(20));
    keyframes_publisher_ = create_publisher<nav_msgs::msg::Path>(
      agt_mapping_backend_api::kKeyframes, rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      agt_mapping_backend_api::kStatus, rclcpp::QoS(10));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(), [](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {});
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
        map_pose_publisher_->publish(bridge_.mapPose(*odometry));
        if (bridge_.acceptOdometry(*odometry)) {
          keyframes_publisher_->publish(bridge_.keyframes());
          publishStatus("keyframe_accepted");
        }
      });
    export_service_ = create_service<std_srvs::srv::Trigger>(
      agt_mapping_backend_api::kArtifactTrigger,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        bridge_.triggerArtifactExport();
        publishStatus(bridge_.artifactTriggerStatus());
        response->success = true;
        response->message = bridge_.artifactTriggerStatus();
      });
    publishStatus("ready_external_pgo");
  }

private:
  void publishStatus(const std::string & value)
  {
    std_msgs::msg::String status;
    status.data = value;
    status_publisher_->publish(status);
  }

  PgoBridge bridge_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr map_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframes_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
};

}  // namespace agt_pgo_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_pgo_backend::PgoBackendNode>());
  rclcpp::shutdown();
  return 0;
}
