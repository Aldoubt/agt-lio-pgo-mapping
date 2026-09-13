#include <memory>
#include <string>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>

#include "agt_mapping_backend_api/backend_topics.hpp"
#include "agt_pgo_backend/pgo_bridge.hpp"
#include "agt_mapping_backend_api/msg/keyframe_array.hpp"
#include "interface/srv/save_maps.hpp"
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
    keyframes_publisher_ = create_publisher<agt_mapping_backend_api::msg::KeyframeArray>(
      agt_mapping_backend_api::kKeyframes, rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      agt_mapping_backend_api::kStatus, rclcpp::QoS(10));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(), [](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {});
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
        map_pose_publisher_->publish(bridge_.mapPose(*odometry));
        if (bridge_.acceptOdometry(*odometry)) {
          publishStatus("frontend_keyframe_pending_pgo_optimization");
        }
      });
    pgo_output_dir_ = declare_parameter<std::string>("pgo_output_dir", "output/pgo_raw");
    save_maps_client_ = create_client<interface::srv::SaveMaps>("/pgo/save_maps");
    export_service_ = create_service<std_srvs::srv::Trigger>(
      agt_mapping_backend_api::kArtifactTrigger,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!save_maps_client_->wait_for_service(std::chrono::seconds(2))) {
          response->success = false;
          response->message = "external PGO save service unavailable";
          publishStatus("pgo_save_service_unavailable");
          return;
        }
        std::filesystem::create_directories(pgo_output_dir_);
        auto request = std::make_shared<interface::srv::SaveMaps::Request>();
        request->file_path = pgo_output_dir_;
        request->save_patches = true;
        save_maps_client_->async_send_request(request,
          [this](rclcpp::Client<interface::srv::SaveMaps>::SharedFuture future) {
            const auto result = future.get();
            if (!result->success) { publishStatus("pgo_export_failed"); return; }
            publishOptimizedOutput();
          });
        response->success = true;
        response->message = "external PGO artifact export requested";
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

  void publishOptimizedOutput()
  {
    std::ifstream poses(std::filesystem::path(pgo_output_dir_) / "poses_timed.txt");
    agt_mapping_backend_api::msg::KeyframeArray result;
    result.header.frame_id = "map";
    std::string patch;
    double stamp, x, y, z, qw, qx, qy, qz;
    while (poses >> patch >> stamp >> x >> y >> z >> qw >> qx >> qy >> qz) {
      agt_mapping_backend_api::msg::Keyframe keyframe;
      const auto seconds = static_cast<int32_t>(stamp);
      keyframe.stamp.sec = seconds;
      keyframe.stamp.nanosec = static_cast<uint32_t>((stamp - seconds) * 1e9);
      keyframe.pose.position.x = x; keyframe.pose.position.y = y; keyframe.pose.position.z = z;
      keyframe.pose.orientation.w = qw; keyframe.pose.orientation.x = qx;
      keyframe.pose.orientation.y = qy; keyframe.pose.orientation.z = qz;
      keyframe.cloud_reference = (std::filesystem::path(pgo_output_dir_) / "patches" / patch).string();
      result.keyframes.push_back(keyframe);
    }
    if (result.keyframes.empty()) { publishStatus("pgo_export_empty"); return; }
    keyframes_publisher_->publish(result);
    geometry_msgs::msg::PoseStamped map_pose;
    map_pose.header = result.header;
    map_pose.header.stamp = result.keyframes.back().stamp;
    map_pose.pose = result.keyframes.back().pose;
    map_pose_publisher_->publish(map_pose);
    publishStatus("{\"backend\":\"PGO\",\"optimized\":true,\"artifact_source\":\"" + pgo_output_dir_ + "\"}");
  }

  PgoBridge bridge_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr map_pose_publisher_;
  rclcpp::Publisher<agt_mapping_backend_api::msg::KeyframeArray>::SharedPtr keyframes_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
  rclcpp::Client<interface::srv::SaveMaps>::SharedPtr save_maps_client_;
  std::string pgo_output_dir_;
};

}  // namespace agt_pgo_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_pgo_backend::PgoBackendNode>());
  rclcpp::shutdown();
  return 0;
}
