#include <memory>
#include <string>

#include "agt_fastlio_backend/frontend_relay.hpp"
#include "agt_mapping_frontend_api/frontend_topics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agt_fastlio_backend
{

class FastLioBackendNode : public rclcpp::Node
{
public:
  FastLioBackendNode() : Node("fastlio_backend_node")
  {
    const auto odom_input = declare_parameter<std::string>("input_odom_topic", "/fastlio2/lio_odom");
    const auto cloud_input = declare_parameter<std::string>("input_cloud_topic", "/fastlio2/body_cloud");
    const auto path_input = declare_parameter<std::string>("input_path_topic", "/fastlio2/lio_path");
    const auto output_namespace = declare_parameter<std::string>(
      "output_namespace", agt_mapping_frontend_api::kDefaultNamespace);
    const auto output = agt_mapping_frontend_api::makeFrontendTopics(output_namespace);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output.odometry, rclcpp::QoS(20));
    // The external PGO subscriber uses the ROS 2 default (reliable) QoS.  Keep
    // the framework boundary reliable so PGO can consume the relayed cloud;
    // the input subscription remains SensorDataQoS for FAST-LIO2 compatibility.
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output.cloud, rclcpp::QoS(20));
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(output.path, rclcpp::QoS(10));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(odom_input, rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { odom_publisher_->publish(relayOdometry(*message)); });
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { cloud_publisher_->publish(relayCloud(*message)); });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(path_input, rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) { path_publisher_->publish(relayPath(*message)); });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
};

}  // namespace agt_fastlio_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_fastlio_backend::FastLioBackendNode>());
  rclcpp::shutdown();
  return 0;
}
