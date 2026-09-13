#include <algorithm>
#include <memory>
#include <string>

#include "agt_mid360_adapter/custommsg_to_pointcloud.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace agt_mid360_adapter
{

class Mid360AdapterNode : public rclcpp::Node
{
public:
  Mid360AdapterNode()
  : Node("mid360_adapter_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/mapping/sensor/cloud");
    const auto sanitized_topic = declare_parameter<std::string>("sanitized_topic", "/mapping/sensor/livox");
    min_points_ = declare_parameter<int>("min_points", 1000);
    max_scan_duration_s_ = declare_parameter<double>("max_scan_duration_s", 0.2);
    frame_id_override_ = declare_parameter<std::string>("frame_id", "");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
    // FAST-LIO2's external node uses the default reliable subscription QoS.
    // This internal processing stream must therefore be reliable; the public
    // PointCloud2 sensor stream above intentionally remains SensorDataQoS.
    sanitized_publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      sanitized_topic, rclcpp::QoS(20));
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
        // Livox point order is not guaranteed to be monotonic in every
        // recorded packet.  Use the largest offset in the packet instead of
        // the final element when deciding whether this is one full scan.
        const auto max_offset = message->points.empty() ? 0U :
          std::max_element(
          message->points.begin(), message->points.end(),
          [](const auto & lhs, const auto & rhs) {
            return lhs.offset_time < rhs.offset_time;
          })->offset_time;
        const double duration_s = static_cast<double>(max_offset) / 1e9;
        if (message->point_num != message->points.size() ||
          static_cast<int>(message->points.size()) < min_points_ ||
          duration_s <= 0.0 || duration_s > max_scan_duration_s_) {
          ++dropped_scans_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Dropped non-aggregate Livox message (points=%zu, duration=%.3fs, total=%zu)",
            message->points.size(), duration_s, dropped_scans_);
          return;
        }
        sanitized_publisher_->publish(*message);
        publisher_->publish(customMsgToPointCloud2(*message, frame_id_override_));
      });
    RCLCPP_INFO(
      get_logger(), "MID360 adapter: %s -> %s; frame override: %s",
      input_topic.c_str(), output_topic.c_str(),
      frame_id_override_.empty() ? "<input frame>" : frame_id_override_.c_str());
  }

private:
  std::string frame_id_override_;
  int min_points_{};
  double max_scan_duration_s_{};
  size_t dropped_scans_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr sanitized_publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

}  // namespace agt_mid360_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_mid360_adapter::Mid360AdapterNode>());
  rclcpp::shutdown();
  return 0;
}
