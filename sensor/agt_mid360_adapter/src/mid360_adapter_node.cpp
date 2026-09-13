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
    frame_id_override_ = declare_parameter<std::string>("frame_id", "");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
        publisher_->publish(customMsgToPointCloud2(*message, frame_id_override_));
      });
    RCLCPP_INFO(
      get_logger(), "MID360 adapter: %s -> %s; frame override: %s",
      input_topic.c_str(), output_topic.c_str(),
      frame_id_override_.empty() ? "<input frame>" : frame_id_override_.c_str());
  }

private:
  std::string frame_id_override_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
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
