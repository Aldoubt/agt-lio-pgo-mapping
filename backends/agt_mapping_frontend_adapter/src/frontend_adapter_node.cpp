#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "agt_mapping_frontend_adapter/frontend_normalizer.hpp"
#include "agt_mapping_frontend_api/frontend_topics.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace agt_mapping_frontend_adapter
{
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

class FrontendAdapterNode : public rclcpp::Node
{
public:
  FrontendAdapterNode()
  : Node("mapping_frontend_adapter_node")
  {
    backend_id_ = declare_parameter<std::string>("backend_id", "unknown");
    source_commit_ = declare_parameter<std::string>("source_commit", "unknown");
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "");
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "");
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "");
    input_lidar_topic_ = declare_parameter<std::string>("input_lidar_topic", "/agt/sensors/lidar/custom");
    input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/agt/sensors/imu/data");
    cloud_frame_mode_ = declare_parameter<std::string>("cloud_frame_mode", "body");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    max_sync_slop_s_ = declare_parameter<double>("max_sync_slop_s", 0.05);
    body_from_lidar_.x = declare_parameter<double>("T_body_lidar.x", 0.011);
    body_from_lidar_.y = declare_parameter<double>("T_body_lidar.y", 0.02329);
    body_from_lidar_.z = declare_parameter<double>("T_body_lidar.z", -0.04412);
    body_from_lidar_.qx = declare_parameter<double>("T_body_lidar.qx", 0.0);
    body_from_lidar_.qy = declare_parameter<double>("T_body_lidar.qy", 0.0);
    body_from_lidar_.qz = declare_parameter<double>("T_body_lidar.qz", 0.0);
    body_from_lidar_.qw = declare_parameter<double>("T_body_lidar.qw", 1.0);
    if (input_odom_topic_.empty() || input_cloud_topic_.empty()) {
      throw std::invalid_argument("native odometry and cloud topics are required");
    }
    if (max_sync_slop_s_ <= 0.0 || max_sync_slop_s_ > 0.25) {
      throw std::invalid_argument("max_sync_slop_s must be in (0, 0.25]");
    }

    const auto topics = agt_mapping_frontend_api::makeFrontendTopics(
      agt_mapping_frontend_api::kDefaultNamespace);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics.odometry, rclcpp::QoS(20));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topics.cloud, rclcpp::QoS(20));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(topics.path, rclcpp::QoS(10));
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topics.status, rclcpp::QoS(10));

    lidar_counter_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_lidar_topic_, rclcpp::SensorDataQoS(),
      [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg) {
        ++ lidar_count_;
        last_lidar_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
      });
    imu_counter_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::ConstSharedPtr & msg) {
        ++ imu_count_;
        last_imu_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
      });
    if (!input_path_topic_.empty()) {
      path_sub_ = create_subscription<nav_msgs::msg::Path>(
        input_path_topic_, rclcpp::QoS(10), [this](const nav_msgs::msg::Path::ConstSharedPtr & msg) {
          try {
            path_pub_->publish(normalizePath(*msg, cloud_frame_mode_, body_frame_, body_from_lidar_));
          } catch (const std::exception & error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "path normalization failed: %s", error.what());
          }
        });
    } else {
      synthesize_path_ = true;
      canonical_path_.header.frame_id = "";
    }

    // Native estimators may publish odometry as best-effort or reliable.
    // A best-effort reader is compatible with both offered reliabilities.
    odom_sub_.subscribe(this, input_odom_topic_, rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this, input_cloud_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(40), odom_sub_, cloud_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_slop_s_));
    sync_->registerCallback(std::bind(
      &FrontendAdapterNode::onPair, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishDiagnostics();});
    diagnostics_clock_ = std::chrono::steady_clock::now();
  }

private:
  using Clock = std::chrono::steady_clock;

  void onPair(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    const auto started = Clock::now();
    try {
      const auto normalized_odom = normalizeOdometry(
        *odometry, cloud_frame_mode_, body_frame_, body_from_lidar_);
      const auto normalized_cloud = normalizeCloud(
        *cloud, *odometry, cloud_frame_mode_, body_frame_, lidar_frame_, body_from_lidar_);
      odom_pub_->publish(normalized_odom);
      cloud_pub_->publish(normalized_cloud);
      if (synthesize_path_) {
        if (canonical_path_.header.frame_id.empty()) {
          canonical_path_.header.frame_id = normalized_odom.header.frame_id;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = normalized_odom.header;
        pose.pose = normalized_odom.pose.pose;
        canonical_path_.poses.push_back(std::move(pose));
        if (canonical_path_.poses.size() > 20000) {
          canonical_path_.poses.erase(canonical_path_.poses.begin());
        }
        canonical_path_.header.stamp = normalized_odom.header.stamp;
        if (output_pair_count_ % 10 == 0) path_pub_->publish(canonical_path_);
      }
      ++ output_odom_count_;
      ++ output_pair_count_;
      last_frontend_cloud_stamp_ = rclcpp::Time(cloud->header.stamp).seconds();
      last_odom_stamp_ = rclcpp::Time(odometry->header.stamp).seconds();
      const auto finished = Clock::now();
      adapter_latency_ms_.push_back(std::chrono::duration<double, std::milli>(finished - started).count());
      if (adapter_latency_ms_.size() > 3000) adapter_latency_ms_.pop_front();
      const double sim_now = now().seconds();
      const double input_stamp = std::max(last_frontend_cloud_stamp_, last_odom_stamp_);
      if (sim_now > 0.0 && input_stamp > 0.0) {
        frontend_latency_ms_.push_back(std::max(0.0, (sim_now - input_stamp) * 1000.0));
        if (frontend_latency_ms_.size() > 3000) frontend_latency_ms_.pop_front();
      }
      running_ = true;
    } catch (const std::exception & error) {
      ++ transform_errors_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "frontend normalization failed: %s", error.what());
    }
  }

  static double percentile(std::deque<double> values, double fraction)
  {
    if (values.empty()) return -1.0;
    std::sort(values.begin(), values.end());
    const auto index = static_cast<size_t>(std::round(fraction * (values.size() - 1)));
    return values.at(index);
  }

  static std::string metric(double value)
  {
    return value < 0.0 ? "UNKNOWN" : std::to_string(value);
  }

  void publishDiagnostics()
  {
    const auto current = Clock::now();
    const double elapsed = std::chrono::duration<double>(current - diagnostics_clock_).count();
    const double lidar_hz = elapsed > 0.0 ? static_cast<double>(lidar_count_ - last_lidar_count_) / elapsed : 0.0;
    const double imu_hz = elapsed > 0.0 ? static_cast<double>(imu_count_ - last_imu_count_) / elapsed : 0.0;
    const double odom_hz = elapsed > 0.0 ? static_cast<double>(output_odom_count_ - last_output_odom_count_) / elapsed : 0.0;
    last_lidar_count_ = lidar_count_;
    last_imu_count_ = imu_count_;
    last_output_odom_count_ = output_odom_count_;
    diagnostics_clock_ = current;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "mapping/frontend";
    status.hardware_id = backend_id_;
    status.level = running_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = running_ ? "frontend relay active" : "waiting for synchronized frontend data";
    append(status, "backend_id", backend_id_);
    append(status, "source_commit", source_commit_);
    append(status, "running", running_ ? "true" : "false");
    append(status, "input_lidar_hz", std::to_string(lidar_hz));
    append(status, "input_imu_hz", std::to_string(imu_hz));
    append(status, "output_odom_hz", std::to_string(odom_hz));
    append(status, "processing_latency", "UNKNOWN: native estimator does not publish per-scan processing time");
    append(status, "frontend_output_latency_p50_ms", metric(percentile(frontend_latency_ms_, 0.50)));
    append(status, "frontend_output_latency_p95_ms", metric(percentile(frontend_latency_ms_, 0.95)));
    append(status, "frontend_output_latency_max_ms", metric(percentile(frontend_latency_ms_, 1.00)));
    append(status, "adapter_processing_latency_p50_ms", metric(percentile(adapter_latency_ms_, 0.50)));
    append(status, "adapter_processing_latency_p95_ms", metric(percentile(adapter_latency_ms_, 0.95)));
    append(status, "adapter_processing_latency_max_ms", metric(percentile(adapter_latency_ms_, 1.00)));
    append(status, "dropped_lidar", "UNKNOWN: native estimator does not expose accepted/drop counters");
    append(status, "dropped_imu", "UNKNOWN: native estimator does not expose accepted/drop counters");
    append(status, "transform_errors", std::to_string(transform_errors_));
    append(status, "paired_cloud_odom_count", std::to_string(output_pair_count_));
    append(status, "max_sync_slop_s", std::to_string(max_sync_slop_s_));
    append(status, "last_lidar_stamp", metric(last_lidar_stamp_));
    append(status, "last_imu_stamp", metric(last_imu_stamp_));
    append(status, "last_frontend_cloud_stamp", metric(last_frontend_cloud_stamp_));
    append(status, "last_odom_stamp", metric(last_odom_stamp_));
    array.status.push_back(std::move(status));
    status_pub_->publish(array);
  }

  static void append(diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }

  std::string backend_id_, source_commit_;
  std::string input_odom_topic_, input_cloud_topic_, input_path_topic_;
  std::string input_lidar_topic_, input_imu_topic_;
  std::string cloud_frame_mode_, body_frame_, lidar_frame_;
  double max_sync_slop_s_{0.05};
  RigidTransform body_from_lidar_;
  bool running_{false};
  bool synthesize_path_{false};
  uint64_t lidar_count_{0}, imu_count_{0}, output_odom_count_{0}, output_pair_count_{0};
  uint64_t transform_errors_{0}, last_lidar_count_{0}, last_imu_count_{0}, last_output_odom_count_{0};
  double last_lidar_stamp_{-1.0}, last_imu_stamp_{-1.0}, last_frontend_cloud_stamp_{-1.0};
  double last_odom_stamp_{-1.0};
  Clock::time_point diagnostics_clock_;
  std::deque<double> adapter_latency_ms_, frontend_latency_ms_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_counter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_counter_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path canonical_path_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace agt_mapping_frontend_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_mapping_frontend_adapter::FrontendAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
