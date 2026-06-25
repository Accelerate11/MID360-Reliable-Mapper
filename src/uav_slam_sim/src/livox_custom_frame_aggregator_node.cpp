#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
uint64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<uint64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time nsToStamp(uint64_t time_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(time_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000ULL);
  return stamp;
}
}  // namespace

class LivoxCustomFrameAggregator : public rclcpp::Node
{
public:
  LivoxCustomFrameAggregator()
  : Node("livox_custom_frame_aggregator")
  {
    const std::string input_topic =
      declare_parameter<std::string>("input_topic", "/livox/lidar");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/livox/lidar_frame");
    const std::string imu_input_topic =
      declare_parameter<std::string>("imu_input_topic", "/livox/imu");
    const std::string imu_output_topic =
      declare_parameter<std::string>("imu_output_topic", "/livox/imu_frame");
    const double frame_interval_ms =
      declare_parameter<double>("frame_interval_ms", 100.0);
    min_points_ = declare_parameter<int>("min_points", 100);
    restamp_lidar_ = declare_parameter<bool>("restamp_lidar", true);
    restamp_imu_ = declare_parameter<bool>("restamp_imu", true);
    filter_points_ = declare_parameter<bool>("filter_points", true);
    min_range_ = declare_parameter<double>("min_range", 0.45);
    max_range_ = declare_parameter<double>("max_range", 35.0);
    z_min_ = declare_parameter<double>("z_min", -3.0);
    z_max_ = declare_parameter<double>("z_max", 5.0);

    frame_interval_ns_ = static_cast<uint64_t>(std::max(1.0, frame_interval_ms) * 1000000.0);

    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic, rclcpp::QoS(20));
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic, rclcpp::QoS(50),
      std::bind(&LivoxCustomFrameAggregator::msgCallback, this, std::placeholders::_1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_topic, rclcpp::QoS(100));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic, rclcpp::QoS(100),
      std::bind(&LivoxCustomFrameAggregator::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Aggregating Livox CustomMsg from %s to %s at %.1f ms frames",
      input_topic.c_str(), output_topic.c_str(), frame_interval_ms);
    RCLCPP_INFO(
      get_logger(), "Republishing Livox IMU from %s to %s%s",
      imu_input_topic.c_str(), imu_output_topic.c_str(),
      restamp_imu_ ? " with ROS receive timestamps" : "");
    RCLCPP_INFO(
      get_logger(), "Livox raw filter: %s, range=[%.2f, %.2f], z=[%.2f, %.2f]",
      filter_points_ ? "on" : "off", min_range_, max_range_, z_min_, z_max_);
  }

private:
  bool acceptPoint(const livox_ros_driver2::msg::CustomPoint & point) const
  {
    if (!filter_points_) {
      return true;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }
    if (point.z < z_min_ || point.z > z_max_) {
      return false;
    }

    const double range_sq =
      static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y +
      static_cast<double>(point.z) * point.z;
    const double min_range_sq = min_range_ > 0.0 ? min_range_ * min_range_ : 0.0;
    const double max_range_sq = max_range_ > 0.0 ? max_range_ * max_range_ :
      std::numeric_limits<double>::infinity();
    return range_sq >= min_range_sq && range_sq <= max_range_sq;
  }

  uint64_t monotonicNowNs(uint64_t & last_stamp_ns)
  {
    uint64_t now_ns = static_cast<uint64_t>(get_clock()->now().nanoseconds());
    if (now_ns <= last_stamp_ns) {
      now_ns = last_stamp_ns + 1;
    }
    last_stamp_ns = now_ns;
    return now_ns;
  }

  uint64_t restampedLidarNs(uint64_t input_start_ns)
  {
    const uint64_t now_ns = static_cast<uint64_t>(get_clock()->now().nanoseconds());
    if (!lidar_time_offset_valid_) {
      lidar_input_to_output_offset_ns_ =
        static_cast<int64_t>(now_ns) - static_cast<int64_t>(input_start_ns);
      lidar_time_offset_valid_ = true;
    }

    int64_t out_ns_i =
      static_cast<int64_t>(input_start_ns) + lidar_input_to_output_offset_ns_;
    if (out_ns_i <= 0 ||
      std::llabs(out_ns_i - static_cast<int64_t>(now_ns)) > static_cast<int64_t>(2000000000LL))
    {
      lidar_input_to_output_offset_ns_ =
        static_cast<int64_t>(now_ns) - static_cast<int64_t>(input_start_ns);
      out_ns_i = static_cast<int64_t>(now_ns);
    }

    uint64_t out_ns = static_cast<uint64_t>(out_ns_i);
    if (out_ns <= last_lidar_pub_stamp_ns_) {
      out_ns = last_lidar_pub_stamp_ns_ + 1;
    }
    last_lidar_pub_stamp_ns_ = out_ns;
    return out_ns;
  }

  void resetFrame(uint64_t start_ns, const livox_ros_driver2::msg::CustomMsg & msg)
  {
    active_ = true;
    frame_start_ns_ = start_ns;
    output_frame_start_ns_ = restamp_lidar_ ? restampedLidarNs(frame_start_ns_) : frame_start_ns_;
    frame_msg_ = livox_ros_driver2::msg::CustomMsg();
    frame_msg_.header = msg.header;
    frame_msg_.header.stamp = nsToStamp(output_frame_start_ns_);
    frame_msg_.timebase = output_frame_start_ns_;
    frame_msg_.lidar_id = msg.lidar_id;
    frame_msg_.points.clear();
  }

  void publishFrame()
  {
    if (!active_ || static_cast<int>(frame_msg_.points.size()) < min_points_) {
      return;
    }

    frame_msg_.point_num = static_cast<uint32_t>(frame_msg_.points.size());
      pub_->publish(frame_msg_);
  }

  void appendPoint(const livox_ros_driver2::msg::CustomPoint & src_point, uint64_t point_abs_ns)
  {
    auto point = src_point;
    const uint64_t offset_ns = point_abs_ns > frame_start_ns_ ? point_abs_ns - frame_start_ns_ : 0;
    point.offset_time = static_cast<uint32_t>(
      std::min<uint64_t>(offset_ns, std::numeric_limits<uint32_t>::max()));
    frame_msg_.points.push_back(point);
  }

  void msgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const uint64_t msg_base_ns = msg->timebase != 0 ? msg->timebase : stampToNs(msg->header.stamp);
    frame_msg_.points.reserve(frame_msg_.points.size() + msg->points.size());

    for (const auto & src_point : msg->points) {
      if (!acceptPoint(src_point)) {
        continue;
      }

      const uint64_t point_abs_ns = msg_base_ns + static_cast<uint64_t>(src_point.offset_time);
      if (!active_) {
        resetFrame(point_abs_ns, *msg);
      }

      if (point_abs_ns < frame_start_ns_) {
        continue;
      }

      while (point_abs_ns - frame_start_ns_ >= frame_interval_ns_) {
        publishFrame();
        resetFrame(frame_start_ns_ + frame_interval_ns_, *msg);
      }

      appendPoint(src_point, point_abs_ns);
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out = *msg;
    if (restamp_imu_) {
      out.header.stamp = nsToStamp(monotonicNowNs(last_imu_pub_stamp_ns_));
    }
    imu_pub_->publish(out);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  livox_ros_driver2::msg::CustomMsg frame_msg_;
  uint64_t frame_start_ns_{0};
  uint64_t output_frame_start_ns_{0};
  uint64_t last_lidar_pub_stamp_ns_{0};
  uint64_t last_imu_pub_stamp_ns_{0};
  uint64_t frame_interval_ns_{100000000};
  int64_t lidar_input_to_output_offset_ns_{0};
  int min_points_{100};
  double min_range_{0.45};
  double max_range_{35.0};
  double z_min_{-3.0};
  double z_max_{5.0};
  bool active_{false};
  bool restamp_lidar_{true};
  bool restamp_imu_{true};
  bool filter_points_{true};
  bool lidar_time_offset_valid_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomFrameAggregator>());
  rclcpp::shutdown();
  return 0;
}
