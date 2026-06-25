#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud2StrideFilter : public rclcpp::Node
{
public:
  PointCloud2StrideFilter()
  : Node("pointcloud2_stride_filter")
  {
    const std::string input_topic =
      declare_parameter<std::string>("input_topic", "/Laser_map");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/Laser_map_vis");
    max_points_ = declare_parameter<int>("max_points", 120000);
    restamp_ = declare_parameter<bool>("restamp", true);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(2));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(2),
      std::bind(&PointCloud2StrideFilter::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Downsampling %s to %s with max_points=%d",
      input_topic.c_str(), output_topic.c_str(), max_points_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint64_t point_count = static_cast<uint64_t>(msg->width) * msg->height;
    if (point_count == 0 || msg->point_step == 0 || msg->data.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;
    if (restamp_) {
      out.header.stamp = now();
    }

    const uint64_t max_points = static_cast<uint64_t>(std::max(1, max_points_));
    if (point_count <= max_points) {
      pub_->publish(out);
      return;
    }

    const uint64_t stride = std::max<uint64_t>(1, point_count / max_points);
    const uint64_t point_step = msg->point_step;
    const uint64_t available_points = msg->data.size() / point_step;
    const uint64_t source_points = std::min(point_count, available_points);
    const uint64_t output_points = (source_points + stride - 1) / stride;

    out.height = 1;
    out.width = static_cast<uint32_t>(output_points);
    out.row_step = static_cast<uint32_t>(output_points * point_step);
    out.data.resize(output_points * point_step);
    out.is_dense = false;

    uint8_t * dst = out.data.data();
    uint64_t written = 0;
    for (uint64_t i = 0; i < source_points && written < output_points; i += stride) {
      std::memcpy(dst + written * point_step, msg->data.data() + i * point_step, point_step);
      ++written;
    }

    if (written != output_points) {
      out.width = static_cast<uint32_t>(written);
      out.row_step = static_cast<uint32_t>(written * point_step);
      out.data.resize(written * point_step);
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  int max_points_{120000};
  bool restamp_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2StrideFilter>());
  rclcpp::shutdown();
  return 0;
}
