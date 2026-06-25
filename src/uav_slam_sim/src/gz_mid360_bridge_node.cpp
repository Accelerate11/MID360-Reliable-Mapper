#include <algorithm>
#include <cstdint>
#include <string>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace
{
builtin_interfaces::msg::Time toRosTime(const gz::msgs::Time &time)
{
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(time.sec());
  ros_time.nanosec = static_cast<uint32_t>(std::max<int32_t>(0, time.nsec()));
  return ros_time;
}

builtin_interfaces::msg::Time stampFromHeader(
    const gz::msgs::Header &header,
    const rclcpp::Clock::SharedPtr &clock)
{
  if (header.has_stamp()) {
    return toRosTime(header.stamp());
  }
  return clock->now();
}

uint8_t toRosPointFieldType(gz::msgs::PointCloudPacked_Field_DataType type)
{
  using GzType = gz::msgs::PointCloudPacked_Field_DataType;
  switch (type) {
    case GzType::PointCloudPacked_Field_DataType_INT8:
      return sensor_msgs::msg::PointField::INT8;
    case GzType::PointCloudPacked_Field_DataType_UINT8:
      return sensor_msgs::msg::PointField::UINT8;
    case GzType::PointCloudPacked_Field_DataType_INT16:
      return sensor_msgs::msg::PointField::INT16;
    case GzType::PointCloudPacked_Field_DataType_UINT16:
      return sensor_msgs::msg::PointField::UINT16;
    case GzType::PointCloudPacked_Field_DataType_INT32:
      return sensor_msgs::msg::PointField::INT32;
    case GzType::PointCloudPacked_Field_DataType_UINT32:
      return sensor_msgs::msg::PointField::UINT32;
    case GzType::PointCloudPacked_Field_DataType_FLOAT32:
      return sensor_msgs::msg::PointField::FLOAT32;
    case GzType::PointCloudPacked_Field_DataType_FLOAT64:
      return sensor_msgs::msg::PointField::FLOAT64;
    default:
      return sensor_msgs::msg::PointField::UINT8;
  }
}
}  // namespace

class GzMid360BridgeNode : public rclcpp::Node
{
public:
  GzMid360BridgeNode()
  : Node("gz_mid360_bridge_node")
  {
    const auto mid360_gz_topic =
      declare_parameter<std::string>("mid360_gz_topic", "/mid360/points");
    const auto mid360_ros_topic =
      declare_parameter<std::string>("mid360_ros_topic", "/mid360/points");
    const auto imu_gz_topic =
      declare_parameter<std::string>("imu_gz_topic", "/mid360/imu");
    const auto imu_ros_topic =
      declare_parameter<std::string>("imu_ros_topic", "/mid360/imu");
    const auto clock_gz_topic =
      declare_parameter<std::string>("clock_gz_topic", "/clock");
    lidar_frame_id_ =
      declare_parameter<std::string>("lidar_frame_id", "mid360_link");
    imu_frame_id_ =
      declare_parameter<std::string>("imu_frame_id", "imu_link");

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      mid360_ros_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_ros_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    if (!gz_node_.Subscribe(mid360_gz_topic, &GzMid360BridgeNode::onPointCloud, this)) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo topic %s", mid360_gz_topic.c_str());
    }
    if (!gz_node_.Subscribe(imu_gz_topic, &GzMid360BridgeNode::onImu, this)) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo topic %s", imu_gz_topic.c_str());
    }
    if (!gz_node_.Subscribe(clock_gz_topic, &GzMid360BridgeNode::onClock, this)) {
      RCLCPP_WARN(get_logger(), "Failed to subscribe to Gazebo topic %s", clock_gz_topic.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s and %s -> %s",
      mid360_gz_topic.c_str(),
      mid360_ros_topic.c_str(),
      imu_gz_topic.c_str(),
      imu_ros_topic.c_str());
  }

private:
  void onPointCloud(const gz::msgs::PointCloudPacked &gz_msg)
  {
    sensor_msgs::msg::PointCloud2 ros_msg;
    if (gz_msg.has_header()) {
      ros_msg.header.stamp = stampFromHeader(gz_msg.header(), get_clock());
    } else {
      ros_msg.header.stamp = now();
    }
    ros_msg.header.frame_id = lidar_frame_id_;
    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    ros_msg.is_bigendian = gz_msg.is_bigendian();
    ros_msg.point_step = gz_msg.point_step();
    ros_msg.row_step = gz_msg.row_step();
    ros_msg.is_dense = gz_msg.is_dense();

    ros_msg.fields.reserve(static_cast<size_t>(gz_msg.field_size()));
    for (int i = 0; i < gz_msg.field_size(); ++i) {
      const auto &gz_field = gz_msg.field(i);
      sensor_msgs::msg::PointField ros_field;
      ros_field.name = gz_field.name();
      ros_field.offset = gz_field.offset();
      ros_field.datatype = toRosPointFieldType(gz_field.datatype());
      ros_field.count = gz_field.count();
      ros_msg.fields.push_back(std::move(ros_field));
    }

    const auto &data = gz_msg.data();
    ros_msg.data.resize(data.size());
    std::copy(data.begin(), data.end(), ros_msg.data.begin());
    pointcloud_pub_->publish(std::move(ros_msg));
  }

  void onImu(const gz::msgs::IMU &gz_msg)
  {
    sensor_msgs::msg::Imu ros_msg;
    if (gz_msg.has_header()) {
      ros_msg.header.stamp = stampFromHeader(gz_msg.header(), get_clock());
    } else {
      ros_msg.header.stamp = now();
    }
    ros_msg.header.frame_id = imu_frame_id_;

    if (gz_msg.has_orientation()) {
      ros_msg.orientation.x = gz_msg.orientation().x();
      ros_msg.orientation.y = gz_msg.orientation().y();
      ros_msg.orientation.z = gz_msg.orientation().z();
      ros_msg.orientation.w = gz_msg.orientation().w();
    } else {
      ros_msg.orientation.w = 1.0;
      ros_msg.orientation_covariance[0] = -1.0;
    }

    if (gz_msg.has_angular_velocity()) {
      ros_msg.angular_velocity.x = gz_msg.angular_velocity().x();
      ros_msg.angular_velocity.y = gz_msg.angular_velocity().y();
      ros_msg.angular_velocity.z = gz_msg.angular_velocity().z();
    }

    if (gz_msg.has_linear_acceleration()) {
      ros_msg.linear_acceleration.x = gz_msg.linear_acceleration().x();
      ros_msg.linear_acceleration.y = gz_msg.linear_acceleration().y();
      ros_msg.linear_acceleration.z = gz_msg.linear_acceleration().z();
    }

    imu_pub_->publish(std::move(ros_msg));
  }

  void onClock(const gz::msgs::Clock &gz_msg)
  {
    if (!gz_msg.has_sim()) {
      return;
    }
    rosgraph_msgs::msg::Clock ros_msg;
    ros_msg.clock = toRosTime(gz_msg.sim());
    clock_pub_->publish(std::move(ros_msg));
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::string lidar_frame_id_;
  std::string imu_frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzMid360BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
