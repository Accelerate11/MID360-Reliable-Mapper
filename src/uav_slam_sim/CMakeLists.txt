cmake_minimum_required(VERSION 3.8)
project(uav_slam_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(gz_mid360_bridge_node src/gz_mid360_bridge_node.cpp)
ament_target_dependencies(
  gz_mid360_bridge_node
  rclcpp
  rosgraph_msgs
  sensor_msgs
)
target_link_libraries(
  gz_mid360_bridge_node
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

add_executable(livox_custom_frame_aggregator_node src/livox_custom_frame_aggregator_node.cpp)
ament_target_dependencies(
  livox_custom_frame_aggregator_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(pointcloud2_stride_filter_node src/pointcloud2_stride_filter_node.cpp)
ament_target_dependencies(
  pointcloud2_stride_filter_node
  rclcpp
  sensor_msgs
)

add_executable(fastlio_cloud_mapper_node src/fastlio_cloud_mapper_node.cpp)
ament_target_dependencies(
  fastlio_cloud_mapper_node
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(pointcloud_occupancy_grid_node src/pointcloud_occupancy_grid_node.cpp)
ament_target_dependencies(
  pointcloud_occupancy_grid_node
  nav_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  gz_mid360_bridge_node
  livox_custom_frame_aggregator_node
  pointcloud2_stride_filter_node
  fastlio_cloud_mapper_node
  pointcloud_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  docs
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

