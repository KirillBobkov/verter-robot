cmake_minimum_required(VERSION 3.8)
project(verter_isolated_costmap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(verter_test_costmap main.cpp)
ament_target_dependencies(verter_test_costmap
  rclcpp nav2_costmap_2d lifecycle_msgs std_msgs)
