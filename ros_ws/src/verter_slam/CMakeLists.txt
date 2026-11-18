cmake_minimum_required(VERSION 3.16)
project(verter_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
# The selected upstream sources share the Karto ABI and ROS interfaces of this version.
find_package(slam_toolbox 2.6.10 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)

set(ros_dependencies
  rclcpp message_filters nav_msgs sensor_msgs tf2 tf2_ros visualization_msgs
  pluginlib tf2_geometry_msgs tf2_sensor_msgs std_msgs std_srvs
  builtin_interfaces interactive_markers rclcpp_components
)
foreach(dependency ${ros_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

get_filename_component(slam_prefix "${slam_toolbox_DIR}/../../.." ABSOLUTE)
find_library(karto_library NAMES kartoSlamToolbox
  PATHS "${slam_prefix}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(slam_typesupport NAMES slam_toolbox__rosidl_typesupport_cpp
  PATHS "${slam_prefix}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(tbb_library NAMES tbb REQUIRED)

set(upstream "${CMAKE_CURRENT_SOURCE_DIR}/third_party/slam_toolbox")
add_library(verter_slam_common SHARED
  ${upstream}/src/slam_toolbox_common.cpp
  ${upstream}/src/map_saver.cpp
  ${upstream}/src/loop_closure_assistant.cpp
  ${upstream}/src/laser_utils.cpp
  ${upstream}/src/slam_mapper.cpp
)
target_include_directories(verter_slam_common BEFORE PUBLIC
  ${upstream}/include ${slam_toolbox_INCLUDE_DIRS})
# Do not link slam_toolbox_LIBRARIES: that would also load the unpatched common library.
ament_target_dependencies(verter_slam_common ${ros_dependencies})
target_link_libraries(verter_slam_common
  ${karto_library} ${slam_typesupport} ${tbb_library} Eigen3::Eigen
  Boost::system Boost::serialization Boost::filesystem Boost::thread)

add_executable(async_slam_toolbox_node
  ${upstream}/src/slam_toolbox_async_node.cpp
  ${upstream}/src/slam_toolbox_async.cpp)
ament_target_dependencies(async_slam_toolbox_node ${ros_dependencies})
target_link_libraries(async_slam_toolbox_node verter_slam_common)
# Also works for a running supervisor whose environment predates this package.
set_target_properties(async_slam_toolbox_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS verter_slam_common LIBRARY DESTINATION lib)
install(TARGETS async_slam_toolbox_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/slam_toolbox/LICENSE third_party/slam_toolbox/ORIGIN.json
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY patches DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_motion_gate test/test_motion_gate.cpp)
  target_include_directories(test_motion_gate PRIVATE ${upstream}/include)
  enable_testing()
  add_test(NAME motion_gate COMMAND test_motion_gate)
endif()

ament_package()
