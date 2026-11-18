// Test fixture only: a named standalone costmap and a read-only freshness probe.
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

int main(int argc, char ** argv)
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const char * localhost = std::getenv("ROS_LOCALHOST_ONLY");
  if (domain == nullptr || std::string(domain) != "231" ||
    localhost == nullptr || std::string(localhost) != "1")
  {
    return 2;
  }
  if (argc < 2) {
    return 2;
  }
  const std::string name = argv[1];
  if (name != "local_costmap" && name != "global_costmap") {
    return 2;
  }

  rclcpp::init(argc, argv);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name);
  auto probe = std::make_shared<rclcpp::Node>("freshness_probe_" + name);
  auto publisher = probe->create_publisher<std_msgs::msg::Bool>(
    "/" + name + "/inputs_current", 10);
  auto timer = probe->create_wall_timer(
    std::chrono::milliseconds(100),
    [costmap, publisher]() {
      std_msgs::msg::Bool message;
      message.data =
        costmap->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        costmap->isCurrent();
      publisher->publish(message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(costmap->get_node_base_interface());
  executor.add_node(probe);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
