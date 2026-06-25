#include "ros_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "frontier_exploration",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));

  auto explorer = std::make_shared<frontier_exploration::ROSInterface>(node);
  explorer->start();

  // Multi-threaded executor so the explore_once service (its own callback group)
  // can run in parallel with the map/scan subscriptions and other callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  explorer->stop();
  rclcpp::shutdown();
  return 0;
}
