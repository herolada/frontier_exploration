#include "wfd_explorer/ros_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "wfd_explorer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));

  auto explorer = std::make_shared<wfd_explorer::ROSInterface>(node);
  explorer->start();

  // Spin the ROS executor so that callbacks (map, tf, action feedback) are processed
  rclcpp::spin(node);

  explorer->stop();
  rclcpp::shutdown();
  return 0;
}
