#pragma once

#include "abstract_logger.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS 2 concrete logger.
 *
 * Wraps RCLCPP_* macros so that core-logic classes that hold an
 * AbstractLogger* work unchanged inside a ROS 2 node.
 *
 * Usage (inside your node):
 *   auto logger =
 * std::make_shared<ROSLogger>(this->get_node_logging_interface());
 */
class ROSLogger final : public AbstractLogger {
public:
  explicit ROSLogger(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
                         logging_interface)
      : logging_interface_(logging_interface) {}

protected:
  void log_impl(Level level, const std::string &message) override {
    switch (level) {
    case Level::Debug:
      RCLCPP_DEBUG(logging_interface_->get_logger(), "%s", message.c_str());
      break;
    case Level::Info:
      RCLCPP_INFO(logging_interface_->get_logger(), "%s", message.c_str());
      break;
    case Level::Warn:
      RCLCPP_WARN(logging_interface_->get_logger(), "%s", message.c_str());
      break;
    case Level::Error:
      RCLCPP_ERROR(logging_interface_->get_logger(), "%s", message.c_str());
      break;
    }
  }

private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
};