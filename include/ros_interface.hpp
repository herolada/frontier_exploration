#pragma once

#include "wfd_explorer/logger.hpp"
#include "wfd_explorer/wfd_types.hpp"
#include "wfd_explorer/wfd_processor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace wfd_explorer
{

struct ExplorerParams
{
  // Topics / frames
  std::string map_topic{"/map"};
  std::string robot_frame{"base_link"};
  std::string nav2_action{"navigate_to_pose"};

  // Timing
  double loop_rate_hz{1.0};          // main exploration loop frequency
  double map_timeout_s{5.0};         // how long to wait for a fresh map
  double nav_goal_timeout_s{60.0};   // timeout for a single nav2 goal
  double tf_timeout_s{1.0};

  // Visualisation
  bool publish_markers{true};
  std::string marker_topic{"~/frontiers"};

  // WFD params forwarded
  wfd::WFDParams wfd;
};

/**
 * @brief ROS2 interface for the WFD explorer.
 *
 * - Subscribes to the occupancy grid (decoupled via mutex-protected shared ptr).
 * - Runs the exploration while-loop in a separate thread so the ROS executor
 *   stays responsive to callbacks.
 * - Calls the Nav2 NavigateToPose action and waits for completion.
 */
class ROSInterface
{
public:
  using NavigateToPose     = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav      = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ROSInterface(rclcpp::Node::SharedPtr node);
  ~ROSInterface();

  /** Start the exploration loop thread. */
  void start();

  /** Signal the exploration loop to stop and join the thread. */
  void stop();

private:
  // -----------------------------------------------------------------------
  // ROS callbacks
  // -----------------------------------------------------------------------

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // -----------------------------------------------------------------------
  // Exploration loop (runs in its own thread)
  // -----------------------------------------------------------------------

  void explorationLoop();

  // -----------------------------------------------------------------------
  // Nav2 helpers
  // -----------------------------------------------------------------------

  /**
   * @brief Send a NavigateToPose goal and block until completion or timeout.
   * @return true if goal succeeded, false otherwise.
   */
  bool navigateTo(const wfd::Point2D & goal, const std::string & map_frame);

  // -----------------------------------------------------------------------
  // TF helpers
  // -----------------------------------------------------------------------

  /** Get robot position in the map frame. Returns nullopt on failure. */
  std::optional<wfd::Point2D> getRobotPosition(const std::string & map_frame);

  // -----------------------------------------------------------------------
  // Visualisation
  // -----------------------------------------------------------------------

  void publishFrontierMarkers(
    const std::vector<wfd::Frontier> & frontiers,
    const std::optional<wfd::Frontier> & best,
    const std::string & frame_id);

  // -----------------------------------------------------------------------
  // Parameter loading
  // -----------------------------------------------------------------------

  ExplorerParams loadParams();

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------

  rclcpp::Node::SharedPtr node_;
  ROSLogger logger_;
  ExplorerParams params_;

  // Map subscription
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::mutex map_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Nav2 action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Frontier visualisation publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Core logic
  std::unique_ptr<wfd::WFDProcessor> wfd_processor_;

  // Exploration thread
  std::thread exploration_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace wfd_explorer
