#include "ros_interface.hpp"
#include "frontier_exploration/srv/set_pose.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <future>
#include <stdexcept>

using namespace std::chrono_literals;

namespace frontier_exploration
{

// ============================================================
//  Constructor
// ============================================================
ROSInterface::ROSInterface(rclcpp::Node::SharedPtr node)
: node_(node),
  logger_(node->get_node_logging_interface())
{
  params_ = loadParams();

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, params_.nav2_action);

  // Map subscription (QoS: keep last 1, transient local to get the latest map)
  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.map_topic, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){ mapCallback(msg); });

  // Marker publisher
  if (params_.publish_markers) {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      params_.marker_topic, 10);
  }

  if (params_.publish_best_frontier) {
    best_frontier_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      params_.best_frontier_topic, 10);
  }

  // Exploration center publisher
  exploration_center_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/exploration_center", rclcpp::QoS(1).transient_local());

  // Service server for setting point around which to explore
  set_exploration_center_server_ = node_->create_service<frontier_exploration::srv::SetPose>(
    "~/set_exploration_center",
    std::bind(&ROSInterface::setExplorationCenterCallback, this,
                std::placeholders::_1, std::placeholders::_2));


  // WFD processor
  wfd_processor_ = std::make_unique<wfd::WFDProcessor>(params_.wfd, logger_);

  logger_.info("WFD Explorer initialised. Listening on '{}'", params_.map_topic);
}

// ============================================================
//  Destructor
// ============================================================
ROSInterface::~ROSInterface()
{
  stop();
}

// ============================================================
//  start / stop
// ============================================================
void ROSInterface::start()
{
  if (running_) return;
  running_ = true;
  exploration_thread_ = std::thread(&ROSInterface::explorationLoop, this);
  logger_.info("Exploration loop started");
}

void ROSInterface::stop()
{
  running_ = false;
  if (exploration_thread_.joinable()) {
    exploration_thread_.join();
  }
}

// ============================================================
//  mapCallback  (runs in executor thread)
// ============================================================
void ROSInterface::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_ = msg;
}

void ROSInterface::setExplorationCenterCallback(
  const std::shared_ptr<frontier_exploration::srv::SetPose::Request> req,
  std::shared_ptr<frontier_exploration::srv::SetPose::Response>)
{
  if (req->pose.header.frame_id == "") {
    logger_.error("Cannot set exploration center to a pose without frame_id.");
    return;
  }
  logger_.info("Setting exploration center to ({}, {}).",req->pose.pose.position.x,req->pose.pose.position.y);
  exploration_center_ = req->pose;
  exploration_center_pub_->publish(req->pose);
}

// ============================================================
//  loadParams
// ============================================================
ExplorerParams ROSInterface::loadParams()
{
  ExplorerParams p;

  p.map_topic        = node_->declare_parameter("map_topic",        p.map_topic);
  p.robot_frame      = node_->declare_parameter("robot_frame",      p.robot_frame);
  p.nav2_action      = node_->declare_parameter("nav2_action",      p.nav2_action);
  p.loop_rate_hz     = node_->declare_parameter("loop_rate_hz",     p.loop_rate_hz);
  p.map_timeout_s    = node_->declare_parameter("map_timeout_s",    p.map_timeout_s);
  p.nav_goal_timeout_s = node_->declare_parameter("nav_goal_timeout_s", p.nav_goal_timeout_s);
  p.tf_timeout_s     = node_->declare_parameter("tf_timeout_s",     p.tf_timeout_s);
  p.publish_markers  = node_->declare_parameter("publish_markers",  p.publish_markers);
  p.marker_topic     = node_->declare_parameter("marker_topic",     p.marker_topic);
  p.send_action      = node_->declare_parameter("send_action",       p.send_action);
  p.publish_best_frontier = node_->declare_parameter("publish_best_frontier", p.publish_best_frontier);
  p.best_frontier_topic = node_->declare_parameter("best_frontier_topic", p.best_frontier_topic);

  p.wfd.free_threshold          = node_->declare_parameter("wfd.free_threshold",          p.wfd.free_threshold);
  p.wfd.occ_threshold           = node_->declare_parameter("wfd.occ_threshold",           p.wfd.occ_threshold);
  p.wfd.min_frontier_size       = node_->declare_parameter("wfd.min_frontier_size",       p.wfd.min_frontier_size);
  p.wfd.min_frontier_dist       = node_->declare_parameter("wfd.min_frontier_dist",       p.wfd.min_frontier_dist);
  p.wfd.sensor_range            = node_->declare_parameter("wfd.sensor_range",            p.wfd.sensor_range);
  p.wfd.kmeans_max_iter         = node_->declare_parameter("wfd.kmeans_max_iter",         p.wfd.kmeans_max_iter);
  p.wfd.weights                 = node_->declare_parameter("wfd.weights",                 p.wfd.weights);
  p.wfd.info_gain_exponent      = node_->declare_parameter("wfd.info_gain_exponent",      p.wfd.info_gain_exponent);

  return p;
}

// ============================================================
//  getRobotPosition
// ============================================================
std::optional<wfd::Pose2D> ROSInterface::getRobotPosition(const std::string & map_frame)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      map_frame, params_.robot_frame, tf2::TimePointZero,
      tf2::durationFromSec(params_.tf_timeout_s));
    wfd::Pose2D pos;
    pos.x = tf.transform.translation.x;
    pos.y = tf.transform.translation.y;

    double yaw = tf2::getYaw(tf.transform.rotation);
    pos.yaw = yaw;

    return pos;
  } catch (const tf2::TransformException & ex) {
    logger_.warn("TF lookup failed: {}", ex.what());
    return std::nullopt;
  }
}

// ============================================================
//  navigateTo
// ============================================================
// bool ROSInterface::navigateTo(const wfd::Pose2D & goal, const std::string & map_frame)
// {
//   if (!nav_client_->wait_for_action_server(2s)) {
//     logger_.error("Nav2 action server '{}' not available", params_.nav2_action);
//     return false;
//   }

//   NavigateToPose::Goal goal_msg;
//   goal_msg.pose.header.frame_id = map_frame;
//   goal_msg.pose.header.stamp    = node_->now();
//   goal_msg.pose.pose.position.x = goal.x;
//   goal_msg.pose.pose.position.y = goal.y;
//   goal_msg.pose.pose.position.z = 0.0;
//   goal_msg.pose.pose.orientation.x = 0.0;  
//   goal_msg.pose.pose.orientation.y = 0.0;  
//   goal_msg.pose.pose.orientation.z = 0.0;  
//   goal_msg.pose.pose.orientation.w = 1.0;  // yaw = 0, nav2 will handle orientation

//   logger_.info("Sending nav goal: ({:.2f}, {:.2f}) in '{}'", goal.x, goal.y, map_frame);

//   auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

//   // Feedback callback – log distance remaining periodically
//   send_goal_options.feedback_callback =
//     [this](GoalHandleNav::SharedPtr /*gh*/,
//            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
//     {
//       LOG_INFO_THROTTLE(logger_, 2000,
//         "Nav2 feedback: dist_remaining={:.2f} m",
//         feedback->distance_remaining);
//     };

//   std::promise<bool> result_promise;
//   std::future<bool>  result_future = result_promise.get_future();

//   send_goal_options.result_callback =
//     [this, &result_promise](const GoalHandleNav::WrappedResult & result)
//     {
//       switch (result.code) {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//           logger_.info("Nav2 goal SUCCEEDED");
//           result_promise.set_value(true);
//           break;
//         case rclcpp_action::ResultCode::ABORTED:
//           logger_.warn("Nav2 goal ABORTED");
//           result_promise.set_value(false);
//           break;
//         case rclcpp_action::ResultCode::CANCELED:
//           logger_.warn("Nav2 goal CANCELED");
//           result_promise.set_value(false);
//           break;
//         default:
//           logger_.error("Nav2 goal: unknown result code");
//           result_promise.set_value(false);
//           break;
//       }
//     };

//   auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);

//   // Wait for goal to be accepted
//   if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
//     logger_.error("Nav2: goal handle future timed out");
//     return false;
//   }
//   auto goal_handle = goal_handle_future.get();
//   if (!goal_handle) {
//     logger_.error("Nav2 rejected the goal");
//     return false;
//   }

//   // Wait for result with overall timeout
//   const auto timeout = std::chrono::duration<double>(params_.nav_goal_timeout_s);
//   if (result_future.wait_for(timeout) != std::future_status::ready) {
//     logger_.warn("Nav2 goal timed out after {:.0f} s, cancelling", params_.nav_goal_timeout_s);
//     nav_client_->async_cancel_goal(goal_handle);
//     return false;
//   }
//   return result_future.get();
// }

bool ROSInterface::navigateTo(const wfd::Pose2D & goal, const std::string & map_frame)
{
  logger_.info("NavigateTo called");  
  if (!nav_client_->wait_for_action_server(2s)) {
    logger_.error("Nav2 action server '{}' not available", params_.nav2_action);
    return false;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = map_frame;
  goal_msg.pose.header.stamp    = node_->now();
  goal_msg.pose.pose.position.x = goal.x;
  goal_msg.pose.pose.position.y = goal.y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.x = 0.0;  
  goal_msg.pose.pose.orientation.y = 0.0;  
  goal_msg.pose.pose.orientation.z = 0.0;  
  goal_msg.pose.pose.orientation.w = 1.0;  // yaw = 0, nav2 will handle orientation

  logger_.info("Sending nav goal: ({:.2f}, {:.2f}) in '{}'", goal.x, goal.y, map_frame);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // Feedback callback – log distance remaining periodically
  send_goal_options.feedback_callback =
    [this](GoalHandleNav::SharedPtr /*gh*/,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      LOG_INFO_THROTTLE(logger_, 2000,
        "Nav2 feedback: dist_remaining={:.2f} m",
        feedback->distance_remaining);
    };

  auto result_promise = std::make_shared<std::promise<bool>>();
  std::future<bool>  result_future = result_promise->get_future();

  send_goal_options.result_callback =
    [this, result_promise](const GoalHandleNav::WrappedResult & result)
    {
      try {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            logger_.info("Nav2 goal SUCCEEDED");
            result_promise->set_value(true);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            logger_.warn("Nav2 goal ABORTED");
            result_promise->set_value(false);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            logger_.warn("Nav2 goal CANCELED");
            result_promise->set_value(false);
            break;
          default:
            logger_.error("Nav2 goal: unknown result code");
            result_promise->set_value(false);
            break;
        }
      } catch (const std::future_error &) {
        // Nav2 can invoke the result callback more than once (e.g. CANCELED
        // followed by ABORTED).  The promise is already satisfied; ignore.
      }
    };
  logger_.error("A");
  auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal to be accepted
  logger_.error("B");
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    logger_.error("Nav2: goal handle future timed out");
    return false;
  }
  logger_.error("C");
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    logger_.error("Nav2 rejected the goal");
    return false;
  }
  logger_.error("D");

  // Wait for result with overall timeout
  const auto timeout = std::chrono::duration<double>(params_.nav_goal_timeout_s);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    logger_.warn("Nav2 goal timed out after {:.0f} s, cancelling", params_.nav_goal_timeout_s);
    nav_client_->async_cancel_goal(goal_handle);
    return false;
  }
  logger_.error("E");
  return result_future.get();
}


// ============================================================
//  publishFrontierMarkers
// ============================================================
void ROSInterface::publishFrontierMarkers(
  const std::vector<wfd::Frontier> & frontiers,
  const std::optional<wfd::Frontier> & best,
  const std::string & frame_id)
{
  if (!marker_pub_) return;

  visualization_msgs::msg::MarkerArray ma;

  // Delete all previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header.frame_id = frame_id;
  del.header.stamp    = node_->now();
  ma.markers.push_back(del);

  // Compute score range for marker scaling
  wfd::Frontier f{};
  double default_score = f.score;
  std::vector<int> unused_frontiers(frontiers.size(), 0); 
  double min_score = std::numeric_limits<double>::max();
  double max_score = std::numeric_limits<double>::lowest();
  for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
    const auto & f = frontiers[fi];
    if (f.score == default_score) {
      unused_frontiers[fi] = 1;
      continue;
    }
    min_score = std::min(min_score, f.score);
    max_score = std::max(max_score, f.score);
  }
  const double score_range = (max_score > min_score) ? (max_score - min_score) : 1.0;

  int id = 0;
  for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
    const auto & f = frontiers[fi];
    bool is_best = best && (f.centroid.x == best->centroid.x &&
                            f.centroid.y == best->centroid.y);

    // Sphere at centroid
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp    = node_->now();
    m.ns              = "frontier_centroids";
    m.id              = id++;
    m.type            = visualization_msgs::msg::Marker::SPHERE;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = f.centroid.x;
    m.pose.position.y = f.centroid.y;
    m.pose.position.z = 0.05;
    m.pose.orientation.w = 1.0;
    const double t = (f.score - min_score) / score_range;
    const double marker_size = std::max(0.2, 0.3 + t * (0.8 - 0.3));
    m.scale.x = m.scale.y = m.scale.z = static_cast<float>(marker_size);
    m.color.a = unused_frontiers[fi] ? 0.5 : 1.0;
    m.color.r = is_best ? 1.0f : 0.0f;
    m.color.g = is_best ? 0.5f : 0.8f;
    m.color.b = 0.0f;
    ma.markers.push_back(m);

    // Points showing frontier cells (downsampled to avoid flooding)
    if (!f.cells.empty()) {
      visualization_msgs::msg::Marker pts;
      pts.header    = m.header;
      pts.ns        = "frontier_cells";
      pts.id        = id++;
      pts.type      = visualization_msgs::msg::Marker::POINTS;
      pts.action    = visualization_msgs::msg::Marker::ADD;
      pts.scale.x   = pts.scale.y = 0.1;
      pts.color.a   = 0.5;
      pts.color.r   = 0.2f;
      pts.color.g   = 0.6f;
      pts.color.b   = 1.0f;
      for (std::size_t ci = 0; ci < f.cells.size(); ci += 3) {  // every 3rd cell
        geometry_msgs::msg::Point p;
        p.x = f.cells[ci].x;
        p.y = f.cells[ci].y;
        p.z = 0.02;
        pts.points.push_back(p);
      }
      ma.markers.push_back(pts);
    }
  }

  marker_pub_->publish(ma);
}

// ============================================================
//  explorationLoop  (runs in its own thread)
// ============================================================
void ROSInterface::explorationLoop()
{
  const rclcpp::Duration map_timeout(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(params_.map_timeout_s)));

  const double period_s = 1.0 / std::max(params_.loop_rate_hz, 0.01);

  while (running_ && rclcpp::ok()) {
    auto loop_start = std::chrono::steady_clock::now();

    // ------------------------------------------------------------------
    // 1. Get a fresh copy of the latest map
    // ------------------------------------------------------------------
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_msg = latest_map_;
    }

    if (!map_msg) {
      LOG_INFO_THROTTLE(logger_, 2000, "Waiting for occupancy grid on '{}'...", params_.map_topic);
      std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
      continue;
    }

    const std::string map_frame = map_msg->header.frame_id;

    // Transform the exploration center pose to the map frame.
    geometry_msgs::msg::PoseStamped exploration_center_map;
    std::optional<wfd::Pose2D> center_pose;

    
    if (exploration_center_.has_value()) {
      try {
        auto tf = tf_buffer_->lookupTransform(exploration_center_.value().header.frame_id, map_frame,
          tf2::TimePointZero,
          tf2::durationFromSec(params_.tf_timeout_s));
        tf2::doTransform(exploration_center_.value(), exploration_center_map, tf);

        wfd::Pose2D exploration_center_map_pose;
        exploration_center_map_pose.x = exploration_center_map.pose.position.x;
        exploration_center_map_pose.y = exploration_center_map.pose.position.y;
        exploration_center_map_pose.yaw = 0.;
        center_pose = exploration_center_map_pose;

        logger_.info("Exploration center: (x {:.2f}, y {:.2f}, yaw {:.2f}) in '{}'",
          center_pose->x, center_pose->y, center_pose->yaw, map_frame);
      } catch (const tf2::TransformException & ex) {
        logger_.warn("TF lookup failed: {}", ex.what());
      }
    }

    // ------------------------------------------------------------------
    // 2. Build thresholded grid
    // ------------------------------------------------------------------
    const auto & info = map_msg->info;
    wfd::OccupancyGrid grid = wfd_processor_->buildGrid(
      map_msg->data,
      static_cast<int>(info.width),
      static_cast<int>(info.height),
      info.resolution,
      info.origin.position.x,
      info.origin.position.y,
      map_frame);

    // ------------------------------------------------------------------
    // 3. Get robot position
    // ------------------------------------------------------------------
    auto robot_pos_opt = getRobotPosition(map_frame);
    if (!robot_pos_opt) {
      logger_.warn("Could not get robot position, skipping iteration");
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
      continue;
    }
    wfd::Pose2D robot_pos = *robot_pos_opt;
    
    logger_.info("Robot position: (x {:.2f}, y {:.2f}, yaw {:.2f}) in '{}'",
      robot_pos.x, robot_pos.y, robot_pos.yaw, map_frame);

    // ------------------------------------------------------------------
    // 4. Run WFD
    // ------------------------------------------------------------------
    auto frontiers = wfd_processor_->detect(grid, robot_pos);

    if (frontiers.empty()) {
      logger_.info("No frontiers detected – exploration may be complete!");
      std::this_thread::sleep_for(std::chrono::duration<double>(2.0));
      continue;
    }

    // ------------------------------------------------------------------
    // 5. Select best frontier
    // ------------------------------------------------------------------
    std::optional<wfd::Frontier> best;
    best = wfd_processor_->selectBest(frontiers, robot_pos, center_pose);

    if (!best) {
      logger_.warn("Could not select best frontier");
      continue;
    }

    // ------------------------------------------------------------------
    // 6. Publish visualisation
    // ------------------------------------------------------------------
    publishFrontierMarkers(frontiers, best, map_frame);

    // ------------------------------------------------------------------
    // 7. Navigate to goal
    // ------------------------------------------------------------------
    if (params_.send_action) {
      bool success = navigateTo(best->centroid, map_frame);
      logger_.info("Navigation result: {}", success ? "SUCCESS" : "FAILED/ABORTED");  
    }

    if (params_.publish_best_frontier) {
      geometry_msgs::msg::PoseStamped best_frontier_pose;
      best_frontier_pose.header.stamp = node_->now();
      best_frontier_pose.header.frame_id = map_frame;
      best_frontier_pose.pose.position.x = best->centroid.x;
      best_frontier_pose.pose.position.y = best->centroid.y;
      best_frontier_pose.pose.position.z = 0.;
      best_frontier_pub_->publish(best_frontier_pose);
    }

    // ------------------------------------------------------------------
    // 8. Rate limiting (respects time already spent)
    // ------------------------------------------------------------------
    auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - loop_start).count();
    double sleep_s = period_s - elapsed;
    if (sleep_s > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
  }

  logger_.info("Exploration loop finished");
}

}  // namespace frontier_exploration
