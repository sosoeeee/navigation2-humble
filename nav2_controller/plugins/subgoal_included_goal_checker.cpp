#include "nav2_controller/plugins/subgoal_included_goal_checker.hpp"

#include <string>
#include <memory>
#include <algorithm> // For std::min, std::max

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller
{

SubgoalIncludedGoalChecker::SubgoalIncludedGoalChecker()
: xy_goal_tolerance_(0.0), yaw_goal_tolerance_(0.0),
  stateful_(true), num_marked_subgoals_(0), xy_goal_tolerance_sq_(0.0),
  mark_subgoal_service_available_(false)
{
}

void SubgoalIncludedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) // costmap_ros is unused for now
{
  node_ = parent;
  plugin_name_ = plugin_name;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in SubgoalIncludedGoalChecker::initialize");
  }
  logger_ = node->get_logger(); // Use node's logger directly

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance",
    rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance",
    rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful",
    rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  num_marked_subgoals_ = 0; // Initialize counter for subgoals

  // Initialize TF Buffer and Listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  robot_frame_id_ = "base_link"; // Assuming "base_link" is the robot's frame
  map_frame_id_ = "map"; // Assuming "map" is the global frame

  // Create service client for marking subgoals
  mark_subgoal_client_ = node->create_client<gym_msgs::srv::MarkSubgoal>("mark_subgoal");
  mark_subgoal_service_available_ = false;

  // Check service availability periodically
  service_check_timer_ = rclcpp::create_timer(
    node,
    node->get_clock(),
    std::chrono::seconds(1),
    [this]() {
      if (!mark_subgoal_service_available_ && mark_subgoal_client_->service_is_ready()) {
        mark_subgoal_service_available_ = true;
        RCLCPP_INFO(logger_, "MarkSubgoal service is now available.");
      }
    });

  // Create service server for marking subgoals (if this checker itself needs to mark subgoals)
  mark_subgoal_service_ = node->create_service<gym_msgs::srv::MarkSubgoal>(
    "mark_subgoal_from_goal_checker",
    std::bind(
      &SubgoalIncludedGoalChecker::markSubgoalCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Set up dynamic parameters callback
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SubgoalIncludedGoalChecker::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "SubgoalIncludedGoalChecker initialized.");
}

void SubgoalIncludedGoalChecker::reset()
{
  // Reset any stateful information, e.g., if XY tolerance was reached
  num_marked_subgoals_ = 0;
  RCLCPP_INFO(logger_, "SubgoalIncludedGoalChecker reset.");
}

bool SubgoalIncludedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*linear_vel*/) // linear_vel is unused for now
{
  geometry_msgs::msg::PoseStamped current_pose_stamped;
  current_pose_stamped.header.frame_id = robot_frame_id_;
  // Use node's clock for timestamp
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to lock node in isGoalReached.");
    return false;
  }
  current_pose_stamped.header.stamp = node->get_clock()->now();
  current_pose_stamped.pose = query_pose;

  geometry_msgs::msg::PoseStamped goal_pose_stamped;
  goal_pose_stamped.header.frame_id = map_frame_id_; // Assuming goal_pose is in map frame
  goal_pose_stamped.header.stamp = node->get_clock()->now();
  goal_pose_stamped.pose = goal_pose;

  // Transform query_pose to map frame if it's not already, for consistent comparison
  geometry_msgs::msg::PoseStamped transformed_query_pose;
  try {
    tf_buffer_->transform(current_pose_stamped, transformed_query_pose, map_frame_id_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Failed to transform pose: %s", ex.what());
    return false; // Cannot check goal if transform fails
  }

  bool reached = withinTolerance(transformed_query_pose, goal_pose_stamped, xy_goal_tolerance_, yaw_goal_tolerance_);

  return reached;
}

bool SubgoalIncludedGoalChecker::withinTolerance(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  double xy_tolerance, double yaw_tolerance)
{
  double dx = goal_pose.pose.position.x - current_pose.pose.position.x;
  double dy = goal_pose.pose.position.y - current_pose.pose.position.y;
  double dist_sq = dx * dx + dy * dy;

  if (dist_sq > xy_tolerance * xy_tolerance) {
    return false; // Not within XY tolerance
  }

  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  double yaw_diff = angles::normalize_angle(goal_yaw - current_yaw);

  if (std::abs(yaw_diff) > yaw_tolerance) {
    return false; // Not within Yaw tolerance
  }

  return true; // Within both XY and Yaw tolerance
}

// Corrected name from getGoalTolerances to getTolerances
bool SubgoalIncludedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  // Set position tolerance
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = 0.0; // Z-tolerance is not typically used for 2D navigation

  // Set orientation tolerance
  pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  // Velocity tolerances are typically not used by goal checkers to report
  // what *they* tolerate, but rather by controllers. So, setting to 0.
  vel_tolerance.linear.x = 0.0;
  vel_tolerance.linear.y = 0.0;
  vel_tolerance.linear.z = 0.0;
  vel_tolerance.angular.x = 0.0;
  vel_tolerance.angular.y = 0.0;
  vel_tolerance.angular.z = 0.0;

  return true;
}

rcl_interfaces::msg::SetParametersResult
SubgoalIncludedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == plugin_name_ + ".xy_goal_tolerance") {
      xy_goal_tolerance_ = parameter.as_double();
      xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      RCLCPP_INFO(logger_, "Updated xy_goal_tolerance to: %.2f", xy_goal_tolerance_);
    } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
      yaw_goal_tolerance_ = parameter.as_double();
      RCLCPP_INFO(logger_, "Updated yaw_goal_tolerance to: %.2f", yaw_goal_tolerance_);
    } else if (name == plugin_name_ + ".stateful") {
      stateful_ = parameter.as_bool();
      RCLCPP_INFO(logger_, "Updated stateful to: %s", stateful_ ? "true" : "false");
    }
  }
  return result;
}

void SubgoalIncludedGoalChecker::markSubgoalCallback(
  const std::shared_ptr<gym_msgs::srv::MarkSubgoal::Request> /*request*/, // Request is unused for now
  std::shared_ptr<gym_msgs::srv::MarkSubgoal::Response> response)
{
  num_marked_subgoals_++;
  RCLCPP_INFO(logger_, "Subgoal marked as reached. Total marked subgoals: %zu", num_marked_subgoals_);
  response->success = true;
  // Temporarily removed response->message as per error analysis, if your .srv has it, add it back.
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SubgoalIncludedGoalChecker, nav2_core::GoalChecker)