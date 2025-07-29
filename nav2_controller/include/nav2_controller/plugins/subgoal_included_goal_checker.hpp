#ifndef SUBGOAL_INCLUDED_GOAL_CHECKER__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_
#define SUBGOAL_INCLUDED_GOAL_CHECKER__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

// Custom service for marking subgoals
#include "gym_msgs/srv/mark_subgoal.hpp"

namespace nav2_controller
{

class SubgoalIncludedGoalChecker : public nav2_core::GoalChecker
{
public:
  SubgoalIncludedGoalChecker();
  ~SubgoalIncludedGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & linear_vel) override;

  // Corrected name from getGoalTolerances to getTolerances
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // Dynamic parameters callback
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Service callback for marking subgoals
  void markSubgoalCallback(
    const std::shared_ptr<gym_msgs::srv::MarkSubgoal::Request> request,
    std::shared_ptr<gym_msgs::srv::MarkSubgoal::Response> response);

  // Private helper function for checking tolerance
  bool withinTolerance(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double xy_tolerance, double yaw_tolerance);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("SubgoalIncludedGoalChecker")};

  // Member variables for goal checking
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  bool stateful_; // If true, once XY is reached, it remains reached until reset()

  size_t num_marked_subgoals_;
  double xy_goal_tolerance_sq_; // Pre-calculated square for efficiency

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // TF Listener for current robot pose
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string robot_frame_id_;
  std::string map_frame_id_;

  // Service client and server for marking subgoals
  rclcpp::Client<gym_msgs::srv::MarkSubgoal>::SharedPtr mark_subgoal_client_;
  bool mark_subgoal_service_available_;
  rclcpp::TimerBase::SharedPtr service_check_timer_;
  rclcpp::Service<gym_msgs::srv::MarkSubgoal>::SharedPtr mark_subgoal_service_;
};

}  // namespace nav2_controller

#endif  // SUBGOAL_INCLUDED_GOAL_CHECKER__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_