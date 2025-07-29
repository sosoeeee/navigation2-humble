/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025, Human Trajectory Prediction
 * All rights reserved.
 */

#ifndef NAV2_CONTROLLER__PLUGINS__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "gym_msgs/srv/mark_subgoal.hpp"
#include "std_msgs/msg/empty.hpp"

namespace nav2_controller
{

/**
 * @class SubgoalIncludedGoalChecker
 * @brief Goal Checker plugin that checks for multiple subgoals before reaching the final goal
 */
class SubgoalIncludedGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  SubgoalIncludedGoalChecker();
  ~SubgoalIncludedGoalChecker();
  
  // Override parent class methods
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

protected:
  // Current robot pose
  geometry_msgs::msg::Pose current_pose_;
  
  // Subgoal list and management
  struct Subgoal {
    double x;
    double y;
    std::string name;
    bool reached;
  };
  
  std::vector<Subgoal> subgoals_;
  std::mutex subgoals_mutex_;
  bool all_subgoals_reached_;
  double subgoal_tolerance_;
  
  // Service for marking subgoals
  rclcpp::Service<gym_msgs::srv::MarkSubgoal>::SharedPtr mark_subgoal_service_;
  
  // Method to handle MarkSubgoal service requests
  void handleMarkSubgoalRequest(
    const std::shared_ptr<gym_msgs::srv::MarkSubgoal::Request> request,
    std::shared_ptr<gym_msgs::srv::MarkSubgoal::Response> response);
  
  // Check if a subgoal is reached
  bool isSubgoalReached(const geometry_msgs::msg::Pose & query_pose, const Subgoal & subgoal);
  
  // Load subgoals from parameters
  void loadSubgoalsFromParams();
  
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
  
  // ROS node pointer
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SUBGOAL_INCLUDED_GOAL_CHECKER_HPP_
