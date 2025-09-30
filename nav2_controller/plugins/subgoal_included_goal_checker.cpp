/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025, Human Trajectory Prediction
 * All rights reserved.
 */

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include "nav2_controller/plugins/subgoal_included_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace nav2_controller
{

SubgoalIncludedGoalChecker::SubgoalIncludedGoalChecker()
: SimpleGoalChecker(),
  all_subgoals_reached_(false),
  subgoal_tolerance_(0.25),
  subgoal_frame_("map")
{ 
}

SubgoalIncludedGoalChecker::~SubgoalIncludedGoalChecker()
{
 
}

void SubgoalIncludedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Initialize parent class
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);
  
  // Store parent node and costmap
  parent_node_ = parent;
  costmap_ros_ = costmap_ros;
  
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node for SubgoalIncludedGoalChecker");
  }
  
  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".subgoal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".subgoal_frame", rclcpp::ParameterValue("map"));
  
  // Get parameters
  node->get_parameter(plugin_name + ".subgoal_tolerance", subgoal_tolerance_);
  node->get_parameter(plugin_name + ".subgoal_frame", subgoal_frame_);
  
  // Create service for marking subgoals
  auto mark_callback = std::bind(&SubgoalIncludedGoalChecker::handleMarkSubgoalRequest, 
                                 this, std::placeholders::_1, std::placeholders::_2);
  
  mark_subgoal_service_ = node->create_service<gym_msgs::srv::MarkSubgoal>(
    "mark_subgoal", mark_callback);
  
  // Create service for updating active subgoals
  auto update_callback = std::bind(&SubgoalIncludedGoalChecker::handleUpdateSubgoalsRequest,
                                   this, std::placeholders::_1, std::placeholders::_2);
  
  update_subgoals_service_ = node->create_service<gym_msgs::srv::UpdateSubgoals>(
    "update_subgoals", update_callback);
    
  // Create visualization publishers
  reached_subgoals_pub_ = node->create_publisher<MarkerArray>(
    "reached_subgoals", 10);
  failed_mark_pub_ = node->create_publisher<Marker>(
    "failed_mark", 10);
  
  // Load subgoals from parameters
  loadSubgoalsFromParams();
  
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SubgoalIncludedGoalChecker::dynamicParametersCallback, this, _1));
  
//  RCLCPP_INFO(node->get_logger(), 
  //  "SubgoalIncludedGoalChecker initialized with %zu subgoals, tolerance: %.2f, frame: %s",
 //   subgoals_.size(), subgoal_tolerance_, global_frame_.c_str());
}

void SubgoalIncludedGoalChecker::reset()
{
  SimpleGoalChecker::reset();
  
  // std::lock_guard<std::mutex> lock(subgoals_mutex_);
  
  // Reset the 'reached' status of all subgoals
  // for (auto & subgoal : subgoals_) {
  //   subgoal.reached = false;
  // }
  // all_subgoals_reached_ = false;
}

bool SubgoalIncludedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
 
  // // Update current robot pose and timestamp
  // current_pose_ = query_pose;
  // auto node = parent_node_.lock();
  // if (node) {
  //   current_pose_timestamp_ = node->get_clock()->now();
  // }
  
  // Check subgoals
  {
    std::lock_guard<std::mutex> lock(subgoals_mutex_);
    
    // If not all subgoals are reached, check each one
    if (!all_subgoals_reached_) {
      bool all_reached = true;
      
      for (auto & subgoal : subgoals_) {
        // Update all_reached flag
        if (!subgoal.reached) {
          all_reached = false;
          break;
        }
      }
      
      all_subgoals_reached_ = all_reached;
      
      auto node = parent_node_.lock();
      if (all_subgoals_reached_ && node) {
        RCLCPP_INFO(node->get_logger(), "All subgoals reached!");
      }
    }
  }
  
  // If all subgoals are reached, then check the final goal with parent method
  if (all_subgoals_reached_) {
    return SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  }
  
  return false;
}

bool SubgoalIncludedGoalChecker::isSubgoalReached(
  const geometry_msgs::msg::PoseStamped & query_pose, const Subgoal & subgoal)
{
  auto node = parent_node_.lock();
  if (!node || !costmap_ros_) {
    return false;
  }
  
  // Transform subgoal from subgoal_frame to global frame if needed
  double x = subgoal.x;
  double y = subgoal.y;
  
  if (subgoal_frame_ != query_pose.header.frame_id) {
    try {
      // Create a pose stamped in the subgoal frame
      geometry_msgs::msg::PoseStamped subgoal_pose;
      subgoal_pose.header.frame_id = subgoal_frame_;
      subgoal_pose.header.stamp = node->get_clock()->now();
      subgoal_pose.pose.position.x = x;
      subgoal_pose.pose.position.y = y;
      subgoal_pose.pose.orientation.w = 1.0;
      
      // Transform to global frame
      geometry_msgs::msg::PoseStamped transformed_pose;
      rclcpp::Duration transform_tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
      nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), query_pose.header.frame_id,
        subgoal_pose, transformed_pose, transform_tolerance);
      
      // Get transformed coordinates
      x = transformed_pose.pose.position.x;
      y = transformed_pose.pose.position.y;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to transform subgoal from %s to %s: %s",
        subgoal_frame_.c_str(), query_pose.header.frame_id.c_str(), ex.what());
      return false;
    }
  }
  
  // Check if the distance to the subgoal is within tolerance
  double dx = query_pose.pose.position.x - x;
  double dy = query_pose.pose.position.y - y;

  RCLCPP_WARN(node->get_logger(), 
      "distance to subgoal (%.2f, %.2f)", 
      dx, dy);

  return (dx * dx + dy * dy) <= (subgoal_tolerance_ * subgoal_tolerance_);
}

void SubgoalIncludedGoalChecker::loadSubgoalsFromParams()
{
  auto node = parent_node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node for SubgoalIncludedGoalChecker");
  }
  
  std::lock_guard<std::mutex> lock(subgoals_mutex_);
  
  subgoals_.clear();
  active_subgoals_.clear();
  
  // Declare parameter for number of subgoals
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".num_subgoals", rclcpp::ParameterValue(0));
  
  // Declare parameter for active subgoals
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".active_subgoals", rclcpp::ParameterValue(std::vector<int64_t>()));
  
  int num_subgoals = 0;
  node->get_parameter(plugin_name_ + ".num_subgoals", num_subgoals);
  node->get_parameter(plugin_name_ + ".active_subgoals", active_subgoals_);
  
  // Load all subgoals first
  for (int i = 0; i < num_subgoals; i++) {
    std::string prefix = plugin_name_ + ".subgoal_" + std::to_string(i);
    
    // Declare parameters for this subgoal
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".x", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".y", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".name", rclcpp::ParameterValue("subgoal_" + std::to_string(i)));
    
    // Get parameters
    double x = 0.0, y = 0.0;
    std::string name = "subgoal_" + std::to_string(i);
    
    node->get_parameter(prefix + ".x", x);
    node->get_parameter(prefix + ".y", y);
    node->get_parameter(prefix + ".name", name);
    
    // Add to all subgoals list
    all_subgoals_.push_back({x, y, name, false});
  }
  
  // Only add active subgoals to the main subgoals_ list
  for (auto active_idx : active_subgoals_) {
    if (active_idx >= 0 && active_idx < static_cast<int>(all_subgoals_.size())) {
      subgoals_.push_back(all_subgoals_[active_idx]);
      RCLCPP_WARN(node->get_logger(), 
        "Activated subgoal '%s' at (%.2f, %.2f) in frame %s", 
        all_subgoals_[active_idx].name.c_str(), 
        all_subgoals_[active_idx].x, 
        all_subgoals_[active_idx].y, 
        subgoal_frame_.c_str());
    } else {
      RCLCPP_WARN(node->get_logger(), 
        "Invalid active subgoal index: %ld (total subgoals: %zu)", 
        active_idx, all_subgoals_.size());
    }
  }
  
  RCLCPP_INFO(node->get_logger(), 
    "Loaded %zu active subgoals out of %d total subgoals", 
    subgoals_.size(), num_subgoals);
}

void SubgoalIncludedGoalChecker::handleMarkSubgoalRequest(
  const std::shared_ptr<gym_msgs::srv::MarkSubgoal::Request> request,
  std::shared_ptr<gym_msgs::srv::MarkSubgoal::Response> response)
{
  std::lock_guard<std::mutex> lock(subgoals_mutex_);
  
  response->success = false;
  response->marked_subgoal_name = "";
  
  // Get current robot pose - check if cached pose is too old
  geometry_msgs::msg::PoseStamped current_pose; // map frame also
  current_pose = request->cur_pose;
  
  // Check each subgoal
  for (auto & subgoal : subgoals_) {
    if (!subgoal.reached && isSubgoalReached(current_pose, subgoal)) {
      subgoal.reached = true;
      response->success = true;
      response->marked_subgoal_name = subgoal.name;
      
      if (auto node = parent_node_.lock()) {
        RCLCPP_INFO(node->get_logger(), 
          "Marked subgoal '%s' (%.2f, %.2f) as reached via service call", 
          subgoal.name.c_str(), subgoal.x, subgoal.y);
        
      }
      
      // Update visualization
      publishReachedSubgoals();
      break;
    }
  }
  
  // If no subgoal was marked as reached, publish the failed position
  if (!response->success) {
    publishFailedMark(current_pose);
  }

  // debug
  // if (auto node = parent_node_.lock()) {
  //   for (auto & subgoal : subgoals_) {
  //     int reached = 0;
  //     if (subgoal.reached){
  //       reached = 1;
  //     }
  //     RCLCPP_WARN(node->get_logger(), 
  //     "Subgoal '%s' status: %d", 
  //     subgoal.name.c_str(), reached);
  //   }
  //   RCLCPP_WARN(node->get_logger(),"\n");
  // }
}

rcl_interfaces::msg::SetParametersResult
SubgoalIncludedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  bool need_reload = false;
  
  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".subgoal_tolerance") {
        subgoal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".subgoal_frame") {
        subgoal_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER_ARRAY) {
      if (name == plugin_name_ + ".active_subgoals") {
        // Reload subgoals when active_subgoals parameter changes
        need_reload = true;
      }
    }
  }
  
  // Reload subgoals if active_subgoals parameter changed
  if (need_reload) {
    loadSubgoalsFromParams();
  }
  
  return result;
}

void SubgoalIncludedGoalChecker::publishReachedSubgoals()
{
  auto node = parent_node_.lock();
  if (!node) {
    return;
  }
 
  MarkerArray marker_array;

  int id = 0;
  for (const auto & subgoal : subgoals_) {
    if (subgoal.reached) {
      Marker marker;
      marker.header.frame_id = subgoal_frame_;
      marker.header.stamp = node->get_clock()->now();
      marker.ns = "reached_subgoals";
      marker.id = id++;
      marker.type = Marker::SPHERE;
      marker.action = Marker::ADD;
      
      marker.pose.position.x = subgoal.x;
      marker.pose.position.y = subgoal.y;
      marker.pose.position.z = 0.2;  // Slightly above ground
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      
      marker.color.r = 0.0;
      marker.color.g = 1.0;  // Green
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent
      
      marker_array.markers.push_back(marker);
    }
  }
  reached_subgoals_pub_->publish(marker_array);
}


void SubgoalIncludedGoalChecker::publishFailedMark(const geometry_msgs::msg::PoseStamped & pose)
{
  auto node = parent_node_.lock();
  if (!node) {
    return;
  }
  
  Marker marker;
  marker.header.frame_id = pose.header.frame_id;
  marker.header.stamp = node->get_clock()->now();
  marker.ns = "failed_mark";
  marker.id = 0;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  
  marker.pose.position.x = pose.pose.position.x;
  marker.pose.position.y = pose.pose.position.y;
  marker.pose.position.z = 0.2;  // Slightly above ground
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.r = 1.0;  // Red
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  marker.lifetime = rclcpp::Duration::from_seconds(5.0);  // Display for 5 seconds
  
  failed_mark_pub_->publish(marker);
}

void SubgoalIncludedGoalChecker::handleUpdateSubgoalsRequest(
  const std::shared_ptr<gym_msgs::srv::UpdateSubgoals::Request> request,
  std::shared_ptr<gym_msgs::srv::UpdateSubgoals::Response> response)
{
  auto node = parent_node_.lock();
  if (!node) {
    response->success = false;
    response->message = "Can not get parent node";
    return;
  }

  std::lock_guard<std::mutex> lock(subgoals_mutex_);
   
  // reset visulization
  MarkerArray marker_array;
  for (int id = 0; id < static_cast<int>(subgoals_.size()); id++) {
    Marker marker;
    marker.header.frame_id = subgoal_frame_;
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "reached_subgoals";
    marker.id = id;
    marker.action = Marker::DELETE;
    marker_array.markers.push_back(marker);
  }
  reached_subgoals_pub_->publish(marker_array);

  // Clear the active subgoals
  subgoals_.clear();
  all_subgoals_reached_ = false;
  
  try {
    // Create new subgoals from the active indices
    for (const auto& index : request->active_subgoal_indices) {
      if (index >= 0 && index < static_cast<int>(all_subgoals_.size())) {
        Subgoal subgoal;
        subgoal.x = all_subgoals_[index].x;
        subgoal.y = all_subgoals_[index].y;
        subgoal.name = all_subgoals_[index].name;
        subgoal.reached = false;  // Reset all reached flags to false
        subgoals_.push_back(subgoal);

        // // debug
        //   if (auto node = parent_node_.lock()) {
        //       RCLCPP_WARN(node->get_logger(), 
        //         "Add subgoal: %ld", index);
        //   }

      } else {
        response->success = false;
        response->message = "Invalid subgoal index: " + std::to_string(index);
        return;
      }
    }
    
    response->success = true;
    response->message = "Active subgoals updated successfully";
    
    if (auto node = parent_node_.lock()) {
      RCLCPP_INFO(node->get_logger(), 
        "Updated active subgoals: %zu subgoals loaded", subgoals_.size());
    }
    
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Error updating subgoals: " + std::string(e.what());
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SubgoalIncludedGoalChecker, nav2_core::GoalChecker)