/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/align_to_path.hpp"
#include <string>
#include <vector>
#include <cmath>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::AlignToPathCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void AlignToPathCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Load parameters with default values
  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  lookahead_points_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_points", 3);
  
  angle_threshold_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".angle_threshold", 0.2);  // ~11.5 degrees
  
  min_translational_vel_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".min_translational_vel", 1e-3);

  reset();
}

void AlignToPathCritic::reset()
{
  in_window_ = false;
  needs_alignment_ = false;
  target_heading_ = 0.0;
  current_heading_ = 0.0;
  optimal_rotation_sign_ = 0.0;
}

bool AlignToPathCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // deactivate when close to goal
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  in_window_ = dxy_sq <= xy_goal_tolerance_sq_;

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("AlignToPathCritic"), "Failed to lock node");
    return false;
  }

  // Reset alignment flag
  needs_alignment_ = false;
  current_heading_ = pose.theta;

  // Check if we have enough path points
  if (global_plan.poses.empty()) {
    RCLCPP_WARN(node->get_logger(), "AlignToPathCritic: Empty global plan");
    return true;
  }

  // find the closest point
  int best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (int i; i < static_cast<int>(global_plan.poses.size()); ++i)
  {
    double dx = global_plan.poses[i].x - pose.x;
    double dy = global_plan.poses[i].y - pose.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2)
    {
      best = i;
      best_d2 = d2;
    } 
  }

  // Calculate tangent direction from path
  std::vector<geometry_msgs::msg::Pose2D> path_points;
  int points_to_use = std::min(lookahead_points_, static_cast<int>(global_plan.poses.size()) - best);
  
  if (points_to_use < lookahead_points_) {
    RCLCPP_INFO(
      node->get_logger(),
      "AlignToPathCritic: Only %d path points available, using all (requested: %d)",
      points_to_use, lookahead_points_);
  }

  for (int i = 0; i < points_to_use; ++i) {
    path_points.push_back(global_plan.poses[i + best]);
  }

  // Calculate target heading (tangent direction)
  target_heading_ = calculateTangentDirection(path_points, pose);

  // Calculate angular error
  double angular_error = std::abs(angles::shortest_angular_distance(current_heading_, target_heading_));

  // Check if alignment is needed
  if (angular_error > angle_threshold_) {
    needs_alignment_ = true;
    optimal_rotation_sign_ = calculateOptimalRotationDirection(current_heading_, target_heading_);
    
    RCLCPP_DEBUG(
      node->get_logger(),
      "AlignToPathCritic: Alignment needed. Current: %.3f, Target: %.3f, Error: %.3f, Sign: %.1f",
      current_heading_, target_heading_, angular_error, optimal_rotation_sign_);
  }

  return true;
}

double AlignToPathCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd)
{
  // Check if human has translational input
  if (abs(human_cmd.linear.x) < min_translational_vel_ && abs(human_cmd.angular.z) < min_translational_vel_) {
    // Human has no significant translational input
    if (needs_alignment_ && !in_window_) {
      return scoreAlignment(traj);
    }
  }
  
  // Return neutral score when human is providing translational input or no alignment needed
  return 0.0;
}

double AlignToPathCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // For regular scoreTrajectory (without human command), only apply when alignment is needed
  if (needs_alignment_ && !in_window_) {
    return scoreAlignment(traj);
  }
  return 0.0;
}

double AlignToPathCritic::scoreAlignment(const dwb_msgs::msg::Trajectory2D & traj)
{
  // Check for translational movement - these should be illegal when aligning
  if (std::abs(traj.velocity.x) > 1e-6 || std::abs(traj.velocity.y) > 1e-6) {
    throw dwb_core::IllegalTrajectoryException(name_, "Translational movement during alignment.");
  }

  // Only rotational movements are allowed
  double angular_velocity = traj.velocity.theta;
  
  // If no rotation, give it a neutral score
  if (std::abs(angular_velocity) < 1e-6) {
    return 1.0;  // Higher score than optimal rotation
  }

  // Check if rotation is in the correct direction
  double rotation_sign = (angular_velocity > 0) ? 1.0 : -1.0;
  
  if (rotation_sign == optimal_rotation_sign_) {
    // Correct direction: give lower score to higher angular velocities
    // Score inversely proportional to angular velocity magnitude
    return 1.0 / (1.0 + std::abs(angular_velocity));
  } else {
    // Wrong direction: higher penalty
    return 2.0;
  }
}

double AlignToPathCritic::calculateTangentDirection(
  const std::vector<geometry_msgs::msg::Pose2D> & path_points,
  const geometry_msgs::msg::Pose2D & robot_pose)
{
  if (path_points.empty()) {
    return robot_pose.theta;  // Default to current heading
  }

  if (path_points.size() == 1) {
    // Single point: direction from robot to point
    double dx = path_points[0].x - robot_pose.x;
    double dy = path_points[0].y - robot_pose.y;
    return std::atan2(dy, dx);
  }

  // Multiple points: calculate average direction from n-1 consecutive point pairs
  double sum_x = 0.0, sum_y = 0.0;
  int valid_segments = 0;

  for (size_t i = 0; i < path_points.size() - 1; ++i) {
    double dx = path_points[i + 1].x - path_points[i].x;
    double dy = path_points[i + 1].y - path_points[i].y;
    
    // Check if segment is long enough to provide meaningful direction
    double segment_length = std::sqrt(dx * dx + dy * dy);
    if (segment_length > 1e-6) {
      // Normalize the direction vector
      dx /= segment_length;
      dy /= segment_length;
      
      sum_x += dx;
      sum_y += dy;
      valid_segments++;
    }
  }

  // If no valid segments found, fallback to robot-to-first-point direction
  if (valid_segments == 0) {
    double dx = path_points[0].x - robot_pose.x;
    double dy = path_points[0].y - robot_pose.y;
    return std::atan2(dy, dx);
  }

  // Calculate average direction
  double avg_x = sum_x / valid_segments;
  double avg_y = sum_y / valid_segments;
  
  return std::atan2(avg_y, avg_x);
}

double AlignToPathCritic::calculateOptimalRotationDirection(double from_angle, double to_angle)
{
  double angular_diff = angles::shortest_angular_distance(from_angle, to_angle);
  return (angular_diff >= 0) ? 1.0 : -1.0;  // +1 for CCW, -1 for CW
}

}  // namespace dwb_critics
