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
#ifndef DWB_CRITICS__ALIGN_TO_PATH_HPP_
#define DWB_CRITICS__ALIGN_TO_PATH_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

/**
 * @class AlignToPathCritic
 * @brief Forces the robot to align its heading with the path direction when human is not providing translational input
 *
 * This critic guides the robot to face forward along the current trajectory. The implementation logic:
 * 1) In prepare function, determine if robot heading needs adjustment based on current heading and local trajectory
 * 2) Select first n trajectory points (n is a configurable parameter) to compute tangent direction at trajectory start
 * 3) Calculate error between tangent direction and current robot heading
 * 4) If error exceeds threshold, set alignment flag to true
 * 5) In scoreTrajectory, when human has no translational input and alignment flag is true:
 *    - Mark all translational commands as illegal
 *    - Give minimum score to rotational command with maximum angular velocity in correct direction
 */
class AlignToPathCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd) override;

  /**
   * @brief Score rotation trajectories when alignment is required
   * @param traj Trajectory to score
   * @return numeric score (lower is better, negative means illegal)
   */
  virtual double scoreAlignment(const dwb_msgs::msg::Trajectory2D & traj);

private:
  /**
   * @brief Calculate tangent direction from path points
   * @param path_points Vector of path points
   * @param robot_pose Current robot pose
   * @return Tangent direction in radians
   */
  double calculateTangentDirection(
    const std::vector<geometry_msgs::msg::Pose2D> & path_points,
    const geometry_msgs::msg::Pose2D & robot_pose);

  /**
   * @brief Calculate the shortest angular distance and determine optimal rotation direction
   * @param from_angle Starting angle
   * @param to_angle Target angle
   * @return Signed angular distance (positive for counter-clockwise, negative for clockwise)
   */
  double calculateOptimalRotationDirection(double from_angle, double to_angle);

  // Configuration parameters
  int lookahead_points_;           ///< Number of path points to use for tangent calculation
  double angle_threshold_;         ///< Angle threshold to trigger alignment (radians)
  double min_translational_vel_;   ///< Minimum translational velocity threshold for human input

  // State variables
  bool needs_alignment_;           ///< Flag indicating if robot needs heading alignment
  double target_heading_;          ///< Target heading direction (tangent to path)
  double current_heading_;         ///< Current robot heading
  double optimal_rotation_sign_;   ///< +1 for counter-clockwise, -1 for clockwise rotation

  bool in_window_;
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;  ///< Cached squared tolerance
};

}  // namespace dwb_critics

#endif  // DWB_CRITICS__ALIGN_TO_PATH_HPP_
