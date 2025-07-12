#ifndef DWB_CRITICS__HUMAN_VEL_HPP_
#define DWB_CRITICS__HUMAN_VEL_HPP_

#include <string>
#include <vector>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace dwb_critics
{
/**
 * @class HumanVelCritic
 * @brief Compare the robot cmd with the human cmd in velocity.
 */
class HumanVelCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd, double avg_clearance) override;

protected:
  double vel_range_;
  double v_max_, v_min_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__HUMAN_VEL_HPP_