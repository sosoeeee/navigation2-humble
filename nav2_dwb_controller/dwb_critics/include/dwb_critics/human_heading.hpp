#ifndef DWB_CRITICS__HUMAN_HEADING_HPP_
#define DWB_CRITICS__HUMAN_HEADING_HPP_

#include <string>
#include <vector>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace dwb_critics
{
/**
 * @class HumanHeadingCritic
 * @brief Compare the robot cmd with the human cmd in direction.
 */
class HumanHeadingCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd) override;

protected:
  double omega_range_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__HUMAN_HEADING_HPP_
