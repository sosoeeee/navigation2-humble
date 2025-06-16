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
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

protected:
  void humanCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  bool cmd_received_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr human_cmd_sub_;
  geometry_msgs::msg::Twist human_cmd_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__HUMAN_HEADING_HPP_
