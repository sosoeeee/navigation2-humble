#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/human_heading.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::HumanHeadingCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void HumanHeadingCritic::onInit()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
        node, dwb_plugin_name_ + "." + name_ + ".omega_range",
        rclcpp::ParameterValue(1.0));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".omega_range", omega_range_);
}

double HumanHeadingCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd)
{
    double score = 0.0;
    if (human_cmd.linear.x == 0.0){
        score = abs(traj.velocity.theta - human_cmd.angular.z) / omega_range_;
    }
    else
    {
        score = abs(std::atan2(traj.velocity.x, traj.velocity.theta) -
                    std::atan2(human_cmd.linear.x, human_cmd.angular.z)) / M_PI;
    }

    //debugging output
    // RCLCPP_INFO(
    //     rclcpp::get_logger("HumanHeadingCritic"),
    //     "Score for trajectory: %f (human cmd: [%f, %f], trajectory cmd: [%f, %f])",
    //     score, human_cmd_.linear.x, human_cmd_.angular.z,
    //     traj.velocity.x, traj.velocity.theta);

    return score;
}

}  // namespace dwb_critics
