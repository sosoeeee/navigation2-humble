#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/human_vel.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::HumanVelCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void HumanVelCritic::onInit()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
        node, dwb_plugin_name_ + "." + name_ + ".vel_range",
        rclcpp::ParameterValue(1.0));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".vel_range", vel_range_);

    nav2_util::declare_parameter_if_not_declared(
        node, dwb_plugin_name_ + "." + name_ + ".v_max",
        rclcpp::ParameterValue(0.26));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".v_max", v_max_);

    nav2_util::declare_parameter_if_not_declared(
        node, dwb_plugin_name_ + "." + name_ + ".v_min",
        rclcpp::ParameterValue(-0.13));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".v_min", v_min_);
}

double HumanVelCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
    return scoreTrajectory(traj, geometry_msgs::msg::Twist{});
}

double HumanVelCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj, const geometry_msgs::msg::Twist & human_cmd)
{
    double score = 0.0;
    score = abs(traj.velocity.x - human_cmd.linear.x) / vel_range_;

    //debugging output
    // RCLCPP_INFO(
    //     rclcpp::get_logger("HumanVelCritic"),
    //     "Score for trajectory: %f (human cmd: [%f, %f], trajectory cmd: [%f, %f])",
    //     score, human_cmd.linear.x, human_cmd.angular.z,
    //     traj.velocity.x, traj.velocity.theta);
    return score;
}

double HumanVelCritic::scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & traj, 
    const geometry_msgs::msg::Twist & human_cmd,
    double avg_clearance)
{
    double score = 0.0;
    double amplitude = 0.0;

    if (traj.velocity.x >= 0){
        amplitude = traj.velocity.x / v_max_;
    }
    else{
        amplitude = traj.velocity.x / v_min_;    
    }

    score = avg_clearance * abs(traj.velocity.x - human_cmd.linear.x) / vel_range_ + (1 - avg_clearance) * amplitude;

    //debugging output
    RCLCPP_DEBUG(
        rclcpp::get_logger("HumanVelCritic"),
        "Score for trajectory: %f, avgClearance: %f",
        score, avg_clearance);
    return score;
}

}  // namespace dwb_critics
