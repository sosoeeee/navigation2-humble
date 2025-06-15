#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/human_heading.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::HumanHeadingCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void HumanHeadingCritic::onInit()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    cmd_received_ = false;

    nav2_util::declare_parameter_if_not_declared(
        node, dwb_plugin_name_ + "." + name_ + ".human_cmd_topic",
        rclcpp::ParameterValue("human_cmd"));
    std::string human_cmd_topic;
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".human_cmd_topic", human_cmd_topic);

    human_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        human_cmd_topic, rclcpp::QoS(10),
        std::bind(&HumanHeadingCritic::humanCmdCallback, this, std::placeholders::_1));
}

void HumanHeadingCritic::humanCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    human_cmd_ = *msg;
    cmd_received_ = true;
}

double HumanHeadingCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
    double score = 0.0;

    if (!cmd_received_) {
        RCLCPP_WARN(
            rclcpp::get_logger("HumanHeadingCritic"),
            "No human command received, returning score of 0.0");
        return score;
    }

    score = abs(std::atan2(traj.velocity.x, traj.velocity.theta) -
                std::atan2(human_cmd_.linear.x, human_cmd_.angular.z)) / M_PI;

    //debugging output
    RCLCPP_INFO(
        rclcpp::get_logger("HumanHeadingCritic"),
        "Score for trajectory: %f (human cmd: [%f, %f], trajectory cmd: [%f, %f])",
        score, human_cmd_.linear.x, human_cmd_.angular.z,
        traj.velocity.x, traj.velocity.theta);

    return score;
}

}  // namespace dwb_critics
