#ifndef NAV2_UTIL__STEPPING_CONTROL_UTILS_HPP_
#define NAV2_UTIL__STEPPING_CONTROL_UTILS_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <std_msgs/msg/string.hpp>

namespace nav2_util
{

class SteppingPublisher
{
private:
    /* data */
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string id_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
public:
    SteppingPublisher(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string id)
    : node_(node), id_(id)
    {
        pub_ = node_->create_publisher<std_msgs::msg::String>("stop_request", 1);
        RCLCPP_INFO(node_->get_logger(), "Stepping publisher created");
    }

    ~SteppingPublisher()
    {        
    }

    void on_activate()
    {
        pub_->on_activate();
    }

    void on_deactivate()
    {
        pub_->on_deactivate();
    }

    void reset()
    {
        pub_.reset();
    }

    void request_stop()
    {
        auto msg = std_msgs::msg::String();
        msg.data = id_ + " _s";  // s for start
        if (pub_->is_activated() && pub_->get_subscription_count() > 0) {
            pub_->publish(msg);
        }
    }

    void request_restart()
    {
        auto msg = std_msgs::msg::String();
        msg.data = id_ + " _e";  // e for end
        if (pub_->is_activated() && pub_->get_subscription_count() > 0) {
            pub_->publish(msg);
        }
    }
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__STEPPING_CONTROL_UTILS_HPP_
