#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/human_involved.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

HumanInvolved::HumanInvolved(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  getInput("human_involved_path_length", human_involved_path_length_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  human_involved_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "human_involved_path",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HumanInvolved::callback_human_involved, this, _1),
    sub_option);
}

inline BT::NodeStatus HumanInvolved::tick()
{
  callback_group_executor_.spin_some();

  if (human_involving_ || child_node_status_ == BT::NodeStatus::RUNNING) {
    // first_tick_ = false;
    child_node_status_ = child_node_->executeTick();
    switch (child_node_status_) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }
  else{
    return BT::NodeStatus::SUCCESS;
  }
}

void
HumanInvolved::callback_human_involved(const std_msgs::msg::String::SharedPtr msg)
{
  // std::cout << "[BT_Decorator] Received human involved path length: " << msg->data << std::endl;

   if (std::stoi(msg->data) > human_involved_path_length_) {
     human_involving_ = true;
   }
   else {
     human_involving_ = false;
   }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::HumanInvolved>("HumanInvolved");
}
