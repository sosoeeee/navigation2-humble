#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_INVOLVED_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_INVOLVED_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that subscribes to a topic of the length of the human involved path. It 
 * ticks its child node only if the human involved path is greater than human_involved_path_length.
 */
class HumanInvolved : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::HumanInvolved
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  HumanInvolved(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("human_involved_path_length", 5, "Length of the human involved path to trigger the child node")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to std_msgs::msg::String message
   */
  void callback_human_involved(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr human_involved_sub_;
  bool human_involving_ = false;
  bool first_tick_ = true;
  int human_involved_path_length_ = 0;
  BT::NodeStatus child_node_status_ = BT::NodeStatus::IDLE;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_INVOLVED_HPP_
