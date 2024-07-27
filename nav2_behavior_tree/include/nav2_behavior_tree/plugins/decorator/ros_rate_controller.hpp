#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ROS_RATE_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ROS_RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate, using ros clock
 */
class RosRateController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RosRateController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RosRateController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  double period_;
  bool first_time_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ROS_RATE_CONTROLLER_HPP_
