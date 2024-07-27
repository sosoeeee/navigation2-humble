#include <chrono>
#include <string>
#include <iostream>

#include "nav2_behavior_tree/plugins/decorator/ros_rate_controller.hpp"

namespace nav2_behavior_tree
{

RosRateController::RosRateController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus RosRateController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = node_->now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = node_->now();
  auto elapsed = now - start_;

  // [DEBUG] Print the current time
  // auto now_msg = now.nanoseconds();
  // std::cout << "[RosRateController] Current time (nanoseconds since epoch): " << now_msg << std::endl;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        start_ = node_->now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RosRateController>("RosRateController");
}
