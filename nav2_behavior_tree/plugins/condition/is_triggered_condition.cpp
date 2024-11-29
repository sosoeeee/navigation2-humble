#include "nav2_behavior_tree/plugins/condition/is_triggered_condition.hpp"

namespace nav2_behavior_tree
{

BT::NodeStatus isTriggered(BT::TreeNode & tree_node)
{
  auto isTriggered = tree_node.config().blackboard->get<bool>("is_triggered");
  return isTriggered ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerSimpleCondition(
    "isTriggered",
    std::bind(&nav2_behavior_tree::isTriggered, std::placeholders::_1));
}
