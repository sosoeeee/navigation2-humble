#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TRIGGERED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TRIGGERED_CONDITION_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace nav2_behavior_tree
{
/**
 * @brief A BT::ConditionNode that returns SUCCESS if replanning
 * has been triggered and FAILURE otherwise
 */
BT::NodeStatus isTriggered(BT::TreeNode & tree_node);
}

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TRIGGERED_CONDITION_HPP_
