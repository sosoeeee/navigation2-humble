#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/record_reward_service.hpp"

namespace nav2_behavior_tree
{

RecordRewardService::RecordRewardService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<gym_msgs::srv::RecordReward>(service_node_name, conf)
{
}

void RecordRewardService::on_tick()
{
  getInput("state_related_reward", request_->reward);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecordRewardService>(
    "RecordReward");
}
