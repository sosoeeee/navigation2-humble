#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/replay_path_service.hpp"

namespace nav2_behavior_tree
{

ReplayPathService::ReplayPathService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<gym_msgs::srv::ReplayPath>(service_node_name, conf)
{
}

BT::NodeStatus ReplayPathService::on_completion(std::shared_ptr<gym_msgs::srv::ReplayPath::Response> response)
{
  setOutput("path", response->path);
  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ReplayPathService>(
    "ReplayPath");
}

