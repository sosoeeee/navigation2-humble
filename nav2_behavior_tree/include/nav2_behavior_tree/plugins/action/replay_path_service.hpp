#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REPLAY_PATH_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REPLAY_PATH_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "gym_msgs/srv/replay_path.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps gym_msgs::srv::ReplayPath
 */
class ReplayPathService : public BtServiceNode<gym_msgs::srv::ReplayPath>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ReplayPathService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ReplayPathService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to put the replayed path on the blackboard.
   * @param response can be used to get the result of the service call in the BT Node.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to return another value
   */
  BT::NodeStatus on_completion(std::shared_ptr<gym_msgs::srv::ReplayPath::Response> response) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path to follow")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REPLAY_PATH_SERVICE_HPP_
