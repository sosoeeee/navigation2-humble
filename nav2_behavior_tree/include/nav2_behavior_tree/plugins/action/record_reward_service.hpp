#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__RECORD_REWARD_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__RECORD_REWARD_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "gym_msgs/srv/record_reward.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps gym_msgs::srv::RecordReward
 */
class RecordRewardService : public BtServiceNode<gym_msgs::srv::RecordReward>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecordRewardService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  RecordRewardService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>(
          "state_related_reward", 1,
          "Additional reward related to the BT state"),
      });
  }
};
E
}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__RECORD_REWARD_SERVICE_HPP_
