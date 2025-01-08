#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PREDICT_GOAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PREDICT_GOAL_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/predict_goal.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::PredictGoal
 */
class PredictGoalAction : public BtActionNode<nav2_msgs::action::PredictGoal>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PredictGoalAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  PredictGoalAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("predicted_goal", "Goal predited by RL agent"),
        BT::OutputPort<bool>("is_triggered", "Whether the replanning is triggered or not"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Global destination"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PREDICT_GOAL_ACTION_HPP_
