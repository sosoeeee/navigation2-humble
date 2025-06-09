#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/predict_goal_action.hpp"

namespace nav2_behavior_tree
{

PredictGoalAction::PredictGoalAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::PredictGoal>(xml_tag_name, action_name, conf)
{
}

void PredictGoalAction::on_tick()
{
  getInput("goal", goal_.goal);
}

BT::NodeStatus PredictGoalAction::on_success()
{
  setOutput("passed_goals", result_.result->passed_goals);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PredictGoalAction::on_aborted()
{
  // when goal is aborted, the predicted goal will be global goal
  setOutput("passed_goals", result_.result->passed_goals);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PredictGoalAction::on_cancelled()
{
  // when goal is cancelled, the predicted goal will be global goal
  setOutput("passed_goals", result_.result->passed_goals);
  return BT::NodeStatus::SUCCESS;
}

void PredictGoalAction::halt()
{
  std::vector<geometry_msgs::msg::PoseStamped> passed_goals;
  geometry_msgs::msg::PoseStamped global_goal;
  getInput("goal", global_goal);
  passed_goals.push_back(global_goal);
  setOutput("passed_goals", passed_goals);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::PredictGoalAction>(
        name, "predict_goal", config);
    };

  factory.registerBuilder<nav2_behavior_tree::PredictGoalAction>(
    "PredictGoal", builder);
}
