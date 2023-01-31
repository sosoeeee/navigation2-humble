// Copyright (c) 2023, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_route/operations_manager.hpp"

namespace nav2_route
{

OperationsManager::OperationsManager(nav2_util::LifecycleNode::SharedPtr node)
: plugin_loader_("nav2_route", "nav2_route::RouteOperation")
{
  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node, "use_feedback_operations", rclcpp::ParameterValue(false));
  use_feedback_operations_ = node->get_parameter("use_feedback_operations").as_bool();

  // Have some default operations
  const std::vector<std::string> default_plugin_id({"AdjustSpeedLimit"});
  const std::string default_plugin_type = "nav2_route::AdjustSpeedLimit";

  nav2_util::declare_parameter_if_not_declared(
    node, "operations", rclcpp::ParameterValue(default_plugin_id));
  auto operation_ids = node->get_parameter("operations").as_string_array();

  if (operation_ids == default_plugin_id) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_plugin_id[0] + ".plugin", rclcpp::ParameterValue(default_plugin_type));
  }

  // Create plugins and sort them into On Query, Status Change, and Graph-calling Operations
  for (size_t i = 0; i != operation_ids.size(); i++) {
    try {
      std::string type = nav2_util::get_plugin_type_param(node, operation_ids[i]);
      RouteOperation::Ptr operation = plugin_loader_.createSharedInstance(type);
      RCLCPP_INFO(
        node->get_logger(), "Created route operation %s of type %s",
        operation_ids[i].c_str(), type.c_str());
      operation->configure(node, operation_ids[i]);
      RouteOperationType process_type = operation->processType();
      if (process_type == RouteOperationType::ON_QUERY) {
        query_operations_.push_back(std::move(operation));
      } else if (process_type == RouteOperationType::ON_STATUS_CHANGE) {
        change_operations_.push_back(std::move(operation));
      } else {
        graph_operations_[operation->getName()] = operation;
      }
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        node->get_logger(),
        "Failed to create route operation. Exception: %s", ex.what());
      throw ex;
    }
  }
}

OperationsPtr OperationsManager::findGraphOperationsToProcess(
  const NodePtr node, const EdgePtr edge_enter, const EdgePtr edge_exit)
{
  OperationsPtr operations;
  Operations::iterator it;
  for (it = node->operations.begin(); it != node->operations.end(); ++it) {
    operations.push_back(&(*it));
  }
  if (edge_enter) {
    for (it = edge_enter->operations.begin(); it != edge_enter->operations.end(); ++it) {
      if (it->trigger == OperationTrigger::ON_ENTER) {
        operations.push_back(&(*it));
      }
    }
  }
  if (edge_exit) {
    for (it = edge_exit->operations.begin(); it != edge_exit->operations.end(); ++it) {
      if (it->trigger == OperationTrigger::ON_EXIT) {
        operations.push_back(&(*it));
      }
    }
  }
  return operations;
}

OperationsResult OperationsManager::process(
  const bool status_change,
  const RouteTrackingState & state,
  const Route & route,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Get important state information
  OperationsResult result;
  NodePtr node = state.last_node;
  EdgePtr edge_entered = state.current_edge;
  EdgePtr edge_exited =
    state.route_edges_idx > 0 ? route.edges[state.route_edges_idx - 1] : nullptr;

  if (status_change) {
    // Process operations defined in the navigation graph at node or edge
    OperationsPtr operations = findGraphOperationsToProcess(node, edge_entered, edge_exited);
    for (unsigned int i = 0; i != operations.size(); i++) {
      auto op = graph_operations_.find(operations[i]->type);
      if (op != graph_operations_.end()) {
        RouteOperation::Ptr & plugin = op->second;
        result.reroute = result.reroute ||
          plugin->perform(node, edge_entered, edge_exited, route, pose, &operations[i]->metadata);
        result.operations_triggered.push_back(plugin->getName());
      } else {
        if (use_feedback_operations_) {
          RCLCPP_INFO(
            logger_, "Operation %s should be called from feedback!",
            operations[i]->type.c_str());
          result.operations_triggered.push_back(operations[i]->type);
        } else {
          RCLCPP_WARN(
            logger_, "Operation %s does not exist in route operations loaded!",
            operations[i]->type.c_str());
        }
      }
    }

    // Process operations which trigger on any status changes
    for (OperationsIter it = change_operations_.begin(); it != change_operations_.end(); ++it) {
      RouteOperation::Ptr & plugin = *it;
      result.reroute = result.reroute ||
        plugin->perform(node, edge_entered, edge_exited, route, pose);
      result.operations_triggered.push_back(plugin->getName());
    }
  }

  // Process operations which trigger regardless of status change or nodes / edges
  for (OperationsIter it = query_operations_.begin(); it != query_operations_.end(); ++it) {
    RouteOperation::Ptr & plugin = *it;
    result.reroute = result.reroute ||
      plugin->perform(node, edge_entered, edge_exited, route, pose);
    result.operations_triggered.push_back(plugin->getName());
  }

  return result;
}

}  // namespace nav2_route