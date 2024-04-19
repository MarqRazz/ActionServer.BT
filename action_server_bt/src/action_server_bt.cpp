// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "action_server_bt/action_server_bt.hpp"
#include "action_server_bt/bt_utils.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("action_server_bt");
}

namespace action_server_bt
{
ActionServerBT::ActionServerBT(const rclcpp::NodeOptions& options, OnTreeCreatedCallback tree_created,
                               OnTreeExecutionCompletedCallback execution_complete)
  : node_{ std::make_shared<rclcpp::Node>("action_server_bt", options) }
{
  // initialize user callbacks for tree creation and execution complete
  on_tree_created_ = tree_created;
  on_execution_complete_ = execution_complete;

  // parameter setup
  param_listener_ = std::make_shared<action_server_bt::ParamListener>(node_);
  params_ = param_listener_->get_params();

  // create the action server
  const auto action_name = params_.action_name;
  RCLCPP_INFO(kLogger, "Starting Action Server: %s", action_name.c_str());
  action_server_ = rclcpp_action::create_server<ActionTree>(
      node_, action_name,
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ActionTree::Goal> goal) {
        return handle_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleActionTree> goal_handle) { return handle_cancel(std::move(goal_handle)); },
      [this](const std::shared_ptr<GoalHandleActionTree> goal_handle) { handle_accepted(std::move(goal_handle)); });

  // register the users Plugins and BehaviorTree.xml files into the factory
  register_behavior_trees(params_, factory_, node_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ActionServerBT::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse ActionServerBT::handle_goal(const rclcpp_action::GoalUUID& /* uuid */,
                                                        std::shared_ptr<const ActionTree::Goal> goal)
{
  RCLCPP_INFO(kLogger, "Received goal request to execute Behavior Tree: %s", goal->target_tree.c_str());
  cancel_requested_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerBT::handle_cancel(const std::shared_ptr<GoalHandleActionTree> goal_handle)
{
  RCLCPP_INFO(kLogger, "Received request to cancel goal");
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(kLogger, "Rejecting request to cancel goal because the server is not processing one.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerBT::handle_accepted(const std::shared_ptr<GoalHandleActionTree> goal_handle)
{
  // Join the previous execute thread before replacing it with a new one
  if (action_thread_.joinable())
  {
    action_thread_.join();
  }
  // To avoid blocking the executor start a new thread to process the goal
  action_thread_ = std::thread{ [=]() { execute(goal_handle); } };
}

void ActionServerBT::execute(const std::shared_ptr<GoalHandleActionTree> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto action_result = std::make_shared<ActionTree::Result>();

  // Before executing check if we have new Behaviors or Subtrees to reload
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    register_behavior_trees(params_, factory_, node_);
  }

  // Loop until something happens with ROS or the node completes
  try
  {
    auto tree = factory_.createTree(goal->target_tree);

    // call user defined function after the tree has been created
    on_tree_created_(tree);
    groot_publisher_.reset();
    groot_publisher_ = std::make_shared<BT::Groot2Publisher>(tree, params_.groot2_port);

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      if (cancel_requested_)
      {
        action_result->error_message = "Action Server canceling and halting Behavior Tree";
        RCLCPP_ERROR(kLogger, action_result->error_message.c_str());
        tree.haltTree();
        goal_handle->canceled(action_result);
        return;
      }

      // tick the tree once and publish the action feedback
      status = tree.tickExactlyOnce();
      auto feedback = std::make_shared<ActionTree::Feedback>();
      feedback->node_status = convert_node_status(status);
      goal_handle->publish_feedback(feedback);

      // sleep to tick the tree at the desired frequency
      tree.sleep(std::chrono::milliseconds(static_cast<int>(1000.0 / params_.behavior_tick_frequency)));
    }
  }
  catch (const std::exception& ex)
  {
    action_result->error_message = "Behavior Tree exception: %s", ex.what();
    RCLCPP_ERROR(kLogger, action_result->error_message.c_str());
    goal_handle->abort(action_result);
    return;
  }

  // call user defined execution complete function
  on_execution_complete_(status);

  // set the node_status result to the action
  action_result->node_status = convert_node_status(status);

  // return success or aborted for the action result
  if (status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(kLogger, "BT finished with status: %s", BT::toStr(status).c_str());
    goal_handle->succeed(action_result);
  }
  else
  {
    action_result->error_message = "Behavior Tree failed during execution with status: %s", BT::toStr(status);
    RCLCPP_ERROR(kLogger, action_result->error_message.c_str());
    goal_handle->abort(action_result);
  }
}
}  // namespace action_server_bt
