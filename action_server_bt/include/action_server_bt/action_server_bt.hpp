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

#include <functional>
#include <memory>
#include <thread>

#include "action_server_bt_msgs/action/action_tree.hpp"
#include "action_server_bt_parameters.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_server_bt
{

typedef std::function<void(BT::Tree&)> OnTreeCreatedCallback;
typedef std::function<void(BT::Blackboard&)> OnLoopCallback;
typedef std::function<void(BT::NodeStatus&)> OnTreeExecutionCompletedCallback;

/**
 * @brief UserCallbacks is a collection of functions that get called
 * after the tree is created, while its running and after it finishes.
 */
struct UserCallbacks
{
  OnTreeCreatedCallback on_create = [](BT::Tree&) {};
  OnLoopCallback on_loop = [](BT::Blackboard&) {};
  OnTreeExecutionCompletedCallback execution_complete = [](BT::NodeStatus&) {};
};

/**
 * @brief ActionServerBT class hosts a ROS Action Server that is able
 * to load Behavior plugins, BehaviorTree.xml files and execute them.
 */
class ActionServerBT
{
public:
  using ActionTree = action_server_bt_msgs::action::ActionTree;
  using GoalHandleActionTree = rclcpp_action::ServerGoalHandle<ActionTree>;

  /**
   * @brief Constructor for ActionServerBT.
   * @details This initializes a ParameterListener to read configurable options from the user and
   * starts an Action Server that takes requests to execute BehaviorTrees.
   *
   * @param options rclcpp::NodeOptions to pass to node_ when initializing it.
   * @param user_callbacks optional struct of user defined function that are called
   * after the tree is created, while its running and after it finishes.
   */
  explicit ActionServerBT(const rclcpp::NodeOptions& options, const UserCallbacks& user_callbacks = UserCallbacks());

  /**
   * @brief Gets the NodeBaseInterface of node_.
   * @details This function exists to allow running ActionServerBT as a component in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of node_.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<ActionTree>::SharedPtr action_server_;
  std::thread action_thread_;
  // thread safe bool to keep track of cancel requests
  std::atomic<bool> cancel_requested_{ false };

  std::shared_ptr<action_server_bt::ParamListener> param_listener_;
  action_server_bt::Params params_;

  BT::BehaviorTreeFactory factory_;
  UserCallbacks user_cbs_;
  std::shared_ptr<BT::Groot2Publisher> groot_publisher_;

  /**
   * @brief handle the goal requested: accept or reject. This implementation always accepts.
   * @param uuid Goal ID
   * @param goal A shared pointer to the specific goal
   * @return GoalResponse response of the goal processed
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const ActionTree::Goal> goal);

  /**
   * @brief Accepts cancellation requests of action server.
   * @param goal A server goal handle to cancel
   * @return CancelResponse response of the goal cancelled
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleActionTree> goal_handle);

  /**
   * @brief Handles accepted goals and starts a thread to process them
   * @param goal_handle Server goal handle to process feedback and set the response when finished
   */
  void handle_accepted(const std::shared_ptr<GoalHandleActionTree> goal_handle);

  /**
   * @brief Background processes to execute the BehaviorTree and handle stop requests
   * @param goal_handle Server goal handle to process feedback and set the response when finished
   */
  void execute(const std::shared_ptr<GoalHandleActionTree> goal_handle);
};

}  // namespace action_server_bt
