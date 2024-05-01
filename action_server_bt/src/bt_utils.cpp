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

#include "action_server_bt/bt_utils.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("action_server_bt");
}

namespace action_server_bt
{

action_server_bt_msgs::msg::NodeStatus convert_node_status(BT::NodeStatus& status)
{
  action_server_bt_msgs::msg::NodeStatus action_status;
  switch (status)
  {
    case BT::NodeStatus::RUNNING:
      action_status.status = action_server_bt_msgs::msg::NodeStatus::RUNNING;
    case BT::NodeStatus::SUCCESS:
      action_status.status = action_server_bt_msgs::msg::NodeStatus::SUCCESS;
    case BT::NodeStatus::FAILURE:
      action_status.status = action_server_bt_msgs::msg::NodeStatus::FAILURE;
    case BT::NodeStatus::IDLE:
      action_status.status = action_server_bt_msgs::msg::NodeStatus::IDLE;
    case BT::NodeStatus::SKIPPED:
      action_status.status = action_server_bt_msgs::msg::NodeStatus::SKIPPED;
  }

  return action_status;
}

std::string get_directory_path(const std::string& parameter_value)
{
  std::string package_name, subfolder;
  auto pos = parameter_value.find_first_of("/");
  if (pos == parameter_value.size())
  {
    RCLCPP_ERROR(kLogger, "Invalid Parameter: %s. Missing subfolder delimiter '/'.", parameter_value.c_str());
    return "";
  }

  package_name = std::string(parameter_value.begin(), parameter_value.begin() + pos);
  subfolder = std::string(parameter_value.begin() + pos + 1, parameter_value.end());
  try
  {
    std::string search_directory = ament_index_cpp::get_package_share_directory(package_name) + "/" + subfolder;
    RCLCPP_DEBUG(kLogger, "Searching for Plugins/BehaviorTrees in path: %s", search_directory.c_str());
    return search_directory;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to find package: %s \n %s", package_name.c_str(), e.what());
  }
  return "";
}

void load_behavior_trees(BT::BehaviorTreeFactory& factory, const std::string& directory_path)
{
  using std::filesystem::directory_iterator;
  for (const auto& entry : directory_iterator(directory_path))
  {
    if (entry.path().extension() == ".xml")
    {
      try
      {
        factory.registerBehaviorTreeFromFile(entry.path().string());
        RCLCPP_INFO(kLogger, "Loaded BehaviorTree: %s", entry.path().filename().c_str());
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(kLogger, "Failed to load BehaviorTree: %s \n %s", entry.path().filename().c_str(), e.what());
      }
    }
  }
}

void load_plugins(BT::BehaviorTreeFactory& factory, const std::string& directory_path)
{
  using std::filesystem::directory_iterator;
  for (const auto& entry : directory_iterator(directory_path))
  {
    if (entry.path().extension() == ".so")
    {
      try
      {
        factory.registerFromPlugin(entry.path().string());
        RCLCPP_INFO(kLogger, "Loaded Plugin: %s", entry.path().filename().c_str());
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(kLogger, "Failed to load Plugin: %s \n %s", entry.path().filename().c_str(), e.what());
      }
    }
  }
}

void load_ros_plugins(BT::BehaviorTreeFactory& factory, const std::string& directory_path, rclcpp::Node::SharedPtr node)
{
  using std::filesystem::directory_iterator;
  BT::RosNodeParams params;
  params.nh = node;
  for (const auto& entry : directory_iterator(directory_path))
  {
    if (entry.path().extension() == ".so")
    {
      try
      {
        RegisterRosNode(factory, entry.path().string(), params);
        RCLCPP_INFO(kLogger, "Loaded ROS Plugin: %s", entry.path().filename().c_str());
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(kLogger, "Failed to load ROS Plugin: %s \n %s", entry.path().filename().c_str(), e.what());
      }
    }
  }
}

void register_behavior_trees(action_server_bt::Params& params, BT::BehaviorTreeFactory& factory,
                             rclcpp::Node::SharedPtr node)
{
  // clear the factory and load/reload it with the Behaviors and Trees specified by the user in their action_server_bt config yaml
  factory.clearRegisteredBehaviorTrees();

  for (const auto& plugin : params.plugins)
  {
    const auto plugin_directory = get_directory_path(plugin);
    // skip invalid plugins directories
    if (plugin_directory.empty())
      continue;
    load_plugins(factory, plugin_directory);
  }
  for (const auto& plugin : params.ros_plugins)
  {
    const auto plugin_directory = get_directory_path(plugin);
    // skip invalid plugins directories
    if (plugin_directory.empty())
      continue;
    load_ros_plugins(factory, plugin_directory, node);
  }
  for (const auto& tree_dir : params.behavior_trees)
  {
    const auto tree_directory = get_directory_path(tree_dir);
    // skip invalid subtree directories
    if (tree_directory.empty())
      continue;
    load_behavior_trees(factory, tree_directory);
  }
}

}  // namespace action_server_bt
