cmake_minimum_required(VERSION 3.16)
project(action_server_bt_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES "msgs/NodeStatus.msg")

set(ACTION_FILES "action/ActionTree.action")

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} ${ACTION_FILES})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
