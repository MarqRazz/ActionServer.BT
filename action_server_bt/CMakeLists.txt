cmake_minimum_required(VERSION 3.16)
project(action_server_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_server_bt_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(THIS_PACKAGE_DEPS ament_index_cpp action_server_bt_msgs behaviortree_cpp
                      behaviortree_ros2 rclcpp rclcpp_action)

generate_parameter_library(action_server_bt_parameters
                           src/action_server_bt_parameters.yaml)

# ----------------------------------
add_library(action_server SHARED src/action_server_bt.cpp src/bt_utils.cpp)
target_include_directories(
  action_server PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_server ${THIS_PACKAGE_DEPS})
target_link_libraries(action_server action_server_bt_parameters)

# ----------------------------------
add_executable(action_server_node src/action_server_bt_node.cpp)
target_include_directories(
  action_server_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_server_node ${THIS_PACKAGE_DEPS})
target_link_libraries(action_server_node action_server)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install node and libraries
install(
  TARGETS action_server action_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
