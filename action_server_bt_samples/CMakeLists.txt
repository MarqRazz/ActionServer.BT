cmake_minimum_required(VERSION 3.8)
project(action_server_bt_samples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
