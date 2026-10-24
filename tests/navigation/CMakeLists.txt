cmake_minimum_required(VERSION 3.16)
project(navigation_regression)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp nav_msgs geometry_msgs pluginlib tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
  find_package(${dep} REQUIRED)
endforeach()
set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
add_executable(probe probe.cpp
  ${ROOT}/src/sp_controller_server/plugins/pid_controller.cpp
  ${ROOT}/src/sp_global_planner/plugins/astar_planner.cpp)
target_include_directories(probe PRIVATE
  ${ROOT}/src/sp_controller_server/include ${ROOT}/src/sp_controller_server/plugins
  ${ROOT}/src/sp_global_planner/include ${ROOT}/src/sp_global_planner/plugins)
ament_target_dependencies(probe rclcpp nav_msgs geometry_msgs pluginlib tf2 tf2_ros
  tf2_geometry_msgs visualization_msgs)
