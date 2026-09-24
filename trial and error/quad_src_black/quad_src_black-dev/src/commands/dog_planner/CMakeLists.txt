cmake_minimum_required(VERSION 3.8)
project(dog_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

add_executable(local_planner_node src/local_planner_node.cpp)
ament_target_dependencies(local_planner_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

add_executable(local_planner_node_nav2 src/local_planner_node_nav2.cpp)
ament_target_dependencies(local_planner_node_nav2 
  rclcpp 
  rclcpp_action
  nav_msgs 
  nav2_msgs 
  geometry_msgs 
)

install(TARGETS
  global_planner_node
  local_planner_node
  local_planner_node_nav2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
