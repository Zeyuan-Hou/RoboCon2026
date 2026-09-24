cmake_minimum_required(VERSION 3.8)
project(lidar_nav_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)

# add executable
include_directories(include)
add_executable(lidar_nav_demo_node
  src/lidar_nav_demo_node.cpp
  src/lidar_nav_callbacks.cpp
  src/lidar_nav_path.cpp
  src/lidar_nav_control.cpp
  src/lidar_nav_auto_parku.cpp
  src/lidar_A_star.cpp
  src/isolation_band.cpp
  src/nav_controller.cpp
  src/field_layout_loader.cpp
)
ament_target_dependencies(lidar_nav_demo_node
  rclcpp rcl_interfaces geometry_msgs tf2_msgs std_msgs tf2 tf2_geometry_msgs ament_index_cpp
)
target_link_libraries(lidar_nav_demo_node nlohmann_json::nlohmann_json)



# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS 
  lidar_nav_demo_node
  
  DESTINATION lib/${PROJECT_NAME})

ament_package()
