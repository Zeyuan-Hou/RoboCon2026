cmake_minimum_required(VERSION 3.8)
project(rknn_quad_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad REQUIRED)

include_directories(include /home/cat/hitcrt_quad_ws/src/controllers/common)

add_executable(rknn_quad_node src/rknn_quad_node.cpp)
add_executable(rknn_quad_zero_node src/rknn_quad_zero_node.cpp)

ament_target_dependencies(rknn_quad_node
  rclcpp
  std_msgs
  quad
)

target_link_libraries(rknn_quad_node
  rknnrt
)

install(TARGETS
  rknn_quad_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(rknn_quad_zero_node
  rclcpp
  std_msgs
)

target_link_libraries(rknn_quad_zero_node
  rknnrt
)

install(TARGETS
  rknn_quad_zero_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
