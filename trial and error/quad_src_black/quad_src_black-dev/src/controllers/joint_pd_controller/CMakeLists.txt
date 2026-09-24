cmake_minimum_required(VERSION 3.8)
project(joint_pd_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(joint_control_node src/joint_control_node.cpp)
ament_target_dependencies(joint_control_node rclcpp quad) 
target_include_directories(joint_control_node PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/joint_pd_controller
    ../common
    )

install(TARGETS 
  joint_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
