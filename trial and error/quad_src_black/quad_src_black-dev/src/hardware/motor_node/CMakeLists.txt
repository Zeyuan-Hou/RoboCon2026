cmake_minimum_required(VERSION 3.8)
project(motor_node)

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

include_directories(./include/motor_node ./comm_basic)

# comm_basic library
add_library(comm_basic SHARED comm_basic/udp_basic.cpp comm_basic/udp_comm.cpp)
install(TARGETS
  comm_basic
  LIBRARY DESTINATION lib
)

# motor_node
add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node comm_basic)
ament_target_dependencies(motor_node rclcpp quad)

install(
  TARGETS motor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
