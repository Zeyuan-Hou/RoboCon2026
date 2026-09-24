cmake_minimum_required(VERSION 3.8)
project(aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc calib3d aruco)
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
 
# 2. 编译节点
add_executable(aruco_processor_node src/aruco_processor.cpp)

# 链接 OpenCV 和消息类型支持
target_link_libraries(aruco_processor_node
  ${OpenCV_LIBRARIES}
)

# 包含头文件
target_include_directories(aruco_processor_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aruco_processor_node
  rclcpp
  quad
)

# 3. 安装指令
install(TARGETS
  aruco_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )


ament_package()
