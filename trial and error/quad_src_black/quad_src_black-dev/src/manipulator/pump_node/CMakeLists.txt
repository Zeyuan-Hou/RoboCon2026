cmake_minimum_required(VERSION 3.8)
project(pump_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

# 查找gpiod库
find_library(GPIOD_LIBRARY gpiod)
find_path(GPIOD_INCLUDE_DIR gpiod.h PATHS /usr/include /usr/local/include)

if(GPIOD_LIBRARY AND GPIOD_INCLUDE_DIR)
  message(STATUS "Found libgpiod: ${GPIOD_LIBRARY}")
  message(STATUS "Found gpiod.h: ${GPIOD_INCLUDE_DIR}")
  set(GPIOD_FOUND TRUE)
else()
  message(WARNING "libgpiod not found. Pump node will not be built.")
  message(WARNING "To install: sudo apt-get install libgpiod-dev")
  set(GPIOD_FOUND FALSE)
endif()

if(GPIOD_FOUND)
  add_executable(pump_node src/pump_node.cpp)
  target_include_directories(pump_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GPIOD_INCLUDE_DIR})
  target_compile_features(pump_node PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(pump_node rclcpp std_msgs)
  target_link_libraries(pump_node ${GPIOD_LIBRARY})

  install(TARGETS pump_node
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
