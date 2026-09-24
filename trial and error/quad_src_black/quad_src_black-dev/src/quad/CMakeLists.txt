cmake_minimum_required(VERSION 3.8)
project(quad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(quad
  "../interface/Encoder.msg"
  "../interface/State.msg"
  "../interface/Torque.msg"
  "../interface/Quadimu.msg"
  "../interface/HighCommands.msg"
  "../interface/Joystick.msg"
  "../interface/JointCmd.msg"
  "../interface/Action.msg"
  "../interface/StateCommand.msg"
  "../interface/QrResult.msg"
  "../interface/MotorState.msg"
  "../interface/Claw.msg"
  DEPENDENCIES sensor_msgs
)
install(
  DIRECTORY ../launch
  DESTINATION share/${PROJECT_NAME}
)
# install(
#  DIRECTORY ../config
#  DESTINATION share/${PROJECT_NAME}
# )
# install(
#   DIRECTORY ../models/a1
#   DESTINATION share/${PROJECT_NAME}
# )
# install(
#   DIRECTORY ../models/black
#   DESTINATION share/${PROJECT_NAME}
# )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
