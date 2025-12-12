cmake_minimum_required(VERSION 3.5)
project(crab_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyCommand.msg"
  "msg/BodyState.msg"
  "msg/GaitCommand.msg"
  "msg/LegIKRequest.msg"
  "msg/LegJointsState.msg"
  "msg/LegPositionState.msg"
  "msg/LegsJointsState.msg"
  "srv/GetLegIKSolver.srv"
  DEPENDENCIES std_msgs
)

ament_package()
