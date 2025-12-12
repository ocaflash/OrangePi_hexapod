cmake_minimum_required(VERSION 3.5)
project(crab_gait)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(crab_msgs REQUIRED)

add_executable(gait_kinematics
  src/gait_kinematics.cpp
  src/gait.cpp
)
ament_target_dependencies(gait_kinematics
  rclcpp
  std_msgs
  kdl_parser
  orocos_kdl_vendor
  crab_msgs
)

install(TARGETS
  gait_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
