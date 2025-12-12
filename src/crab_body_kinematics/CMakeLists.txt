cmake_minimum_required(VERSION 3.5)
project(crab_body_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(crab_msgs REQUIRED)

add_executable(body_kinematics src/body_kinematics.cpp)
ament_target_dependencies(body_kinematics
  rclcpp
  std_msgs
  kdl_parser
  orocos_kdl_vendor
  crab_msgs
)

install(TARGETS
  body_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
