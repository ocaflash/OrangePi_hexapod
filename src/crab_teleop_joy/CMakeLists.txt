cmake_minimum_required(VERSION 3.5)
project(crab_teleop_joy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

add_executable(teleop_joy src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy
  rclcpp
  sensor_msgs
  std_msgs
  crab_msgs
)

install(TARGETS
  teleop_joy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
