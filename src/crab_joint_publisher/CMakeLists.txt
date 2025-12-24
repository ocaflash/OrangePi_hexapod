cmake_minimum_required(VERSION 3.5)
project(crab_joint_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher
  rclcpp
  sensor_msgs
  std_msgs
  crab_msgs
)

install(TARGETS
  joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
