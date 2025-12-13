cmake_minimum_required(VERSION 3.5)
project(crab_teleop_joy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

# Teleop joy node
add_executable(teleop_joy src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy
  rclcpp
  sensor_msgs
  std_msgs
  crab_msgs
)

# DS4 IMU publisher node
add_executable(ds4_imu_publisher src/ds4_imu_publisher.cpp)
ament_target_dependencies(ds4_imu_publisher
  rclcpp
  sensor_msgs
)

install(TARGETS
  teleop_joy
  ds4_imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
