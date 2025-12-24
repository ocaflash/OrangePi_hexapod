cmake_minimum_required(VERSION 3.5)
project(crab_imu)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

# Include directories
include_directories(
  include
  src/IMU
  src/sys
)

# IMU library
add_library(imu
  src/IMU/Compass.cpp
  src/IMU/DCM.cpp
  src/IMU/IMU.cpp
  src/IMU/L3G.cpp
  src/IMU/LSM303.cpp
  src/IMU/Matrix.cpp
  src/IMU/Vector.cpp
)

ament_target_dependencies(imu rclcpp sensor_msgs std_msgs)

# IMU control node
add_executable(imu_control
  src/imu_control.cpp
  src/sys/Timer.cpp
  src/sys/I2C.cpp
)

target_link_libraries(imu_control imu)
ament_target_dependencies(imu_control rclcpp sensor_msgs std_msgs crab_msgs)

# Installation
install(TARGETS
  imu
  imu_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
