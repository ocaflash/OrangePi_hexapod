cmake_minimum_required(VERSION 3.5)
project(crab_imu)

# Найти зависимости ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Пути к заголовочным файлам
include_directories(
  include
  src/IMU
  src/sys
)

# Библиотека IMU
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

# Узел управления IMU
add_executable(imu_control
  src/imu_control.cpp
  src/sys/Timer.cpp
  src/sys/I2C.cpp
)

target_link_libraries(imu_control imu)
ament_target_dependencies(imu_control rclcpp sensor_msgs std_msgs)

# Установка
install(TARGETS
  imu
  imu_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Завершение
ament_package()
