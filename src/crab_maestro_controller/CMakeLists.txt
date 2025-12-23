cmake_minimum_required(VERSION 3.5)
project(crab_maestro_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

add_library(polstro
  src/PolstroSerialInterface.cpp
  src/PolstroSerialInterfacePOSIX.cpp
)

ament_target_dependencies(polstro
  rclcpp
)

add_executable(maestro_joint_sub src/maestro_joint_sub.cpp)
target_link_libraries(maestro_joint_sub polstro)
ament_target_dependencies(maestro_joint_sub
  rclcpp
  sensor_msgs
)

add_executable(maestro_leg_sub src/maestro_leg_sub.cpp)
target_link_libraries(maestro_leg_sub polstro)
ament_target_dependencies(maestro_leg_sub
  rclcpp
  crab_msgs
)

add_executable(controller_sub src/controller_sub.cpp)
target_link_libraries(controller_sub polstro)
ament_target_dependencies(controller_sub
  rclcpp
  crab_msgs
)

install(TARGETS
  polstro
  maestro_joint_sub
  maestro_leg_sub
  controller_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
