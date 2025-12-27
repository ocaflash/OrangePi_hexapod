cmake_minimum_required(VERSION 3.5)
project(maestro_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crab_msgs REQUIRED)

# Polstro library
add_library(polstro
  src/PolstroSerialInterface.cpp
  src/PolstroSerialInterfacePOSIX.cpp
)

# Servo controller node
add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node polstro)
ament_target_dependencies(servo_node rclcpp std_msgs crab_msgs)

# Servo test node (для диагностики)
add_executable(servo_test_node src/servo_test_node.cpp)
target_link_libraries(servo_test_node polstro)
ament_target_dependencies(servo_test_node rclcpp)

install(TARGETS
  polstro
  servo_node
  servo_test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
