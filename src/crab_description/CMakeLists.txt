cmake_minimum_required(VERSION 3.5)
project(crab_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  models
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/demo_gait.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
