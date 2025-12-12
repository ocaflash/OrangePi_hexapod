cmake_minimum_required(VERSION 3.5)
project(crab_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  models
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
