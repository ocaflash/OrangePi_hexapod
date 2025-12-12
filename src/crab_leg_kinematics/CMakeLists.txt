cmake_minimum_required(VERSION 3.5)
project(crab_leg_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(crab_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# IK solver library
add_library(leg_ik_solver
  src/hp_chainiksolverpos_nr_jl.cpp
  src/chainiksolvervel_pinv.cpp
  src/svd_HH.cpp
)
target_include_directories(leg_ik_solver PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(leg_ik_solver rclcpp orocos_kdl_vendor)

# Leg IK service node
add_executable(leg_ik_service src/leg_ik_service.cpp)
target_link_libraries(leg_ik_service leg_ik_solver)
ament_target_dependencies(leg_ik_service
  rclcpp
  std_msgs
  kdl_parser
  orocos_kdl_vendor
  crab_msgs
)

install(TARGETS
  leg_ik_solver
  leg_ik_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
