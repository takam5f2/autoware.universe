cmake_minimum_required(VERSION 3.22)
project(kinematic_reader)

find_package(autoware_cmake REQUIRED)
autoware_package()

# planning validator
ament_auto_add_library(kinematic_reader_component SHARED
  include/kinematic_reader/kinematic_reader.hpp
  src/kinematic_reader.cpp
)

target_link_libraries(kinematic_reader_component)

rclcpp_components_register_node(kinematic_reader_component
  PLUGIN "kinematic_reader::KinematicReader"
  EXECUTABLE kinematic_reader_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch)
