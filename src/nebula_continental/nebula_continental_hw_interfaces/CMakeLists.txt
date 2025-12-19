cmake_minimum_required(VERSION 3.20)
project(nebula_continental_hw_interfaces)

find_package(autoware_cmake REQUIRED)
find_package(rclcpp REQUIRED)
autoware_package()

add_library(
  nebula_continental_hw_interfaces SHARED
  src/continental_ars548_hw_interface.cpp
  src/continental_srr520_hw_interface.cpp)

target_include_directories(
  nebula_continental_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  nebula_continental_hw_interfaces
  PUBLIC

  nebula_msgs
  rclcpp

  nebula_core_common
  nebula_continental_common
  nebula_core_hw_interfaces)

install(TARGETS nebula_continental_hw_interfaces
        EXPORT export_nebula_continental_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_continental_hw_interfaces)
ament_export_dependencies(
  nebula_core_hw_interfaces nebula_msgs)

ament_package()
