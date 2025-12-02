cmake_minimum_required(VERSION 3.20)
project(nebula_robosense_hw_interfaces)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_robosense_hw_interfaces SHARED
            src/robosense_hw_interface.cpp)

target_include_directories(
  nebula_robosense_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  nebula_robosense_hw_interfaces
  PUBLIC
  boost_tcp_driver
  boost_udp_driver
  robosense_msgs
  nebula_core_common
  nebula_robosense_common
  nebula_core_hw_interfaces)

install(TARGETS nebula_robosense_hw_interfaces
        EXPORT export_nebula_robosense_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_robosense_hw_interfaces)
ament_export_dependencies(
  boost_tcp_driver
  boost_udp_driver
  nebula_core_common
  nebula_robosense_common
  nebula_core_hw_interfaces
  nebula_msgs
  robosense_msgs)

ament_package()
