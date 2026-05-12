cmake_minimum_required(VERSION 3.20)
project(nebula_continental_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Continental Decoder
add_library(
  nebula_continental_decoders SHARED
  src/decoders/continental_ars548_decoder.cpp
  src/decoders/continental_srr520_decoder.cpp)

target_include_directories(
  nebula_continental_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_continental_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_continental_common::nebula_continental_common rclcpp::rclcpp)

ament_target_dependencies(nebula_continental_decoders PUBLIC continental_msgs
                          diagnostic_msgs nebula_msgs)

install(TARGETS nebula_continental_decoders
        EXPORT export_nebula_continental_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_continental_decoders)

ament_export_dependencies(continental_msgs diagnostic_msgs nebula_core_common
                          nebula_continental_common nebula_msgs rclcpp)

ament_package()
