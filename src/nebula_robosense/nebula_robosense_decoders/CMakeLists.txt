cmake_minimum_required(VERSION 3.20)
project(nebula_robosense_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Robosense Decoder
add_library(nebula_robosense_decoders SHARED src/robosense_driver.cpp)

target_include_directories(
  nebula_robosense_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_robosense_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_robosense_common::nebula_robosense_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp)

# Robosense Info Decoder
add_library(nebula_robosense_decoders_info SHARED src/robosense_info_driver.cpp)

target_include_directories(
  nebula_robosense_decoders_info
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_robosense_decoders_info
  PUBLIC nebula_core_common::nebula_core_common
         nebula_robosense_common::nebula_robosense_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp)

install(TARGETS nebula_robosense_decoders
        EXPORT export_nebula_robosense_decoders)
install(TARGETS nebula_robosense_decoders_info
        EXPORT export_nebula_robosense_decoders_info)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_robosense_decoders)
ament_export_targets(export_nebula_robosense_decoders_info)

ament_export_dependencies(nebula_core_common nebula_robosense_common
                          nebula_core_decoders rclcpp)

ament_package()
