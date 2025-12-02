cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Hesai Decoder
add_library(nebula_hesai_decoders SHARED src/hesai_driver.cpp)

target_include_directories(
  nebula_hesai_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_hesai_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_hesai_common::nebula_hesai_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp)

install(TARGETS nebula_hesai_decoders EXPORT export_nebula_hesai_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY calibration DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hesai_decoders)

ament_export_dependencies(nebula_core_common nebula_hesai_common
                          nebula_core_decoders rclcpp)

ament_package()
