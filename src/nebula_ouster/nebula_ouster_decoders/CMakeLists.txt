cmake_minimum_required(VERSION 3.20)
project(nebula_ouster_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(nlohmann_json REQUIRED)

add_library(
  nebula_ouster_decoders SHARED
  src/ouster_decoder.cpp
  src/ouster_metadata.cpp)

target_include_directories(
  nebula_ouster_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_ouster_decoders
  PUBLIC nlohmann_json::nlohmann_json)

ament_target_dependencies(
  nebula_ouster_decoders
  PUBLIC
  nebula_core_common
  nebula_core_decoders
  nebula_core_hw_interfaces
  nebula_ouster_common)

install(TARGETS nebula_ouster_decoders EXPORT export_nebula_ouster_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_ouster_decoders)

ament_export_dependencies(
  nebula_core_common
  nebula_core_decoders
  nebula_core_hw_interfaces
  nebula_ouster_common
  nlohmann_json)

ament_package()
