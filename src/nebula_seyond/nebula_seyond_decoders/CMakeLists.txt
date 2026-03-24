cmake_minimum_required(VERSION 3.20)
project(nebula_seyond_decoders)

find_package(autoware_cmake REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nebula_seyond_common REQUIRED)
autoware_package()

add_library(nebula_seyond_decoders SHARED src/seyond_decoder.cpp)

target_include_directories(
  nebula_seyond_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_seyond_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_seyond_common::nebula_seyond_common
         nebula_core_decoders::nebula_core_decoders)

install(TARGETS nebula_seyond_decoders EXPORT export_nebula_seyond_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_seyond_decoders)
ament_export_dependencies(nebula_core_common nebula_seyond_common nebula_core_decoders)

ament_package()
