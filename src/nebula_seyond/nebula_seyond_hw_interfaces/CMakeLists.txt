cmake_minimum_required(VERSION 3.20)
project(nebula_seyond_hw_interfaces)

find_package(autoware_cmake REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_hw_interfaces REQUIRED)
find_package(nebula_seyond_common REQUIRED)
autoware_package()

add_library(nebula_seyond_hw_interfaces SHARED src/seyond_hw_interface.cpp)

target_include_directories(
  nebula_seyond_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_seyond_hw_interfaces
  PUBLIC nebula_core_common::nebula_core_common
         nebula_seyond_common::nebula_seyond_common
         nebula_core_hw_interfaces::nebula_core_hw_interfaces)

install(TARGETS nebula_seyond_hw_interfaces
        EXPORT export_nebula_seyond_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_seyond_hw_interfaces)
ament_export_dependencies(nebula_core_common nebula_seyond_common nebula_core_hw_interfaces)

ament_package()
