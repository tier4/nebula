cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

add_library(nebula_velodyne_common SHARED src/velodyne_calibration_decoder.cpp)

target_include_directories(
  nebula_velodyne_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_velodyne_common PUBLIC yaml-cpp)

install(TARGETS nebula_velodyne_common EXPORT export_nebula_velodyne_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_velodyne_common)
ament_export_dependencies(nebula_core_common yaml-cpp)

ament_package()
