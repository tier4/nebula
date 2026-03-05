cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne_hw_interfaces)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_velodyne_hw_interfaces SHARED src/velodyne_hw_interface.cpp)

target_include_directories(
  nebula_velodyne_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  nebula_velodyne_hw_interfaces
  PUBLIC

  nebula_core_common
  nebula_velodyne_common
  nebula_core_hw_interfaces)

install(TARGETS nebula_velodyne_hw_interfaces
        EXPORT export_nebula_velodyne_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_velodyne_hw_interfaces)
ament_export_dependencies(nebula_core_common
                          nebula_velodyne_common nebula_core_hw_interfaces)

ament_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_velodyne_hw_interface
    test/test_velodyne_hw_interface.cpp
  )
  target_include_directories(
    test_velodyne_hw_interface
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(
    test_velodyne_hw_interface
    nebula_velodyne_hw_interfaces
  )
  ament_target_dependencies(
    test_velodyne_hw_interface
    nebula_core_common
    nebula_velodyne_common
    nebula_core_hw_interfaces
  )
endif()
