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

  nebula_core_common
  nebula_robosense_common
  nebula_core_hw_interfaces
  nebula_msgs
  robosense_msgs)

ament_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_robosense_hw_interface
    test/test_robosense_hw_interface.cpp
  )
  target_include_directories(
    test_robosense_hw_interface
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(
    test_robosense_hw_interface
    nebula_robosense_hw_interfaces
  )
  ament_target_dependencies(
    test_robosense_hw_interface
    nebula_core_common
    nebula_robosense_common
    nebula_core_hw_interfaces
  )
endif()
