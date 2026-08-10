cmake_minimum_required(VERSION 3.20)
project(nebula_robosense_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_robosense_common INTERFACE)

target_include_directories(
  nebula_robosense_common
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_robosense_common
                      INTERFACE nebula_core_common::nebula_core_common)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_robosense_common test/test_robosense_common.cpp)
  target_link_libraries(test_robosense_common nebula_robosense_common)
endif()

install(TARGETS nebula_robosense_common EXPORT export_nebula_robosense_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_nebula_robosense_common)
ament_export_dependencies(nebula_core_common)

ament_package()
