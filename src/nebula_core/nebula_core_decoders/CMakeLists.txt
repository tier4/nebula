cmake_minimum_required(VERSION 3.20)
project(nebula_core_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_core_decoders INTERFACE)

target_include_directories(
  nebula_core_decoders
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_core_decoders INTERFACE PNG::PNG)
ament_target_dependencies(nebula_core_decoders INTERFACE nebula_core_common
                          sensor_msgs)

install(TARGETS nebula_core_decoders EXPORT export_nebula_core_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_definitions(
    -D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")

  ament_add_gtest(test_downsample_mask
                  tests/point_filters/test_downsample_mask.cpp)
  target_include_directories(test_downsample_mask PRIVATE include)
  target_link_libraries(test_downsample_mask nebula_core_decoders)

  ament_add_gtest(test_angles tests/test_angles.cpp)
  target_include_directories(test_angles PRIVATE include)
  target_link_libraries(test_angles nebula_core_decoders)
endif()

ament_export_targets(export_nebula_core_decoders)
ament_export_dependencies(nebula_core_common sensor_msgs PNG)

ament_package()
