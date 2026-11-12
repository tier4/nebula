cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_decoders)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
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
         nebula_core_decoders::nebula_core_decoders
         Boost::headers)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_hesai_decoder_configuration
                  tests/test_hesai_decoder_configuration.cpp)
  target_link_libraries(test_hesai_decoder_configuration nebula_hesai_decoders)
endif()

install(TARGETS nebula_hesai_decoders EXPORT export_nebula_hesai_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY calibration DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hesai_decoders)

ament_export_dependencies(Boost nebula_core_common nebula_hesai_common
                          nebula_core_decoders)

ament_package()
