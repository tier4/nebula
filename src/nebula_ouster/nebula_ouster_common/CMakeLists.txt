cmake_minimum_required(VERSION 3.20)
project(nebula_ouster_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_ouster_common INTERFACE)

target_include_directories(
  nebula_ouster_common
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_ouster_common
                      INTERFACE nebula_core_common::nebula_core_common
                                nebula_core_decoders::nebula_core_decoders)

install(TARGETS nebula_ouster_common EXPORT export_nebula_ouster_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_nebula_ouster_common)
ament_export_dependencies(nebula_core_common nebula_core_decoders)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(_nebula_ouster_sdk "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/ouster-sdk")

  set(BUILD_SENSOR OFF CACHE BOOL "" FORCE)
  set(BUILD_PCAP OFF CACHE BOOL "" FORCE)
  set(BUILD_OSF OFF CACHE BOOL "" FORCE)
  set(BUILD_VIZ OFF CACHE BOOL "" FORCE)
  set(BUILD_MAPPING OFF CACHE BOOL "" FORCE)
  set(_PARENT_BUILD_TESTING ${BUILD_TESTING})
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)

  if(NOT TARGET ouster_client)
    add_subdirectory("${_nebula_ouster_sdk}" "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build")
  endif()

  # Restore BUILD_TESTING after ouster-sdk
  set(BUILD_TESTING ${_PARENT_BUILD_TESTING} CACHE BOOL "" FORCE)

  set(_ouster_gen_includes "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build/generated")

  ament_add_gtest(
    test_ouster_calibration_data
    test/test_ouster_calibration_data.cpp
  )
  target_link_libraries(
    test_ouster_calibration_data
    nebula_ouster_common
    nebula_core_common::nebula_core_common
    ouster_client
  )
  target_include_directories(
    test_ouster_calibration_data
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${_nebula_ouster_sdk}/ouster_client/include
            ${_nebula_ouster_sdk}/ouster_client/include/optional-lite
            ${_ouster_gen_includes}
  )
endif()

ament_package()
