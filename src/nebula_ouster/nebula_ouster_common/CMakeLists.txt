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

  if(NOT TARGET ouster_core)
    add_subdirectory("${_nebula_ouster_sdk}" "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build")
  endif()

  # Restore BUILD_TESTING after ouster-sdk
  set(BUILD_TESTING ${_PARENT_BUILD_TESTING} CACHE BOOL "" FORCE)

  # ouster-sdk has few C++ warnings which are treated as errors by the CI. Disable them for now.
  target_compile_options(ouster_core PRIVATE -Wno-pedantic)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # cspell:ignore stringop memaccess
    target_compile_options(ouster_core PRIVATE -Wno-error=stringop-overflow -Wno-class-memaccess)
  endif()

  ament_add_gtest(
    test_ouster_calibration_data
    test/test_ouster_calibration_data.cpp
  )
  target_link_libraries(
    test_ouster_calibration_data
    nebula_ouster_common
    nebula_core_common::nebula_core_common
    ouster_core
  )
  target_include_directories(
    test_ouster_calibration_data
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${_nebula_ouster_sdk}/ouster_core/include
            ${_nebula_ouster_sdk}/ouster_core/include/optional-lite
  )
endif()

ament_package()
