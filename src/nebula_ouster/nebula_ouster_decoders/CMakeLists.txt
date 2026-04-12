cmake_minimum_required(VERSION 3.20)
project(nebula_ouster_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(_nebula_ouster_sdk "${CMAKE_CURRENT_SOURCE_DIR}/../../../ouster-sdk")
if(NOT EXISTS "${_nebula_ouster_sdk}/CMakeLists.txt")
  message(
    FATAL_ERROR
      "nebula_ouster_decoders: bundled ouster-sdk is missing (${_nebula_ouster_sdk})")
endif()

set(BUILD_SENSOR OFF CACHE BOOL "Ouster SDK: sensor module" FORCE)
set(BUILD_PCAP OFF CACHE BOOL "Ouster SDK: pcap" FORCE)
set(BUILD_OSF OFF CACHE BOOL "Ouster SDK: osf" FORCE)
set(BUILD_VIZ OFF CACHE BOOL "Ouster SDK: visualizer" FORCE)
set(BUILD_MAPPING OFF CACHE BOOL "Ouster SDK: mapping" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Ouster SDK: tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Ouster SDK: examples" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Ouster SDK: python" FORCE)

add_subdirectory("${_nebula_ouster_sdk}" "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build")

if(TARGET ouster_client)
  target_compile_options(ouster_client PRIVATE -Wno-pedantic)
endif()
if(TARGET nmea)
  target_compile_options(nmea PRIVATE -Wno-pedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(libzip REQUIRED)

# vcpkg / distro CMake config names differ; normalize to IMPORTED target @c libzip::zip .
if(NOT TARGET libzip::zip)
  if(TARGET libzip::libzip)
    add_library(libzip::zip ALIAS libzip::libzip)
  elseif(TARGET LibZip::libzip)
    add_library(libzip::zip ALIAS LibZip::libzip)
  endif()
endif()
if(NOT TARGET libzip::zip)
  message(FATAL_ERROR "nebula_ouster_decoders: libzip CMake target not found (expected libzip::zip)")
endif()

add_library(
  nebula_ouster_decoders SHARED
  src/ouster_decoder.cpp
  src/ouster_lidar_scan_conversions.cpp)

set(_ouster_gen_includes "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build/generated")
target_include_directories(
  nebula_ouster_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${_nebula_ouster_sdk}/ouster_client/include>
         $<BUILD_INTERFACE:${_ouster_gen_includes}>
         $<INSTALL_INTERFACE:include/nebula_vendor>)

target_compile_definitions(nebula_ouster_decoders PRIVATE EIGEN_MPL2_ONLY)

target_link_libraries(
  nebula_ouster_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_ouster_common::nebula_ouster_common
         nebula_core_decoders::nebula_core_decoders
         Eigen3::Eigen
  PRIVATE ouster_client
          OpenSSL::SSL
          OpenSSL::Crypto
          libzip::zip
          Threads::Threads)

install(
  DIRECTORY "${_nebula_ouster_sdk}/ouster_client/include/"
  DESTINATION include/nebula_vendor)
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build/generated/"
  DESTINATION include/nebula_vendor)

install(TARGETS nebula_ouster_decoders EXPORT export_nebula_ouster_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_ouster_decoders)

ament_export_dependencies(
  Eigen3
  nebula_core_common
  nebula_ouster_common
  nebula_core_decoders
  OpenSSL)

ament_package()
