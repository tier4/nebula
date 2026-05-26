cmake_minimum_required(VERSION 3.20)
project(nebula_sample_decoders)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  find_package(autoware_cmake REQUIRED)
endif()

find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nebula_sample_common REQUIRED)

# Sample Decoder
add_library(nebula_sample_decoders SHARED src/sample_decoder.cpp)

target_include_directories(
  nebula_sample_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_sample_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_sample_common::nebula_sample_common
         nebula_core_decoders::nebula_core_decoders)

# Sample Plugin Library
add_library(nebula_sample_decoders_plugin SHARED src/sample_plugin.cpp)
target_link_libraries(nebula_sample_decoders_plugin PUBLIC nebula_sample_decoders)

install(TARGETS nebula_sample_decoders nebula_sample_decoders_plugin EXPORT export_nebula_sample_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
set(CPACK_PACKAGE_VERSION "1.0.0")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${CPACK_PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT export_nebula_sample_decoders
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  ament_export_include_directories("include/${PROJECT_NAME}")
  ament_export_targets(export_nebula_sample_decoders)
  ament_export_dependencies(nebula_core_common nebula_sample_common nebula_core_decoders)
  autoware_package()
  ament_package()
endif()
