cmake_minimum_required(VERSION 3.20)
project(nebula_sample_common)
# cspell: ignore CPACK ARGN

if(DEFINED ENV{AMENT_PREFIX_PATH})
  find_package(autoware_cmake REQUIRED)
endif()

find_package(nlohmann_json REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)

add_library(nebula_sample_common INTERFACE)

target_include_directories(
  nebula_sample_common
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_sample_common
                      INTERFACE nebula_core_common::nebula_core_common
                                nebula_core_decoders::nebula_core_decoders
                                nlohmann_json::nlohmann_json)

install(TARGETS nebula_sample_common EXPORT export_nebula_sample_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY schemas DESTINATION share/${PROJECT_NAME})
install(FILES nebula_sample_plugin.json DESTINATION share/${PROJECT_NAME})

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

install(EXPORT export_nebula_sample_common
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
  ament_export_targets(export_nebula_sample_common)
  ament_export_dependencies(nebula_core_common nebula_core_decoders nlohmann_json)
  autoware_package()
  ament_package()
endif()
