cmake_minimum_required(VERSION 3.20)
project(nebula_seyond_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_seyond_common INTERFACE)

target_include_directories(
  nebula_seyond_common
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_seyond_common
                      INTERFACE nebula_core_common::nebula_core_common
                                nebula_core_decoders::nebula_core_decoders)

install(TARGETS nebula_seyond_common
        EXPORT export_nebula_seyond_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_nebula_seyond_common)
ament_export_dependencies(nebula_core_common nebula_core_decoders nlohmann_json)

ament_package()
