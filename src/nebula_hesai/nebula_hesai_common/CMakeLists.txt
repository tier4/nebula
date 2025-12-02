cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_hesai_common INTERFACE)

target_include_directories(nebula_hesai_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(nebula_hesai_common INTERFACE
    nebula_core_common::nebula_core_common
)

install(TARGETS nebula_hesai_common EXPORT export_nebula_hesai_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_nebula_hesai_common)
ament_export_dependencies(nebula_core_common)

ament_package()
