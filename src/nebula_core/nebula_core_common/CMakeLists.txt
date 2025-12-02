cmake_minimum_required(VERSION 3.20)
project(nebula_core_common)

find_package(autoware_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nlohmann_json REQUIRED)
autoware_package()

add_library(nebula_core_common SHARED
    src/nebula_common.cpp
)

target_include_directories(nebula_core_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_core_common PUBLIC
    pcl_common
    nlohmann_json::nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_rate_checker
        test/test_rate_checker.cpp
    )
  target_link_libraries(test_rate_checker
        nebula_core_common
    )
endif()

install(TARGETS nebula_core_common EXPORT export_nebula_core_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_core_common)

ament_export_dependencies(
    PCL
    nlohmann_json
)

ament_package()
