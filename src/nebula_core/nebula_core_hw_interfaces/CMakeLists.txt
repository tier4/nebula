cmake_minimum_required(VERSION 3.14)
project(nebula_core_hw_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(nebula_core_hw_interfaces INTERFACE)

target_include_directories(nebula_core_hw_interfaces INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_core_hw_interfaces INTERFACE
    nebula_core_common::nebula_core_common
)

install(TARGETS nebula_core_hw_interfaces EXPORT export_nebula_core_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_udp
        tests/common/test_udp.cpp
    )

    target_include_directories(test_udp PRIVATE
        include
        tests
    )
    target_link_libraries(test_udp
        nebula_core_common::nebula_core_common
    )
endif()

ament_export_targets(export_nebula_core_hw_interfaces)
ament_export_dependencies(nebula_core_common)

ament_package()
