cmake_minimum_required(VERSION 3.20)
project(nebula_core_common)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nlohmann_json REQUIRED)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

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
        tests/test_rate_checker.cpp
    )
    target_link_libraries(test_rate_checker
        nebula_core_common
    )

    ament_add_gtest(test_ring_buffer
        tests/test_ring_buffer.cpp
    )
    target_link_libraries(test_ring_buffer
        nebula_core_common
    )

    ament_add_gtest(test_bitfield
        tests/test_bitfield.cpp
    )
    target_link_libraries(test_bitfield
        nebula_core_common
    )

    ament_add_gtest(test_expected
        tests/test_expected.cpp
    )
    target_link_libraries(test_expected
        nebula_core_common
    )

    ament_add_gtest(test_rate_limiter
        tests/test_rate_limiter.cpp
    )
    target_link_libraries(test_rate_limiter
        nebula_core_common
    )

    ament_add_gtest(test_string_conversions
        tests/test_string_conversions.cpp
    )
    target_link_libraries(test_string_conversions
        nebula_core_common
    )

    ament_add_gtest(test_nebula_common
        tests/test_nebula_common.cpp
    )
    target_link_libraries(test_nebula_common
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


# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
