cmake_minimum_required(VERSION 3.20)
project(nebula_decoders_base)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PNG REQUIRED)

add_library(nebula_decoders_base INTERFACE)

target_include_directories(nebula_decoders_base INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nebula_decoders_base INTERFACE
    nebula_common_base
    sensor_msgs
)

install(TARGETS nebula_decoders_base EXPORT export_nebula_decoders_base)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    add_definitions(-D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")

    ament_add_gtest(test_downsample_mask tests/point_filters/test_downsample_mask.cpp)
    target_include_directories(test_downsample_mask PRIVATE
        include
    )
    target_link_libraries(test_downsample_mask
        PNG::PNG
        nebula_decoders_base
    )
endif()

ament_export_targets(export_nebula_decoders_base)
ament_export_dependencies(
    nebula_common_base
    sensor_msgs
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
