cmake_minimum_required(VERSION 3.20)
project(nebula_continental_decoders)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(continental_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_continental_common REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)

# Continental Decoder
add_library(nebula_continental_decoders SHARED
    src/decoders/continental_ars548_decoder.cpp
    src/decoders/continental_srr520_decoder.cpp
)

target_include_directories(nebula_continental_decoders PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_continental_decoders PUBLIC
    nebula_core_common::nebula_core_common
    nebula_continental_common::nebula_continental_common
    rclcpp::rclcpp
)

ament_target_dependencies(
    nebula_continental_decoders
    PUBLIC
    continental_msgs
    diagnostic_msgs
    nebula_msgs
)

install(TARGETS nebula_continental_decoders EXPORT export_nebula_continental_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_continental_decoders)

ament_export_dependencies(
    continental_msgs
    diagnostic_msgs
    nebula_core_common
    nebula_continental_common
    nebula_msgs
    rclcpp
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
