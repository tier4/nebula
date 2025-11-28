cmake_minimum_required(VERSION 3.20)
project(nebula_robosense_decoders)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_robosense_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)

# Robosense Decoder
add_library(nebula_robosense_decoders SHARED
    src/robosense_driver.cpp
)

target_include_directories(nebula_robosense_decoders PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_robosense_decoders PUBLIC
    nebula_core_common::nebula_core_common
    nebula_robosense_common::nebula_robosense_common
    nebula_core_decoders::nebula_core_decoders
    rclcpp::rclcpp
)

# Robosense Info Decoder
add_library(nebula_robosense_decoders_info SHARED
    src/robosense_info_driver.cpp
)

target_include_directories(nebula_robosense_decoders_info PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_robosense_decoders_info PUBLIC
    nebula_core_common::nebula_core_common
    nebula_robosense_common::nebula_robosense_common
    nebula_core_decoders::nebula_core_decoders
    rclcpp::rclcpp
)

install(TARGETS nebula_robosense_decoders EXPORT export_nebula_robosense_decoders)
install(TARGETS nebula_robosense_decoders_info EXPORT export_nebula_robosense_decoders_info)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_robosense_decoders)
ament_export_targets(export_nebula_robosense_decoders_info)

ament_export_dependencies(
    nebula_core_common
    nebula_robosense_common
    nebula_core_decoders
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
