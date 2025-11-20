cmake_minimum_required(VERSION 3.20)
project(nebula_decoders_hesai)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_hesai REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(rclcpp REQUIRED)

# Hesai Decoder
add_library(nebula_decoders_hesai SHARED
    src/hesai_driver.cpp
)

target_include_directories(nebula_decoders_hesai PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_decoders_hesai PUBLIC
    nebula_common_base::nebula_common_base
    nebula_common_hesai::nebula_common_hesai
    nebula_decoders_base::nebula_decoders_base
    rclcpp::rclcpp
)

install(TARGETS nebula_decoders_hesai EXPORT export_nebula_decoders_hesai)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_decoders_hesai)

ament_export_dependencies(
    nebula_common_base
    nebula_common_hesai
    nebula_decoders_base
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
