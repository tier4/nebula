cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne_decoders)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(angles REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_velodyne_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(velodyne_msgs REQUIRED)

# Velodyne Decoder
add_library(nebula_velodyne_decoders SHARED
    src/velodyne_driver.cpp
    src/decoders/vls128_decoder.cpp
    src/decoders/vlp16_decoder.cpp
    src/decoders/vlp32_decoder.cpp
)

target_include_directories(nebula_velodyne_decoders PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nebula_velodyne_decoders PUBLIC
    angles
    nebula_core_common
    nebula_velodyne_common
    nebula_core_decoders
    pcl_conversions
    rclcpp
    velodyne_msgs
)

target_link_libraries(nebula_velodyne_decoders PUBLIC
    pcl_common
)

install(TARGETS nebula_velodyne_decoders EXPORT export_nebula_velodyne_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_velodyne_decoders)

ament_export_dependencies(
    PCL
    pcl_conversions
    angles
    nebula_core_common
    nebula_velodyne_common
    nebula_core_decoders
    velodyne_msgs
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
