cmake_minimum_required(VERSION 3.14)
project(nebula_decoders)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nebula_common REQUIRED)

include_directories(
        include
        SYSTEM
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)

# Lidar Decoders
# Hesai
ament_auto_add_library(nebula_decoders_hesai SHARED
        src/nebula_decoders_hesai/hesai_driver.cpp
        )

# Velodyne
ament_auto_add_library(nebula_decoders_velodyne SHARED
        src/nebula_decoders_velodyne/velodyne_driver.cpp
        src/nebula_decoders_velodyne/decoders/vls128_decoder.cpp
        src/nebula_decoders_velodyne/decoders/vlp16_decoder.cpp
        src/nebula_decoders_velodyne/decoders/vlp32_decoder.cpp
        )

# Robosense
ament_auto_add_library(nebula_decoders_robosense SHARED
        src/nebula_decoders_robosense/robosense_driver.cpp
        )

ament_auto_add_library(nebula_decoders_robosense_info SHARED
        src/nebula_decoders_robosense/robosense_info_driver.cpp
        )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        calibration
        )

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
