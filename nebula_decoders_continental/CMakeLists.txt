cmake_minimum_required(VERSION 3.14)
project(nebula_decoders_continental)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(boost_udp_driver REQUIRED)
find_package(continental_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_continental REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

# Check if pcl_conversions provides modern CMake targets (ROS 2 Jazzy and later)
# or if we need to use legacy variables (ROS 2 Humble and earlier)
if(TARGET pcl_conversions::pcl_conversions)
    set(PCL_CONVERSIONS_TARGET pcl_conversions::pcl_conversions)
    set(PCL_CONVERSIONS_INCLUDE_DIRS "")
else()
    set(PCL_CONVERSIONS_TARGET "")
    set(PCL_CONVERSIONS_INCLUDE_DIRS ${pcl_conversions_INCLUDE_DIRS})
endif()

include_directories(PUBLIC
    include
    SYSTEM
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_common_continental_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_CONVERSIONS_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${nebula_common_continental_TARGETS}
    ${nebula_decoders_base_TARGETS}
    ${PCL_LIBRARIES}
    ${PCL_CONVERSIONS_TARGET}
    ${sensor_msgs_TARGETS}
)

# Continental Decoder
add_library(nebula_decoders_continental SHARED
    src/decoders/continental_ars548_decoder.cpp
    src/decoders/continental_srr520_decoder.cpp
)
target_link_libraries(nebula_decoders_continental PUBLIC
    ${continental_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${boost_udp_driver_TARGETS}
    ${nebula_common_base_TARGETS}
    ${nebula_msgs_TARGETS}
)
target_include_directories(nebula_decoders_continental PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${boost_udp_driver_INCLUDE_DIRS}
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
)

install(TARGETS nebula_decoders_continental EXPORT export_nebula_decoders_continental)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_decoders_continental)

ament_export_dependencies(
    PCL
    pcl_conversions
    boost_udp_driver
    continental_msgs
    diagnostic_msgs
    nebula_common_base
    nebula_common_continental
    nebula_decoders_base
    nebula_msgs
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
