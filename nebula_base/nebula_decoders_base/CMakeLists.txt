cmake_minimum_required(VERSION 3.14)
project(nebula_decoders_base)

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
find_package(nebula_common_base REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PNG REQUIRED)

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
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_CONVERSIONS_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${PCL_LIBRARIES}
    ${PCL_CONVERSIONS_TARGET}
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    add_definitions(-D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")

    ament_add_gtest(test_downsample_mask tests/point_filters/test_downsample_mask.cpp)
    target_link_libraries(test_downsample_mask
        ${PNG_LIBRARIES})
    target_include_directories(test_downsample_mask PUBLIC
        include
        ${PNG_INCLUDE_DIRS})
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
    PCL
    pcl_conversions
    angles
    nebula_common_base
    nebula_msgs
    rclcpp
    sensor_msgs
    yaml-cpp
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
