cmake_minimum_required(VERSION 3.14)
project(nebula_decoders)

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
find_package(continental_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nebula_common REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pandar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PNG REQUIRED)

include_directories(PUBLIC
    include
    SYSTEM
    ${nebula_common_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_TARGETS}
    ${PCL_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
)

# Lidar Decoders
# Hesai
add_library(nebula_decoders_hesai SHARED
    src/nebula_decoders_hesai/hesai_driver.cpp
)
target_link_libraries(nebula_decoders_hesai PUBLIC
    ${pandar_msgs_TARGETS}
    ${PNG_LIBRARIES}
)

target_include_directories(nebula_decoders_hesai PUBLIC
    ${pandar_msgs_INCLUDE_DIRS}
    ${PNG_INCLUDE_DIRS}
)

# Velodyne
add_library(nebula_decoders_velodyne SHARED
    src/nebula_decoders_velodyne/velodyne_driver.cpp
    src/nebula_decoders_velodyne/decoders/vls128_decoder.cpp
    src/nebula_decoders_velodyne/decoders/vlp16_decoder.cpp
    src/nebula_decoders_velodyne/decoders/vlp32_decoder.cpp
)
target_link_libraries(nebula_decoders_velodyne PUBLIC
    ${angles_msgs_TARGETS}
    ${velodyne_msgs_TARGETS}
)
target_include_directories(nebula_decoders_velodyne PUBLIC
    ${angles_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

# Robosense
add_library(nebula_decoders_robosense SHARED
    src/nebula_decoders_robosense/robosense_driver.cpp
)
target_link_libraries(nebula_decoders_robosense PUBLIC
    ${robosense_msgs_TARGETS}
)
target_include_directories(nebula_decoders_robosense PUBLIC
    ${robosense_msgs_INCLUDE_DIRS}
)

add_library(nebula_decoders_robosense_info SHARED
    src/nebula_decoders_robosense/robosense_info_driver.cpp
)
target_link_libraries(nebula_decoders_robosense_info PUBLIC
    ${robosense_msgs_TARGETS}
)
target_include_directories(nebula_decoders_robosense_info PUBLIC
    ${robosense_msgs_INCLUDE_DIRS}
)

# Continental
add_library(nebula_decoders_continental SHARED
    src/nebula_decoders_continental/decoders/continental_ars548_decoder.cpp
    src/nebula_decoders_continental/decoders/continental_srr520_decoder.cpp
)
target_link_libraries(nebula_decoders_continental PUBLIC
    ${continental_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${boost_udp_driver_TARGETS}
    ${nebula_common_TARGETS}
    ${nebula_msgs_TARGETS}
)
target_include_directories(nebula_decoders_continental PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${boost_udp_driver_INCLUDE_DIRS}
    ${nebula_common_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
)

install(TARGETS nebula_decoders_hesai EXPORT export_nebula_decoders_hesai)
install(TARGETS nebula_decoders_velodyne EXPORT export_nebula_decoders_velodyne)
install(TARGETS nebula_decoders_robosense EXPORT export_nebula_decoders_robosense)
install(TARGETS nebula_decoders_robosense_info EXPORT export_nebula_decoders_robosense_info)
install(TARGETS nebula_decoders_continental EXPORT export_nebula_decoders_continental)
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
ament_export_targets(export_nebula_decoders_hesai)
ament_export_targets(export_nebula_decoders_velodyne)
ament_export_targets(export_nebula_decoders_robosense)
ament_export_targets(export_nebula_decoders_robosense_info)
ament_export_targets(export_nebula_decoders_continental)

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    PCL
    pcl_conversions
    angles
    continental_msgs
    diagnostic_msgs
    nebula_common
    nebula_msgs
    pandar_msgs
    rclcpp
    robosense_msgs
    sensor_msgs
    velodyne_msgs
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
