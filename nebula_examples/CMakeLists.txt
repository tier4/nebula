cmake_minimum_required(VERSION 3.14)
project(nebula_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_common REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
    ${nebula_common_INCLUDE_DIRS}
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_ros_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rosbag2_cpp_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_TARGETS}
    ${nebula_ros_TARGETS}
    ${rosbag2_cpp_TARGETS}
    ${PCL_LIBRARIES}
)

## Hesai
# Offline Lib
add_library(hesai_ros_offline_extract_pcd SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_pcd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/parameter_descriptors.cpp
)
target_link_libraries(hesai_ros_offline_extract_pcd PUBLIC
    nebula_decoders::nebula_decoders_hesai
)

add_executable(hesai_ros_offline_extract_pcd_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_pcd_main.cpp
)

target_link_libraries(hesai_ros_offline_extract_pcd_node PUBLIC
    hesai_ros_offline_extract_pcd
)

# Extraction for TEST Lib
add_library(hesai_ros_offline_extract_bag_pcd SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_bag_pcd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/parameter_descriptors.cpp
)
target_link_libraries(hesai_ros_offline_extract_bag_pcd PUBLIC
    nebula_decoders::nebula_decoders_hesai
)

add_executable(hesai_ros_offline_extract_bag_pcd_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_bag_pcd_main.cpp
)

target_link_libraries(hesai_ros_offline_extract_bag_pcd_node PUBLIC
    hesai_ros_offline_extract_bag_pcd
)

## Velodyne
# Extraction for TEST Lib
add_library(velodyne_ros_offline_extract_bag_pcd SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_ros_offline_extract_bag_pcd.cpp
)
target_link_libraries(velodyne_ros_offline_extract_bag_pcd PUBLIC
    nebula_decoders::nebula_decoders_velodyne
)

add_executable(velodyne_ros_offline_extract_bag_pcd_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_ros_offline_extract_bag_pcd_main.cpp
)

target_link_libraries(velodyne_ros_offline_extract_bag_pcd_node PUBLIC
    velodyne_ros_offline_extract_bag_pcd
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS hesai_ros_offline_extract_pcd_node
                hesai_ros_offline_extract_bag_pcd_node
                velodyne_ros_offline_extract_bag_pcd_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    INSTALL_TO_SHARE
    launch
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
