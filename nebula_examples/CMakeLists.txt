cmake_minimum_required(VERSION 3.5)
project(nebula_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_ros REQUIRED)
find_package(nebula_common REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        include
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)
## HESAI
# Offline Lib
ament_auto_add_library(hesai_ros_offline_extract_pcd SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_pcd.cpp
)
target_link_libraries(hesai_ros_offline_extract_pcd ${PCL_LIBRARIES})
ament_auto_add_executable(hesai_ros_offline_extract_pcd_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_pcd_main.cpp
)
# Extraction for TEST Lib
ament_auto_add_library(hesai_ros_offline_extract_bag_pcd SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_bag_pcd.cpp
)
target_link_libraries(hesai_ros_offline_extract_bag_pcd ${PCL_LIBRARIES})
ament_auto_add_executable(hesai_ros_offline_extract_bag_pcd_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_ros_offline_extract_bag_pcd_main.cpp
)

## Velodyne
# Extraction for TEST Lib
ament_auto_add_library(velodyne_ros_offline_extract_bag_pcd SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_ros_offline_extract_bag_pcd.cpp
)
target_link_libraries(velodyne_ros_offline_extract_bag_pcd ${PCL_LIBRARIES})
ament_auto_add_executable(velodyne_ros_offline_extract_bag_pcd_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_ros_offline_extract_bag_pcd_main.cpp
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

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
