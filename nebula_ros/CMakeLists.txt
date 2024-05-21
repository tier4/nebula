cmake_minimum_required(VERSION 3.14)
project(nebula_ros)

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
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_common REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_hw_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pandar_msgs REQUIRED)

include_directories(
        include
        SYSTEM
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)
link_libraries(${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})
## Hesai
ament_auto_add_library(hesai_ros_wrapper SHARED
        src/hesai/hesai_ros_wrapper.cpp
        src/hesai/decoder_wrapper.cpp
        src/hesai/hw_interface_wrapper.cpp
        src/hesai/hw_monitor_wrapper.cpp
        src/common/parameter_descriptors.cpp
        )

rclcpp_components_register_node(hesai_ros_wrapper
        PLUGIN "HesaiRosWrapper"
        EXECUTABLE hesai_ros_wrapper_node
        )

## Velodyne
ament_auto_add_library(velodyne_ros_wrapper SHARED
        src/velodyne/velodyne_ros_wrapper.cpp
        src/velodyne/decoder_wrapper.cpp
        src/velodyne/hw_interface_wrapper.cpp
        src/velodyne/hw_monitor_wrapper.cpp
        src/common/parameter_descriptors.cpp
        )

rclcpp_components_register_node(velodyne_ros_wrapper
        PLUGIN "VelodyneRosWrapper"
        EXECUTABLE velodyne_ros_wrapper_node
        )

## Robosense
# Hw Interface
ament_auto_add_library(robosense_ros_wrapper SHARED
        src/robosense/robosense_ros_wrapper.cpp
        src/robosense/decoder_wrapper.cpp
        src/robosense/hw_interface_wrapper.cpp
        src/robosense/hw_monitor_wrapper.cpp
        src/common/parameter_descriptors.cpp
        )

rclcpp_components_register_node(robosense_ros_wrapper
        PLUGIN "RobosenseRosWrapper"
        EXECUTABLE robosense_ros_wrapper_node
        )

## Continental
ament_auto_add_library(continental_ars548_ros_wrapper SHARED
        src/continental/continental_ars548_ros_wrapper.cpp
        src/continental/continental_ars548_decoder_wrapper.cpp
        src/continental/continental_ars548_hw_interface_wrapper.cpp
        src/common/parameter_descriptors.cpp
        )

rclcpp_components_register_node(continental_ars548_ros_wrapper
        PLUGIN "ContinentalArs548RosWrapper"
        EXECUTABLE continental_ars548_ros_wrapper_node
        )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)

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
