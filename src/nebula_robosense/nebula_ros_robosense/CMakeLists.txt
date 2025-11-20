cmake_minimum_required(VERSION 3.20)
project(nebula_ros_robosense)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED components common)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_robosense REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_decoders_robosense REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_hw_interfaces_robosense REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

## Robosense
add_library(robosense_ros_wrapper SHARED
    src/robosense/robosense_ros_wrapper.cpp
    src/robosense/decoder_wrapper.cpp
    src/robosense/hw_interface_wrapper.cpp
    src/robosense/hw_monitor_wrapper.cpp
)

target_include_directories(robosense_ros_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(robosense_ros_wrapper PUBLIC
    diagnostic_updater::diagnostic_updater
    nebula_decoders_robosense::nebula_decoders_robosense
    nebula_decoders_robosense::nebula_decoders_robosense_info
    nebula_hw_interfaces_robosense::nebula_hw_interfaces_robosense
    nebula_ros_base::nebula_ros_base_common
    pcl_common)

ament_target_dependencies(robosense_ros_wrapper PUBLIC
    diagnostic_msgs
    robosense_msgs
    nebula_msgs
    rclcpp_components
)

rclcpp_components_register_node(robosense_ros_wrapper
    PLUGIN "RobosenseRosWrapper"
    EXECUTABLE robosense_ros_wrapper_node
)

install(TARGETS robosense_ros_wrapper EXPORT export_robosense_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY config launch schema
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_robosense_ros_wrapper)

ament_export_dependencies(
    PCL
    diagnostic_msgs
    diagnostic_updater
    nebula_common_base
    nebula_common_robosense
    nebula_decoders_base
    nebula_decoders_robosense
    nebula_hw_interfaces_base
    nebula_hw_interfaces_robosense
    nebula_msgs
    nebula_ros_base
    rclcpp_components
    robosense_msgs
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
