cmake_minimum_required(VERSION 3.14)
project(nebula_ros_continental)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED components common)
find_package(autoware_sensing_msgs REQUIRED)
find_package(continental_msgs REQUIRED)
find_package(continental_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_continental REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_decoders_continental REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_hw_interfaces_continental REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sync_tooling_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_common_continental_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_continental_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_continental_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${nebula_common_continental_TARGETS}
    ${nebula_decoders_base_TARGETS}
    ${nebula_hw_interfaces_base_TARGETS}
    ${nebula_ros_base_TARGETS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
)

## Continental

# ARS548
add_library(continental_ars548_ros_wrapper SHARED
    src/continental/continental_ars548_ros_wrapper.cpp
    src/continental/continental_ars548_decoder_wrapper.cpp
    src/continental/continental_ars548_hw_interface_wrapper.cpp
)

target_include_directories(continental_ars548_ros_wrapper PUBLIC
    ${autoware_sensing_msgs_INCLUDE_DIRS}
    ${continental_msgs_INCLUDE_DIRS}
    ${continental_srvs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${diagnostic_updater_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_continental_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_continental_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${sync_tooling_msgs_INCLUDE_DIRS}
)

target_link_libraries(continental_ars548_ros_wrapper PUBLIC
    ${autoware_sensing_msgs_TARGETS}
    ${continental_msgs_TARGETS}
    ${continental_srvs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nebula_msgs_TARGETS}
    ${radar_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${visualization_msgs_TARGETS}
    ${sync_tooling_msgs_TARGETS}
    nebula_decoders_continental::nebula_decoders_continental
    nebula_hw_interfaces_continental::nebula_hw_interfaces_continental
)

rclcpp_components_register_node(continental_ars548_ros_wrapper
    PLUGIN "ContinentalARS548RosWrapper"
    EXECUTABLE continental_ars548_ros_wrapper_node
)

# SRR520
add_library(continental_srr520_ros_wrapper SHARED
    src/continental/continental_srr520_ros_wrapper.cpp
    src/continental/continental_srr520_decoder_wrapper.cpp
    src/continental/continental_srr520_hw_interface_wrapper.cpp
)

target_link_libraries(continental_srr520_ros_wrapper PUBLIC
    nebula_ros_base::nebula_ros_base_common
)

target_include_directories(continental_srr520_ros_wrapper PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${continental_srvs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_continental_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_continental_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
)

target_link_libraries(continental_srr520_ros_wrapper PUBLIC
    ${continental_msgs_TARGETS}
    ${continental_srvs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nebula_msgs_TARGETS}
    ${radar_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${visualization_msgs_TARGETS}
    ${message_filters_TARGETS}
    nebula_decoders_continental::nebula_decoders_continental
    nebula_hw_interfaces_continental::nebula_hw_interfaces_continental
)

rclcpp_components_register_node(continental_srr520_ros_wrapper
    PLUGIN "ContinentalSRR520RosWrapper"
    EXECUTABLE continental_srr520_ros_wrapper_node
)

install(TARGETS continental_ars548_ros_wrapper EXPORT export_continental_ars548_ros_wrapper LIBRARY DESTINATION lib)
install(TARGETS continental_srr520_ros_wrapper EXPORT export_continental_srr520_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY config launch schema
  DESTINATION share/${PROJECT_NAME}
)

# Install common schema files from nebula_ros_base
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../nebula_ros_base/schema/communication.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../nebula_ros_base/schema/diagnostics.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../nebula_ros_base/schema/hardware.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../nebula_ros_base/schema/misc.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../nebula_ros_base/schema/topic.json
  DESTINATION share/${PROJECT_NAME}/schema
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_continental_ars548_ros_wrapper)
ament_export_targets(export_continental_srr520_ros_wrapper)

ament_export_dependencies(
    PCL
    autoware_sensing_msgs
    continental_msgs
    continental_srvs
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    message_filters
    nebula_common_base
    nebula_common_continental
    nebula_decoders_base
    nebula_decoders_continental
    nebula_hw_interfaces_base
    nebula_hw_interfaces_continental
    nebula_msgs
    nebula_ros_base
    radar_msgs
    rclcpp_components
    sync_tooling_msgs
    tf2_ros
    visualization_msgs
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
