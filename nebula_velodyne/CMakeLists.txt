cmake_minimum_required(VERSION 3.14)
project(nebula_ros)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED components common)
find_package(continental_msgs REQUIRED)
find_package(continental_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nebula_common REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_hw_interfaces REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pandar_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
    ${nebula_common_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_TARGETS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
)

## Velodyne
add_library(velodyne_ros_wrapper SHARED
    src/velodyne/velodyne_ros_wrapper.cpp
    src/velodyne/decoder_wrapper.cpp
    src/velodyne/hw_interface_wrapper.cpp
    src/velodyne/hw_monitor_wrapper.cpp
    src/common/parameter_descriptors.cpp
)

target_include_directories(velodyne_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

target_link_libraries(velodyne_ros_wrapper PUBLIC
    ${diagnostic_updater_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${velodyne_msgs_TARGETS}
    nebula_decoders::nebula_decoders_velodyne
    nebula_hw_interfaces::nebula_hw_interfaces_velodyne
)

rclcpp_components_register_node(velodyne_ros_wrapper
        PLUGIN "VelodyneRosWrapper"
        EXECUTABLE velodyne_ros_wrapper_node
)

install(TARGETS velodyne_ros_wrapper EXPORT export_velodyne_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ros_testing REQUIRED)

    ament_lint_auto_find_test_dependencies()

    foreach(MODEL VLP16 VLP32 VLS128)
        string(TOLOWER ${MODEL}_smoke_test test_name)
        add_ros_test(
            test/smoke_test.py
            TARGET ${test_name}
            ARGS "launch_file_path:=${PROJECT_SOURCE_DIR}/launch/nebula_launch.py" "sensor_model:=${MODEL}"
            TIMEOUT "10"
        )
    endforeach()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_velodyne_ros_wrapper)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    PCL
    continental_msgs
    continental_srvs
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    nebula_common
    nebula_decoders
    nebula_hw_interfaces
    nebula_msgs
    pandar_msgs
    radar_msgs
    rclcpp_components
    robosense_msgs
    sensor_msgs
    tf2_ros
    velodyne_msgs
    visualization_msgs
    yaml-cpp
)

ament_package()

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
