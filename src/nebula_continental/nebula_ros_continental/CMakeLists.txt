cmake_minimum_required(VERSION 3.20)
project(nebula_ros_continental)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

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
find_package(nebula_decoders_continental REQUIRED)
find_package(nebula_hw_interfaces_continental REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sync_tooling_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# # Continental

# ARS548
add_library(continental_ars548_ros_wrapper SHARED
    src/continental/continental_ars548_ros_wrapper.cpp
    src/continental/continental_ars548_decoder_wrapper.cpp
    src/continental/continental_ars548_hw_interface_wrapper.cpp
)

target_include_directories(continental_ars548_ros_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(continental_ars548_ros_wrapper PUBLIC
    diagnostic_updater::diagnostic_updater
    tf2_ros::tf2_ros
    nebula_decoders_continental::nebula_decoders_continental
    nebula_hw_interfaces_continental::nebula_hw_interfaces_continental
    nebula_ros_base::nebula_ros_base
    pcl_common)
ament_target_dependencies(continental_ars548_ros_wrapper PUBLIC
    autoware_sensing_msgs
    continental_msgs
    continental_srvs
    diagnostic_msgs
    geometry_msgs
    pcl_conversions
    nebula_msgs
    radar_msgs
    sensor_msgs
    visualization_msgs
    sync_tooling_msgs
    rclcpp_components
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

target_include_directories(continental_srr520_ros_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(continental_srr520_ros_wrapper PUBLIC
    tf2_ros::tf2_ros
    message_filters::message_filters
    nebula_decoders_continental::nebula_decoders_continental
    nebula_hw_interfaces_continental::nebula_hw_interfaces_continental
    nebula_ros_base::nebula_ros_base
    pcl_common)

ament_target_dependencies(continental_srr520_ros_wrapper PUBLIC
    continental_msgs
    continental_srvs
    diagnostic_msgs
    geometry_msgs
    nebula_msgs
    pcl_conversions
    radar_msgs
    sensor_msgs
    visualization_msgs
    rclcpp_components
)

rclcpp_components_register_node(continental_srr520_ros_wrapper
    PLUGIN "ContinentalSRR520RosWrapper"
    EXECUTABLE continental_srr520_ros_wrapper_node
)

install(TARGETS continental_ars548_ros_wrapper EXPORT export_continental_ars548_ros_wrapper LIBRARY DESTINATION lib)
install(TARGETS continental_srr520_ros_wrapper EXPORT export_continental_srr520_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(PCL REQUIRED COMPONENTS common)

    ament_lint_auto_find_test_dependencies()

    add_definitions(-D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test_resources/")

    # ARS548 tests
    add_library(continental_ros_decoder_test_ars548 SHARED
        tests/continental_ros_decoder_test_ars548.cpp
        tests/parameter_descriptors.cpp
    )
    target_include_directories(continental_ros_decoder_test_ars548 PRIVATE
        tests
        include
    )
    target_link_libraries(continental_ros_decoder_test_ars548 PRIVATE
        nebula_common_base::nebula_common_base
        pcl_common
        rosbag2_cpp::rosbag2_cpp
        diagnostic_updater::diagnostic_updater
        nebula_decoders_continental::nebula_decoders_continental
    )

    ament_add_gtest(continental_ros_decoder_test_main_ars548
        tests/continental_ros_decoder_test_main_ars548.cpp
    )
    target_include_directories(continental_ros_decoder_test_main_ars548 PRIVATE
        tests
    )
    target_link_libraries(continental_ros_decoder_test_main_ars548
        continental_ros_decoder_test_ars548
        nebula_common_base::nebula_common_base
        nebula_decoders_continental::nebula_decoders_continental
        diagnostic_updater::diagnostic_updater
    )

    # SRR520 tests
    add_library(continental_ros_decoder_test_srr520 SHARED
        tests/continental_ros_decoder_test_srr520.cpp
        tests/parameter_descriptors.cpp
    )
    target_include_directories(continental_ros_decoder_test_srr520 PRIVATE
        tests
        include
    )
    target_link_libraries(continental_ros_decoder_test_srr520
        nebula_common_base::nebula_common_base
        pcl_common
        rosbag2_cpp::rosbag2_cpp
        diagnostic_updater::diagnostic_updater
        nebula_decoders_continental::nebula_decoders_continental
    )

    ament_add_gtest(continental_ros_decoder_test_main_srr520
        tests/continental_ros_decoder_test_main_srr520.cpp
    )
    target_include_directories(continental_ros_decoder_test_main_srr520 PRIVATE
        tests
    )
    target_link_libraries(continental_ros_decoder_test_main_srr520
        continental_ros_decoder_test_srr520
        nebula_common_base::nebula_common_base
        nebula_decoders_continental::nebula_decoders_continental
        diagnostic_updater::diagnostic_updater
    )

    install(
        DIRECTORY test_resources
        DESTINATION share/${PROJECT_NAME}
    )
endif()

install(
    DIRECTORY config launch schema
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

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
    nebula_decoders_continental
    nebula_hw_interfaces_continental
    nebula_msgs
    nebula_ros_base
    pcl_conversions
    radar_msgs
    rclcpp_components
    sensor_msgs
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
