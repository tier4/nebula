cmake_minimum_required(VERSION 3.14)
project(nebula_ros_velodyne)

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
find_package(nebula_common_velodyne REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_decoders_velodyne REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_hw_interfaces_velodyne REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_common_velodyne_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_velodyne_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_velodyne_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${nebula_common_velodyne_TARGETS}
    ${nebula_decoders_base_TARGETS}
    ${nebula_hw_interfaces_base_TARGETS}
    ${nebula_ros_base_TARGETS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
)

## Velodyne
add_library(velodyne_ros_wrapper SHARED
    src/velodyne/velodyne_ros_wrapper.cpp
    src/velodyne/decoder_wrapper.cpp
    src/velodyne/hw_interface_wrapper.cpp
    src/velodyne/hw_monitor_wrapper.cpp
)

target_link_libraries(velodyne_ros_wrapper PUBLIC
    nebula_ros_base::nebula_ros_base_common
)

target_include_directories(velodyne_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_velodyne_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_velodyne_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

target_link_libraries(velodyne_ros_wrapper PUBLIC
    ${diagnostic_updater_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${velodyne_msgs_TARGETS}
    nebula_decoders_velodyne::nebula_decoders_velodyne
    nebula_hw_interfaces_velodyne::nebula_hw_interfaces_velodyne
)

rclcpp_components_register_node(velodyne_ros_wrapper
        PLUGIN "VelodyneRosWrapper"
        EXECUTABLE velodyne_ros_wrapper_node
)

install(TARGETS velodyne_ros_wrapper EXPORT export_velodyne_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(PCL REQUIRED COMPONENTS common io)

    ament_lint_auto_find_test_dependencies()

    add_definitions(-D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
    add_definitions(-D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../nebula_decoders_velodyne/calibration/")

    # VLP16 tests
    add_library(velodyne_ros_decoder_test_vlp16 SHARED
        tests/velodyne_ros_decoder_test_vlp16.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_vlp16 PUBLIC
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_vlp16 PUBLIC
        ${nebula_common_base_TARGETS}
        ${nebula_common_velodyne_TARGETS}
        ${PCL_LIBRARIES}
        ${rosbag2_cpp_TARGETS}
        ${diagnostic_updater_TARGETS}
        nebula_decoders_velodyne::nebula_decoders_velodyne
    )

    ament_add_gtest(velodyne_ros_decoder_test_main_vlp16
        tests/velodyne_ros_decoder_test_main_vlp16.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_main_vlp16 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne
        include
        tests
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_main_vlp16
        velodyne_ros_decoder_test_vlp16
    )

    # VLS128 tests
    add_library(velodyne_ros_decoder_test_vls128 SHARED
        tests/velodyne_ros_decoder_test_vls128.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_vls128 PUBLIC
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_vls128 PUBLIC
        ${nebula_common_base_TARGETS}
        ${nebula_common_velodyne_TARGETS}
        ${PCL_LIBRARIES}
        ${rosbag2_cpp_TARGETS}
        ${diagnostic_updater_TARGETS}
        nebula_decoders_velodyne::nebula_decoders_velodyne
    )

    ament_add_gtest(velodyne_ros_decoder_test_main_vls128
        tests/velodyne_ros_decoder_test_main_vls128.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_main_vls128 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne
        include
        tests
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_main_vls128
        velodyne_ros_decoder_test_vls128
    )

    # VLP32 tests
    add_library(velodyne_ros_decoder_test_vlp32 SHARED
        tests/velodyne_ros_decoder_test_vlp32.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_vlp32 PUBLIC
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_vlp32 PUBLIC
        ${nebula_common_base_TARGETS}
        ${nebula_common_velodyne_TARGETS}
        ${PCL_LIBRARIES}
        ${rosbag2_cpp_TARGETS}
        ${diagnostic_updater_TARGETS}
        nebula_decoders_velodyne::nebula_decoders_velodyne
    )

    ament_add_gtest(velodyne_ros_decoder_test_main_vlp32
        tests/velodyne_ros_decoder_test_main_vlp32.cpp
    )
    target_include_directories(velodyne_ros_decoder_test_main_vlp32 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne
        include
        tests
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_decoder_test_main_vlp32
        ${PCL_LIBRARIES}
        velodyne_ros_decoder_test_vlp32
    )
endif()

if(BUILD_EXAMPLES)
    find_package(PCL REQUIRED COMPONENTS common io)
    find_package(rosbag2_cpp REQUIRED)

    add_library(velodyne_ros_offline_extract_bag_pcd SHARED
        examples/velodyne_ros_offline_extract_bag_pcd.cpp
    )
    target_include_directories(velodyne_ros_offline_extract_bag_pcd PUBLIC
        examples
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(velodyne_ros_offline_extract_bag_pcd PUBLIC
        ${nebula_common_base_TARGETS}
        ${nebula_common_velodyne_TARGETS}
        ${PCL_LIBRARIES}
        ${rosbag2_cpp_TARGETS}
        nebula_decoders_velodyne::nebula_decoders_velodyne
        nebula_ros_base::nebula_ros_base_common
    )

    add_executable(velodyne_ros_offline_extract_bag_pcd_node
        examples/velodyne_ros_offline_extract_bag_pcd_main.cpp
    )
    target_link_libraries(velodyne_ros_offline_extract_bag_pcd_node PUBLIC
        velodyne_ros_offline_extract_bag_pcd
    )

    install(TARGETS velodyne_ros_offline_extract_bag_pcd_node
            DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_velodyne_ros_wrapper)

ament_export_dependencies(
    PCL
    diagnostic_msgs
    diagnostic_updater
    nebula_common_base
    nebula_common_velodyne
    nebula_decoders_base
    nebula_decoders_velodyne
    nebula_hw_interfaces_base
    nebula_hw_interfaces_velodyne
    nebula_msgs
    nebula_ros_base
    rclcpp_components
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
