cmake_minimum_required(VERSION 3.20)
project(nebula_ros_hesai)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
if(BUILD_TESTING OR BUILD_EXAMPLES)
    find_package(PCL REQUIRED components common io)
else()
    find_package(PCL REQUIRED components common)
endif()
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_hesai REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_decoders_hesai REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_hw_interfaces_hesai REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pandar_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sync_tooling_msgs REQUIRED)
find_package(autoware_utils_debug REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(agnocastlib)

if("$ENV{ENABLE_AGNOCAST}" AND NOT agnocastlib_FOUND)
    message(FATAL_ERROR "agnocastlib is required when Agnocast is enabled")
endif()

## Hesai
add_library(hesai_ros_wrapper SHARED
    src/hesai/hesai_ros_wrapper.cpp
    src/hesai/decoder_wrapper.cpp
    src/hesai/hw_interface_wrapper.cpp
    src/hesai/hw_monitor_wrapper.cpp
)

target_include_directories(hesai_ros_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(hesai_ros_wrapper PUBLIC
    diagnostic_updater::diagnostic_updater
    nebula_decoders_hesai::nebula_decoders_hesai
    nebula_hw_interfaces_hesai::nebula_hw_interfaces_hesai
    nebula_ros_base::nebula_ros_base_common
    pcl_common
)
ament_target_dependencies(
    hesai_ros_wrapper
    PUBLIC
    autoware_internal_debug_msgs
    autoware_utils_debug
    diagnostic_msgs
    nebula_msgs
    pandar_msgs
    sensor_msgs
    pcl_conversions
    sync_tooling_msgs
)

if("$ENV{ENABLE_AGNOCAST}")
    message("Agnocast support is enabled")
    ament_target_dependencies(hesai_ros_wrapper PUBLIC agnocastlib)
    add_definitions(-DUSE_AGNOCAST_ENABLED)
endif()

rclcpp_components_register_node(hesai_ros_wrapper
        PLUGIN "HesaiRosWrapper"
        EXECUTABLE hesai_ros_wrapper_node
)

install(TARGETS hesai_ros_wrapper EXPORT export_hesai_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rosbag2_cpp REQUIRED)

    ament_lint_auto_find_test_dependencies()

    add_definitions(-D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
    add_definitions(-D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
    add_definitions(-D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../nebula_decoders_hesai/calibration/")

    # Functional safety test
    ament_add_gtest(test_hesai_functional_safety
        test/test_hesai_functional_safety.cpp
    )
    target_include_directories(test_hesai_functional_safety PRIVATE
        include
    )
    target_link_libraries(test_hesai_functional_safety
        hesai_ros_wrapper
    )

    # Decoder tests
    add_library(hesai_ros_decoder_test SHARED
        tests/hesai_ros_decoder_test.cpp
    )
    target_link_libraries(hesai_ros_decoder_test PRIVATE
        nebula_common_base::nebula_common_base
        nebula_ros_base::nebula_ros_base_common
        pcl_common
        pcl_io
        rosbag2_cpp::rosbag2_cpp
        diagnostic_updater::diagnostic_updater
        nebula_decoders_hesai::nebula_decoders_hesai
    )
    ament_target_dependencies(hesai_ros_decoder_test PUBLIC pandar_msgs)

    ament_add_gtest(hesai_ros_decoder_test_main
        tests/hesai_ros_decoder_test_main.cpp
    )
    target_include_directories(hesai_ros_decoder_test_main PRIVATE
        tests
    )
    target_link_libraries(hesai_ros_decoder_test_main
        hesai_ros_decoder_test
        nebula_common_base::nebula_common_base
        nebula_common_hesai::nebula_common_hesai
        nebula_decoders_hesai::nebula_decoders_hesai
        diagnostic_updater::diagnostic_updater
        pcl_io
    )

    ament_add_gtest(hesai_ros_scan_cutting_test_main
        tests/hesai_ros_scan_cutting_test_main.cpp
    )
    target_include_directories(hesai_ros_scan_cutting_test_main PRIVATE
        tests
    )
    target_link_libraries(hesai_ros_scan_cutting_test_main
        hesai_ros_decoder_test
        nebula_common_base::nebula_common_base
        nebula_common_hesai::nebula_common_hesai
        nebula_decoders_hesai::nebula_decoders_hesai
        diagnostic_updater::diagnostic_updater
        pcl_io
    )
endif()

if(BUILD_EXAMPLES)
    find_package(PCL REQUIRED COMPONENTS common io)
    find_package(rosbag2_cpp REQUIRED)

    # Offline PCD extraction library
    add_library(hesai_ros_offline_extract_pcd SHARED
        examples/hesai_ros_offline_extract_pcd.cpp
    )
    target_include_directories(hesai_ros_offline_extract_pcd PRIVATE
        examples
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(hesai_ros_offline_extract_pcd PRIVATE
        nebula_common_base::nebula_common_base
        pcl_common
        pcl_io
        rosbag2_cpp::rosbag2_cpp
        nebula_decoders_hesai::nebula_decoders_hesai
        nebula_ros_base::nebula_ros_base_common
    )

    add_executable(hesai_ros_offline_extract_pcd_node
        examples/hesai_ros_offline_extract_pcd_main.cpp
    )
    target_link_libraries(hesai_ros_offline_extract_pcd_node PRIVATE
        hesai_ros_offline_extract_pcd
    )

    # Bag PCD extraction library
    add_library(hesai_ros_offline_extract_bag_pcd SHARED
        examples/hesai_ros_offline_extract_bag_pcd.cpp
    )
    target_include_directories(hesai_ros_offline_extract_bag_pcd PRIVATE
        examples
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(hesai_ros_offline_extract_bag_pcd PRIVATE
        nebula_common_base::nebula_common_base
        pcl_common
        pcl_io
        rosbag2_cpp::rosbag2_cpp
        nebula_decoders_hesai::nebula_decoders_hesai
        nebula_ros_base::nebula_ros_base_common
    )

    add_executable(hesai_ros_offline_extract_bag_pcd_node
        examples/hesai_ros_offline_extract_bag_pcd_main.cpp
    )
    target_link_libraries(hesai_ros_offline_extract_bag_pcd_node PRIVATE
        hesai_ros_offline_extract_bag_pcd
    )

    install(TARGETS hesai_ros_offline_extract_pcd_node
                    hesai_ros_offline_extract_bag_pcd_node
            DESTINATION lib/${PROJECT_NAME}
    )
endif()

install(
  DIRECTORY config launch schema
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    install(
        DIRECTORY test_resources
        DESTINATION share/${PROJECT_NAME}
    )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_hesai_ros_wrapper)

  ament_export_dependencies(
      PCL
      diagnostic_msgs
      diagnostic_updater
      nebula_common_base
      nebula_common_hesai
      nebula_decoders_base
      nebula_decoders_hesai
      nebula_hw_interfaces_base
      nebula_hw_interfaces_hesai
      nebula_msgs
      nebula_ros_base
      pandar_msgs
      rclcpp_components
      sensor_msgs
      sync_tooling_msgs
      pcl_conversions
      autoware_utils_debug
      autoware_internal_debug_msgs
  )

if("$ENV{ENABLE_AGNOCAST}")
    ament_export_dependencies(agnocastlib)
endif()

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
