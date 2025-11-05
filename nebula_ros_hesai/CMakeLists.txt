cmake_minimum_required(VERSION 3.14)
project(nebula_ros_hesai)

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
find_package(pandar_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sync_tooling_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(autoware_utils_debug REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(agnocastlib)

if("$ENV{ENABLE_AGNOCAST}" AND NOT agnocastlib_FOUND)
    message(FATAL_ERROR "agnocastlib is required when Agnocast is enabled")
endif()

include_directories(
    include
    SYSTEM
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_common_hesai_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_hesai_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_hesai_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${nebula_common_hesai_TARGETS}
    ${nebula_decoders_base_TARGETS}
    ${nebula_hw_interfaces_base_TARGETS}
    ${nebula_ros_base_TARGETS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
)

## Hesai
add_library(hesai_ros_wrapper SHARED
    src/hesai/hesai_ros_wrapper.cpp
    src/hesai/decoder_wrapper.cpp
    src/hesai/hw_interface_wrapper.cpp
    src/hesai/hw_monitor_wrapper.cpp
)

target_include_directories(hesai_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_decoders_base_INCLUDE_DIRS}
    ${nebula_decoders_hesai_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    ${nebula_hw_interfaces_hesai_INCLUDE_DIRS}
    ${nebula_ros_base_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
    ${sync_tooling_msgs_INCLUDE_DIRS}
    ${autoware_utils_debug_INCLUDE_DIRS}
    ${autoware_internal_debug_msgs_INCLUDE_DIRS}
)

target_link_libraries(hesai_ros_wrapper PUBLIC
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${pandar_msgs_TARGETS}
    ${sync_tooling_msgs_TARGETS}
    ${autoware_utils_debug_TARGETS}
    ${autoware_internal_debug_msgs_TARGETS}
    nebula_decoders_hesai::nebula_decoders_hesai
    nebula_hw_interfaces_hesai::nebula_hw_interfaces_hesai
    nebula_ros_base::nebula_ros_base_common
)

if("$ENV{ENABLE_AGNOCAST}")
    message("Agnocast support is enabled")
    target_include_directories(hesai_ros_wrapper PUBLIC ${agnocastlib_INCLUDE_DIRS})
    target_link_libraries(hesai_ros_wrapper PUBLIC ${agnocastlib_LIBRARIES})
    add_definitions(-DUSE_AGNOCAST_ENABLED)
endif()

rclcpp_components_register_node(hesai_ros_wrapper
        PLUGIN "HesaiRosWrapper"
        EXECUTABLE hesai_ros_wrapper_node
)

install(TARGETS hesai_ros_wrapper EXPORT export_hesai_ros_wrapper LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(nebula_decoders_hesai REQUIRED)

    ament_lint_auto_find_test_dependencies()

    add_definitions(-D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")

    ament_add_gtest(test_hesai_functional_safety
        test/test_hesai_functional_safety.cpp
    )
    target_include_directories(test_hesai_functional_safety
        PRIVATE
        include
        ${nebula_decoders_hesai_INCLUDE_DIRS}
    )
    target_link_libraries(test_hesai_functional_safety
        hesai_ros_wrapper
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
    sync_tooling_msgs
    yaml-cpp
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
