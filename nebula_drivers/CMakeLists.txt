cmake_minimum_required(VERSION 3.5)
project(nebula_lidar_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

ament_auto_find_build_dependencies()

# cURLpp requires libcurl to be installed
message(STATUS "Looking for curl...")
find_package(CURL REQUIRED)

if (CURL_FOUND)
    message(STATUS "Found curl version: ${CURL_VERSION_STRING}")
    message(STATUS "Using curl version: ${CURL_INCLUDE_DIRS}")
    message(STATUS "Using curl libraries: ${CURL_LIBRARIES}\n")
    list(APPEND Project_INCLUDE_DIRS ${CURL_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Could not find curl.")
endif()


include_directories(
        include
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)
## HESAI
# HwInterface Lib
ament_auto_add_library(hesai_hw_interface SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface.cpp
        )
# RosHwInterface
ament_auto_add_library(hesai_hw_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_hw_ros_wrapper
        PLUGIN "HesaiHwInterfaceRosWrapper"
        EXECUTABLE hesai_hw_interface_ros_wrapper_node
        )

# RosHwMonitor
ament_auto_add_library(hesai_hw_monitor_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_monitor_ros_wrapper.cpp
        )
target_link_libraries(hesai_hw_monitor_ros_wrapper ${CURL_LIBRARIES})

rclcpp_components_register_node(hesai_hw_monitor_ros_wrapper
        PLUGIN "HesaiHwMonitorRosWrapper"
        EXECUTABLE hesai_hw_monitor_ros_wrapper_node
        )

# Driver Lib
ament_auto_add_library(hesai_driver SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_40_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_64_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_qt_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_xt_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_xtm_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_at_decoder.cpp
        )
# RosDriver
ament_auto_add_library(hesai_driver_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_driver_ros_wrapper
        PLUGIN "HesaiDriverRosWrapper"
        EXECUTABLE hesai_driver_ros_wrapper_node
        )


## Velodyne
# HwInterfaces Lib
ament_auto_add_library(velodyne_hw_interface SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_hw_interface.cpp
        )
# RosHwInterface
ament_auto_add_library(velodyne_hw_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_hw_interface_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_hw_ros_wrapper
        PLUGIN "VelodyneHwInterfaceRosWrapper"
        EXECUTABLE velodyne_hw_ros_wrapper_node
        )

target_link_libraries(velodyne_hw_ros_wrapper ${CURL_LIBRARIES})

# RosHwMonitor
ament_auto_add_library(velodyne_hw_monitor_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_hw_monitor_ros_wrapper.cpp
        )
target_link_libraries(velodyne_hw_monitor_ros_wrapper ${CURL_LIBRARIES})
rclcpp_components_register_node(velodyne_hw_monitor_ros_wrapper
        PLUGIN "VelodyneHwMonitorRosWrapper"
        EXECUTABLE velodyne_hw_monitor_ros_wrapper_node
        )

# Driver Lib
ament_auto_add_library(velodyne_driver SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_driver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/decoders/velodyne_calibration_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/decoders/vls128_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/decoders/vlp16_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/decoders/vlp32_decoder.cpp
        )

target_link_libraries(velodyne_driver ${YAML_CPP_LIBRARIES})
# RosDriver
ament_auto_add_library(velodyne_driver_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_driver_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_driver_ros_wrapper
        PLUGIN "VelodyneDriverRosWrapper"
        EXECUTABLE velodyne_driver_ros_wrapper_node
        )


## NEW SENSOR
# HwInterfaces Lib
# RosHwInterface
# RosHwMonitor
# Driver Lib
# RosDriver

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()

        add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/")
        add_definitions(-D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/calibration/")
        # lib for TEST
        ament_auto_add_library(hesai_ros_decoder_test_at128 SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/test/hesai_ros_decoder_test_at128.cpp
        )
        target_link_libraries(hesai_ros_decoder_test_at128 ${PCL_LIBRARIES})
        ament_auto_add_library(hesai_ros_decoder_test_xt32m SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/test/hesai_ros_decoder_test_xt32m.cpp
        )
        target_link_libraries(hesai_ros_decoder_test_xt32m ${PCL_LIBRARIES})

        find_package(ament_cmake_gtest)
        ament_add_gtest(hesai_ros_decoder_test_main_at128
                test/hesai_ros_decoder_test_main_at128.cpp
                )
        ament_target_dependencies(hesai_ros_decoder_test_main_at128
                rclcpp
                rosbag2_cpp
                pandar_msgs
                )
        target_include_directories(hesai_ros_decoder_test_main_at128 PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/src
                include
                )
        target_link_libraries(hesai_ros_decoder_test_main_at128
                ${PCL_LIBRARIES}
                hesai_ros_decoder_test_at128
                )
        ament_add_gtest(hesai_ros_decoder_test_main_xt32m
                test/hesai_ros_decoder_test_main_xt32m.cpp
                )
        ament_target_dependencies(hesai_ros_decoder_test_main_xt32m
                rclcpp
                rosbag2_cpp
                pandar_msgs
                )
        target_include_directories(hesai_ros_decoder_test_main_xt32m PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/src
                include
                )
        target_link_libraries(hesai_ros_decoder_test_main_xt32m
                ${PCL_LIBRARIES}
                hesai_ros_decoder_test_xt32m
                )
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        calibration
        config
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
