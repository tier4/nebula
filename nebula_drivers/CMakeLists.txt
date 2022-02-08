cmake_minimum_required(VERSION 3.5)
project(nebula_lidar_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
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
ament_auto_add_executable(hesai_hw_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface_ros_main.cpp
        )
# Driver Lib
ament_auto_add_library(hesai_driver SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_40_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_64_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_qt_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_xt_decoder.cpp
        )
# RosDriver
ament_auto_add_library(hesai_driver_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver_ros_wrapper.cpp
        )
ament_auto_add_executable(hesai_driver_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver_ros_main.cpp
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
ament_auto_add_executable(velodyne_hw_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_hw_interface_ros_main.cpp
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
ament_auto_add_executable(velodyne_driver_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne/velodyne_driver_ros_main.cpp
        )

## NEW SENSOR
# HwInterfaces Lib
# RosHwInterface
# Driver Lib
# RosDriver

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        calibration
        config
)

