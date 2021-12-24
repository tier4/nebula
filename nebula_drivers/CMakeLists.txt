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

ament_auto_find_build_dependencies()

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)

# HwInterfaces Lib
ament_auto_add_library(hesai_hw_interface SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface.cpp
        )
# RosHWInterface
ament_auto_add_library(hesai_hw_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface_ros_wrapper.cpp
        )
ament_auto_add_executable(hesai_hw_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_hw_interface_ros_main.cpp
        )

# RosDriver Lib
ament_auto_add_library(hesai_driver SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_40_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_64_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_qt_decoder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/decoders/pandar_xt_decoder.cpp
        )

# RosDriverNode
ament_auto_add_library(hesai_driver_ros_wrapper SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver_ros_wrapper.cpp
        )
ament_auto_add_executable(hesai_driver_ros_wrapper_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/hesai_driver_ros_main.cpp
        )

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        calibration
        config
)

