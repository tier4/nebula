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

ament_auto_find_build_dependencies()

# HwInterfaces
ament_auto_add_library(hesai_hw_interface SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/nebula_hesai_hw_interface.cpp
        )

# Driver
ament_auto_add_library(hesai_driver SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai/nebula_hesai_driver.cpp
        )