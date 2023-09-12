cmake_minimum_required(VERSION 3.14)
project(nebula_hw_interfaces)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

include_directories(
        include
        SYSTEM
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_COMMON_INCLUDE_DIRS}
)

ament_auto_add_library(nebula_hw_interfaces_hesai SHARED
        src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp
        )

ament_auto_add_library(nebula_hw_interfaces_velodyne SHARED
        src/nebula_velodyne_hw_interfaces/velodyne_hw_interface.cpp
        )

ament_auto_add_library(nebula_hw_interfaces_robosense SHARED
        src/nebula_robosense_hw_interfaces/robosense_hw_interface.cpp
        )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
