cmake_minimum_required(VERSION 3.20)
project(nebula_hw_interfaces_velodyne)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(boost_tcp_driver)
find_package(boost_udp_driver)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_velodyne REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(velodyne_msgs REQUIRED)

add_library(nebula_hw_interfaces_velodyne SHARED
    src/velodyne_hw_interface.cpp
)

target_include_directories(nebula_hw_interfaces_velodyne PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nebula_hw_interfaces_velodyne PUBLIC
    boost_tcp_driver
    boost_udp_driver
    velodyne_msgs
    nebula_common_base
    nebula_common_velodyne
    nebula_hw_interfaces_base
)

install(TARGETS nebula_hw_interfaces_velodyne EXPORT export_nebula_hw_interfaces_velodyne)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hw_interfaces_velodyne)
ament_export_dependencies(
    boost_tcp_driver
    boost_udp_driver
    nebula_common_base
    nebula_common_velodyne
    nebula_hw_interfaces_base
    nebula_msgs
    velodyne_msgs
)

ament_package()
