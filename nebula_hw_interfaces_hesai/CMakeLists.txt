cmake_minimum_required(VERSION 3.14)
project(nebula_hw_interfaces_hesai)

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
find_package(nebula_common_hesai REQUIRED)
find_package(nebula_hw_interfaces_base REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(pandar_msgs REQUIRED)

include_directories(
    ${nebula_common_base_INCLUDE_DIRS}
    ${nebula_common_hesai_INCLUDE_DIRS}
    ${nebula_hw_interfaces_base_INCLUDE_DIRS}
    include
    SYSTEM
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${nebula_common_hesai_TARGETS}
    ${nebula_hw_interfaces_base_TARGETS}
)

add_library(nebula_hw_interfaces_hesai SHARED
    src/hesai_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_hesai PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${pandar_msgs_TARGETS}
)

target_include_directories(nebula_hw_interfaces_hesai PUBLIC
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${boost_udp_driver_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
)

install(TARGETS nebula_hw_interfaces_hesai EXPORT export_nebula_hw_interfaces_hesai)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hw_interfaces_hesai)
ament_export_dependencies(
    boost_tcp_driver
    boost_udp_driver
    nebula_common_base
    nebula_common_hesai
    nebula_hw_interfaces_base
    nebula_msgs
    pandar_msgs
)

ament_package()
