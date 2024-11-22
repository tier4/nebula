cmake_minimum_required(VERSION 3.14)
project(nebula_hw_interfaces)

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
find_package(nebula_common)
find_package(nebula_msgs)
find_package(pandar_msgs)
find_package(robosense_msgs)
find_package(ros2_socketcan)
find_package(velodyne_msgs)

# Common includes for all targets
include_directories(
    ${nebula_common_INCLUDE_DIRS}
    include
    SYSTEM
)

link_libraries(
    ${nebula_common_TARGETS}
)

add_library(nebula_hw_interfaces_hesai SHARED
    src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp
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

add_library(nebula_hw_interfaces_velodyne SHARED
    src/nebula_velodyne_hw_interfaces/velodyne_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_velodyne PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${velodyne_msgs_TARGETS}
)
target_include_directories(nebula_hw_interfaces_velodyne PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

add_library(nebula_hw_interfaces_robosense SHARED
    src/nebula_robosense_hw_interfaces/robosense_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_robosense PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${robosense_msgs_TARGETS}
)
target_include_directories(nebula_hw_interfaces_robosense PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${robosense_msgs_INCLUDE_DIRS}
)

add_library(nebula_hw_interfaces_continental SHARED
    src/nebula_continental_hw_interfaces/continental_ars548_hw_interface.cpp
    src/nebula_continental_hw_interfaces/continental_srr520_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_continental PUBLIC
    ${boost_udp_driver_LIBRARIES}
    ${nebula_msgs_TARGETS}
    ${ros2_socketcan_LIBRARIES}
)
target_include_directories(nebula_hw_interfaces_continental PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${ros2_socketcan_INCLUDE_DIRS}
)

install(TARGETS nebula_hw_interfaces_hesai EXPORT export_nebula_hw_interfaces_hesai)
install(TARGETS nebula_hw_interfaces_velodyne EXPORT export_nebula_hw_interfaces_velodyne)
install(TARGETS nebula_hw_interfaces_robosense EXPORT export_nebula_hw_interfaces_robosense)
install(TARGETS nebula_hw_interfaces_continental EXPORT export_nebula_hw_interfaces_continental)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_udp
        test/common/test_udp.cpp
    )

    target_include_directories(test_udp PUBLIC
        ${nebula_common_INCLUDE_DIRS}
        include
        test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hw_interfaces_hesai)
ament_export_targets(export_nebula_hw_interfaces_velodyne)
ament_export_targets(export_nebula_hw_interfaces_robosense)
ament_export_targets(export_nebula_hw_interfaces_continental)

ament_export_dependencies(
    boost_tcp_driver
    boost_udp_driver
    nebula_common
    nebula_msgs
    pandar_msgs
    robosense_msgs
    ros2_socketcan
    velodyne_msgs
)

ament_package()
