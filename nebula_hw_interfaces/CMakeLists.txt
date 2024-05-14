cmake_minimum_required(VERSION 3.14)
project(nebula_hw_interfaces)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
#find_package(pcl_conversions REQUIRED)
find_package(boost_tcp_driver)
find_package(boost_udp_driver)
find_package(nebula_common)
find_package(nebula_msgs)
find_package(pandar_msgs)
find_package(rclcpp)
find_package(rclcpp_components)
find_package(robosense_msgs)
find_package(sensor_msgs)
find_package(velodyne_msgs)

#ament_auto_find_build_dependencies() # this finds all components to it is slower

message(STATUS "boost_tcp_driver_TARGETS : ${boost_tcp_driver_TARGETS}")
message(STATUS "boost_tcp_driver_LIBRARIES : ${boost_tcp_driver_LIBRARIES}")
message(STATUS "boost_tcp_driver_INCLUDE_DIRS : ${boost_tcp_driver_INCLUDE_DIRS}")
message(STATUS "boost_udp_driver_TARGETS : ${boost_udp_driver_TARGETS}")
message(STATUS "boost_udp_driver_LIBRARIES : ${boost_udp_driver_LIBRARIES}")
message(STATUS "boost_udp_driver_INCLUDE_DIRS : ${boost_udp_driver_INCLUDE_DIRS}")

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

# Common includes for all targets
include_directories(
    include
    SYSTEM
)

set(common_include_dirs
    ${nebula_common_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${rcl_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

set(common_targets
    ${nebula_common_TARGETS}
    ${rcl_TARGETS}
    ${rclcpp_TARGETS}
)

add_library(nebula_hw_interfaces_hesai SHARED
    src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_hesai PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${pandar_msgs_TARGETS}
    ${common_targets}
)

target_include_directories(nebula_hw_interfaces_hesai PUBLIC
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${boost_udp_driver_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
    ${common_include_dirs}
)

add_library(nebula_hw_interfaces_velodyne SHARED
    src/nebula_velodyne_hw_interfaces/velodyne_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_velodyne PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${velodyne_msgs_TARGETS}
    ${common_targets}
)
target_include_directories(nebula_hw_interfaces_velodyne PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
    ${common_include_dirs}
)

add_library(nebula_hw_interfaces_robosense SHARED
    src/nebula_robosense_hw_interfaces/robosense_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_robosense PUBLIC
    ${boost_tcp_driver_LIBRARIES}
    ${boost_udp_driver_LIBRARIES}
    ${robosense_msgs_TARGETS}
    ${common_targets}
)
target_include_directories(nebula_hw_interfaces_robosense PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${boost_tcp_driver_INCLUDE_DIRS}
    ${robosense_msgs_INCLUDE_DIRS}
    ${common_include_dirs}
)

add_library(nebula_hw_interfaces_continental SHARED
    src/nebula_continental_hw_interfaces/continental_ars548_hw_interface.cpp
    src/nebula_continental_hw_interfaces/multi_continental_ars548_hw_interface.cpp
)
target_link_libraries(nebula_hw_interfaces_continental PUBLIC
    ${boost_udp_driver_LIBRARIES}
    ${nebula_msgs_TARGETS}
    ${common_targets}
)
target_include_directories(nebula_hw_interfaces_continental PUBLIC
    ${boost_udp_driver_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${common_include_dirs}
)

install(TARGETS nebula_hw_interfaces_hesai EXPORT export_nebula_hw_interfaces_hesai)
install(TARGETS nebula_hw_interfaces_velodyne EXPORT export_nebula_hw_interfaces_velodyne)
install(TARGETS nebula_hw_interfaces_robosense EXPORT export_nebula_hw_interfaces_robosense)
install(TARGETS nebula_hw_interfaces_continental EXPORT export_nebula_hw_interfaces_continental)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hw_interfaces_hesai)
ament_export_targets(export_nebula_hw_interfaces_velodyne)
ament_export_targets(export_nebula_hw_interfaces_robosense)
ament_export_targets(export_nebula_hw_interfaces_continental)

# Need to consider this
ament_export_dependencies(
    PCL
    boost_tcp_driver
    boost_udp_driver
    nebula_common
    nebula_msgs
    pandar_msgs
    rcl
    rclcpp
    rclcpp_components
    robosense_msgs
    sensor_msgs
    velodyne_msgs
)


#ament_auto_package() trying it without
ament_package()
