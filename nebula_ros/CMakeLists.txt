cmake_minimum_required(VERSION 3.14)
project(nebula_ros)

find_package(ament_cmake_auto REQUIRED)

#ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(PCL REQUIRED components common)
find_package(continental_msgs REQUIRED)
find_package(continental_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_common REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_hw_interfaces REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

#get_cmake_property(_variableNames VARIABLES)
#foreach(_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

message(STATUS "continental_msgs_LIBRARIES:" ${continental_msgs_LIBRARIES})
message(STATUS "continental_srvs_LIBRARIES: " ${continental_srvs_LIBRARIES})
message(STATUS "nebula_hw_interfaces_LIBRARIES: " ${nebula_hw_interfaces_LIBRARIES})
message(STATUS "nebula_decoders_LIBRARIES: " ${nebula_decoders_LIBRARIES})
message(STATUS "nebula_msgs_LIBRARIES: " ${nebula_msgs_LIBRARIES})
message(STATUS "PCL_LIBRARIES: " ${PCL_LIBRARIES})
message(STATUS "radar_msgs_LIBRARIES: " ${radar_msgs_LIBRARIES})
message(STATUS "rclcpp_LIBRARIES: " ${rclcpp_LIBRARIES})
message(STATUS "rcl_LIBRARIES:  " ${rcl_LIBRARIES})
message(STATUS "rclcpp_components_LIBRARIES: " ${rclcpp_components_LIBRARIES})
message(STATUS "tf2_eigen_LIBRARIES: " ${tf2_eigen_LIBRARIES})
message(STATUS "tf2_msgs_LIBRARIES: " ${tf2_msgs_LIBRARIES})
message(STATUS "====nebula_decoders::nebula_decoders_continental: " nebula_decoders::nebula_decoders_continental)
message(STATUS "diagnostic_updater_TARGETS: ${diagnostic_updater_TARGETS}")
message(STATUS "diagnostic_updater_LIBRARIES: ${diagnostic_updater_LIBRARIES}")
message(STATUS "diagnostic_updater_INCLUDE_DIRS: ${diagnostic_updater_INCLUDE_DIRS}")

#get_target_property(linked_libs nebula_decoders::nebula_decoders_continental INTERFACE_LINK_LIBRARIES)
#message(STATUS "Libraries linked by MyPackage::Comp1: ${linked_libs}")

message(STATUS " --- rcl_INCLUDE_DIRS : ${rcl_INCLUDE_DIRS}")
message(STATUS " --- rclcpp_INCLUDE_DIRS : ${rclcpp_INCLUDE_DIRS}")
message(STATUS " --- rclcpp_components_INCLUDE_DIRS : ${rclcpp_components_INCLUDE_DIRS}")
message(STATUS " --- nebula_decoders_INCLUDE_DIRS : ${nebula_decoders_INCLUDE_DIRS}")
message(STATUS " --- nebula_hw_interfaces_INCLUDE_DIRS : ${nebula_hw_interfaces_INCLUDE_DIRS}")

include_directories(
    include
    SYSTEM
    ${nebula_common_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    #${rcl_INCLUDE_DIRS}
    #${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)
#link_libraries(${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})

link_libraries(
    ${nebula_common_TARGETS}
    ${rcl_TARGETS}
    ${rclcpp_TARGETS}
    # Consider rclcpp_components::component
)



## Hesai
# Hw Interface
add_library(hesai_hw_ros_wrapper SHARED
    src/hesai/hesai_hw_interface_ros_wrapper.cpp
)
target_link_libraries(hesai_hw_ros_wrapper PUBLIC
    ${pandar_msgs_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_hesai
)
target_include_directories(hesai_hw_ros_wrapper PUBLIC
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(hesai_hw_ros_wrapper
    PLUGIN "HesaiHwInterfaceRosWrapper"
    EXECUTABLE hesai_hw_interface_ros_wrapper_node
)

# Monitor
add_library(hesai_hw_monitor_ros_wrapper SHARED
    src/hesai/hesai_hw_monitor_ros_wrapper.cpp
)
target_link_libraries(hesai_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${pandar_msgs_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_hesai
)
target_include_directories(hesai_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(hesai_hw_monitor_ros_wrapper
    PLUGIN "HesaiHwMonitorRosWrapper"
    EXECUTABLE hesai_hw_monitor_ros_wrapper_node
)

# DriverDecoder
add_library(hesai_driver_ros_wrapper SHARED
    src/hesai/hesai_decoder_ros_wrapper.cpp
)
target_link_libraries(hesai_driver_ros_wrapper PUBLIC
    ${pandar_msgs_TARGETS}
    nebula_decoders::nebula_decoders_hesai
    nebula_hw_interfaces::nebula_hw_interfaces_hesai
)
target_include_directories(hesai_driver_ros_wrapper PUBLIC
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${pandar_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(hesai_driver_ros_wrapper
    PLUGIN "HesaiDriverRosWrapper"
    EXECUTABLE hesai_driver_ros_wrapper_node
)

## Velodyne
# Hw Interface
add_library(velodyne_hw_ros_wrapper SHARED
    src/velodyne/velodyne_hw_interface_ros_wrapper.cpp
)
target_link_libraries(velodyne_hw_ros_wrapper PUBLIC
    ${velodyne_msgs_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_velodyne
)
target_include_directories(velodyne_hw_ros_wrapper PUBLIC
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(velodyne_hw_ros_wrapper
    PLUGIN "VelodyneHwInterfaceRosWrapper"
    EXECUTABLE velodyne_hw_ros_wrapper_node
)


# Monitor
add_library(velodyne_hw_monitor_ros_wrapper SHARED
    src/velodyne/velodyne_hw_monitor_ros_wrapper.cpp
)
target_link_libraries(velodyne_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_updater_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${velodyne_msgs_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_velodyne
)
target_include_directories(velodyne_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(velodyne_hw_monitor_ros_wrapper
    PLUGIN "VelodyneHwMonitorRosWrapper"
    EXECUTABLE velodyne_hw_monitor_ros_wrapper_node
)

# DriverDecoder
add_library(velodyne_driver_ros_wrapper SHARED
    src/velodyne/velodyne_decoder_ros_wrapper.cpp
)
target_link_libraries(velodyne_driver_ros_wrapper PUBLIC
    ${velodyne_msgs_TARGETS}
    nebula_decoders::nebula_decoders_velodyne
    nebula_hw_interfaces::nebula_hw_interfaces_velodyne
)
target_include_directories(velodyne_driver_ros_wrapper PUBLIC
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${velodyne_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(velodyne_driver_ros_wrapper
    PLUGIN "VelodyneDriverRosWrapper"
    EXECUTABLE velodyne_driver_ros_wrapper_node
)


## Robosense
# Hw Interface
add_library(robosense_hw_ros_wrapper SHARED
    src/robosense/robosense_hw_interface_ros_wrapper.cpp
)
target_link_libraries(robosense_hw_ros_wrapper PUBLIC
    ${robosense_msgs_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_robosense
)
target_include_directories(robosense_hw_ros_wrapper PUBLIC
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${robosense_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(robosense_hw_ros_wrapper
    PLUGIN "RobosenseHwInterfaceRosWrapper"
    EXECUTABLE robosense_hw_interface_ros_wrapper_node
)

# DriverDecoder
add_library(robosense_driver_ros_wrapper SHARED
    src/robosense/robosense_decoder_ros_wrapper.cpp
)
target_link_libraries(robosense_driver_ros_wrapper PUBLIC
    ${robosense_msgs_TARGETS}
    nebula_decoders::nebula_decoders_robosense
    nebula_decoders::nebula_decoders_robosense_info
    nebula_hw_interfaces::nebula_hw_interfaces_robosense
)
target_include_directories(robosense_driver_ros_wrapper PUBLIC
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${robosense_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(robosense_driver_ros_wrapper
    PLUGIN "RobosenseDriverRosWrapper"
    EXECUTABLE robosense_driver_ros_wrapper_node
)

# Monitor
add_library(robosense_hw_monitor_ros_wrapper SHARED
    src/robosense/robosense_hw_monitor_ros_wrapper.cpp
)
target_link_libraries(robosense_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_updater_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${robosense_msgs_TARGETS}
    nebula_decoders::nebula_decoders_robosense
    nebula_decoders::nebula_decoders_robosense_info
    nebula_hw_interfaces::nebula_hw_interfaces_robosense
)
target_include_directories(robosense_hw_monitor_ros_wrapper PUBLIC
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nebula_decoders_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${robosense_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(robosense_hw_monitor_ros_wrapper
    PLUGIN "RobosenseHwMonitorRosWrapper"
    EXECUTABLE robosense_hw_monitor_ros_wrapper_node
)


## Continental

# Hw Interface
add_library(continental_ars548_hw_ros_wrapper SHARED
    src/continental/continental_ars548_hw_interface_ros_wrapper.cpp
)
target_link_libraries(continental_ars548_hw_ros_wrapper PUBLIC
    ${continental_msgs_TARGETS}
    ${continental_srvs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${nebula_msgs_TARGETS}
    ${radar_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_continental
)
target_include_directories(continental_ars548_hw_ros_wrapper PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${continental_srvs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

#ament_target_dependencies(continental_ars548_hw_ros_wrapper
#    continental_msgs
#    diagnostic_msgs
#    nebula_msgs
#    radar_msgs
#)

rclcpp_components_register_node(continental_ars548_hw_ros_wrapper
        PLUGIN "ContinentalARS548HwInterfaceRosWrapper"
        EXECUTABLE continental_ars548_hw_interface_ros_wrapper_node
        )

add_library(multi_continental_ars548_hw_ros_wrapper SHARED
    src/continental/multi_continental_ars548_hw_interface_ros_wrapper.cpp
)
target_link_libraries(multi_continental_ars548_hw_ros_wrapper PUBLIC
    ${continental_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${nebula_msgs_TARGETS}
    ${radar_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    nebula_hw_interfaces::nebula_hw_interfaces_continental
)
target_include_directories(multi_continental_ars548_hw_ros_wrapper PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${nebula_hw_interfaces_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

# Consider changing lin libraries and include directories to ament_target_dependencies
#ament_target_dependencies(multi_continental_ars548_hw_ros_wrapper
#    continental_msgs
#    diagnostic_msgs
#    nebula_msgs
#    radar_msgs
#)

rclcpp_components_register_node(multi_continental_ars548_hw_ros_wrapper
        PLUGIN "MultiContinentalARS548HwInterfaceRosWrapper"
        EXECUTABLE multi_continental_ars548_hw_interface_ros_wrapper_node
        )

# DriverDecoder
add_library(continental_ars548_driver_ros_wrapper SHARED
  src/continental/continental_ars548_decoder_ros_wrapper.cpp
)
target_link_libraries(continental_ars548_driver_ros_wrapper PUBLIC
    ${continental_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${nebula_msgs_TARGETS}
    ${radar_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${visualization_msgs_TARGETS}
    nebula_decoders::nebula_decoders_continental
)
target_include_directories(continental_ars548_driver_ros_wrapper PUBLIC
    ${continental_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${nebula_msgs_INCLUDE_DIRS}
    ${nebula_decoders_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)

rclcpp_components_register_node(continental_ars548_driver_ros_wrapper
    PLUGIN "ContinentalARS548DriverRosWrapper"
    EXECUTABLE continental_ars548_driver_ros_wrapper_node
)

install(TARGETS hesai_hw_ros_wrapper EXPORT export_hesai_hw_ros_wrapper)
install(TARGETS hesai_hw_monitor_ros_wrapper EXPORT export_hesai_hw_monitor_ros_wrapper)
install(TARGETS hesai_driver_ros_wrapper EXPORT export_hesai_driver_ros_wrapper)
install(TARGETS velodyne_hw_ros_wrapper EXPORT export_velodyne_hw_ros_wrapper)
install(TARGETS velodyne_hw_monitor_ros_wrapper EXPORT export_velodyne_hw_monitor_ros_wrapper)
install(TARGETS velodyne_driver_ros_wrapper EXPORT export_velodyne_driver_ros_wrapper)
install(TARGETS robosense_hw_ros_wrapper EXPORT export_robosense_hw_ros_wrapper)
install(TARGETS robosense_hw_monitor_ros_wrapper EXPORT export_robosense_hw_monitor_ros_wrapper)
install(TARGETS robosense_driver_ros_wrapper EXPORT export_robosense_driver_ros_wrapper)
install(TARGETS continental_ars548_hw_ros_wrapper EXPORT export_continental_ars548_hw_ros_wrapper)
install(TARGETS multi_continental_ars548_hw_ros_wrapper EXPORT export_multi_continental_ars548_hw_ros_wrapper)
install(TARGETS continental_ars548_driver_ros_wrapper EXPORT export_continental_ars548_driver_ros_wrapper)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_hesai_hw_ros_wrapper)
ament_export_targets(export_hesai_hw_monitor_ros_wrapper)
ament_export_targets(export_hesai_driver_ros_wrapper)
ament_export_targets(export_velodyne_hw_ros_wrapper)
ament_export_targets(export_velodyne_hw_monitor_ros_wrapper)
ament_export_targets(export_velodyne_driver_ros_wrapper)
ament_export_targets(export_robosense_hw_ros_wrapper)
ament_export_targets(export_robosense_hw_monitor_ros_wrapper)
ament_export_targets(export_robosense_driver_ros_wrapper)
ament_export_targets(export_continental_ars548_hw_ros_wrapper)
ament_export_targets(export_multi_continental_ars548_hw_ros_wrapper)
ament_export_targets(export_continental_ars548_driver_ros_wrapper)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    PCL
    continental_msgs
    continental_srvs
    diagnostic_msgs
    diagnostic_updater
    nebula_common
    nebula_decoders
    nebula_hw_interfaces
    nebula_msgs
    pandar_msgs
    pcl_conversions
    radar_msgs
    rcl
    rclcpp
    rclcpp_components
    robosense_msgs
    sensor_msgs
    tf2_ros
    velodyne_msgs
    visualization_msgs
    yaml-cpp
)

#ament_auto_package(
ament_package()

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
