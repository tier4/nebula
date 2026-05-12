cmake_minimum_required(VERSION 3.20)
project(nebula_robosense)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Robosense
add_library(
  robosense_ros_wrapper SHARED
  src/robosense_ros_wrapper.cpp src/decoder_wrapper.cpp
  src/hw_interface_wrapper.cpp src/hw_monitor_wrapper.cpp)

target_include_directories(
  robosense_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  robosense_ros_wrapper
  PUBLIC diagnostic_updater::diagnostic_updater
         nebula_robosense_decoders::nebula_robosense_decoders
         nebula_robosense_decoders::nebula_robosense_decoders_info
         nebula_robosense_hw_interfaces::nebula_robosense_hw_interfaces
         nebula_core_ros::nebula_core_ros
         pcl_common)

ament_target_dependencies(
  robosense_ros_wrapper
  PUBLIC
  diagnostic_msgs
  robosense_msgs
  nebula_msgs
  rclcpp_components
  sensor_msgs
  pcl_conversions)

rclcpp_components_register_node(
  robosense_ros_wrapper PLUGIN "RobosenseRosWrapper" EXECUTABLE
  robosense_ros_wrapper_node)

install(
  TARGETS robosense_ros_wrapper
  EXPORT export_robosense_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch schema DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_robosense_ros_wrapper)

ament_export_dependencies(
  PCL
  diagnostic_msgs
  diagnostic_updater
  nebula_core_common
  nebula_robosense_common
  nebula_core_decoders
  nebula_robosense_decoders
  nebula_core_hw_interfaces
  nebula_robosense_hw_interfaces
  nebula_msgs
  nebula_core_ros
  rclcpp_components
  robosense_msgs
  sensor_msgs
  pcl_conversions
  yaml-cpp)

ament_package()
