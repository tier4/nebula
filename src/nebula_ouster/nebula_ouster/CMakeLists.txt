cmake_minimum_required(VERSION 3.20)
project(nebula_ouster)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ouster
add_library(
  ouster_ros_wrapper SHARED
  src/ouster_ros_wrapper.cpp)

set(_nebula_ouster_sdk "${CMAKE_CURRENT_SOURCE_DIR}/../../../ouster-sdk")
if(NOT EXISTS "${_nebula_ouster_sdk}/CMakeLists.txt")
  message(
    FATAL_ERROR
      "nebula_ouster_decoders: bundled ouster-sdk is missing (${_nebula_ouster_sdk})")
endif()

set(BUILD_SENSOR ON CACHE BOOL "Ouster SDK: sensor module" FORCE)
set(BUILD_PCAP OFF CACHE BOOL "Ouster SDK: pcap" FORCE)
set(BUILD_OSF OFF CACHE BOOL "Ouster SDK: osf" FORCE)
set(BUILD_VIZ OFF CACHE BOOL "Ouster SDK: visualizer" FORCE)
set(BUILD_MAPPING OFF CACHE BOOL "Ouster SDK: mapping" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Ouster SDK: tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Ouster SDK: examples" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Ouster SDK: python" FORCE)

add_subdirectory("${_nebula_ouster_sdk}" "${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk-build")

if(TARGET ouster_client)
  target_compile_options(ouster_client PRIVATE -Wno-pedantic)
endif()
if(TARGET ouster_sensor)
  target_compile_options(ouster_sensor PRIVATE -Wno-pedantic)
endif()

target_include_directories(
  ouster_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${_nebula_ouster_sdk}/ouster_client/include/optional-lite>
         $<BUILD_INTERFACE:${_nebula_ouster_sdk}/ouster_sensor/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ouster_ros_wrapper
  PUBLIC diagnostic_updater::diagnostic_updater
         nebula_ouster_decoders::nebula_ouster_decoders
         nebula_ouster_hw_interfaces::nebula_ouster_hw_interfaces
         nebula_core_ros::nebula_core_ros
  PRIVATE ouster_client
          ouster_sensor)
ament_target_dependencies(
  ouster_ros_wrapper
  PUBLIC
  diagnostic_msgs
  nebula_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs)

rclcpp_components_register_node(ouster_ros_wrapper PLUGIN "nebula::ros::OusterRosWrapper"
                                EXECUTABLE ouster_ros_wrapper_node)

install(
  TARGETS ouster_ros_wrapper
  EXPORT export_ouster_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_ouster_ros_wrapper)

ament_export_dependencies(
  diagnostic_msgs
  diagnostic_updater
  nebula_core_common
  nebula_ouster_common
  nebula_core_decoders
  nebula_ouster_decoders
  nebula_core_hw_interfaces
  nebula_ouster_hw_interfaces
  nebula_core_ros
  nebula_msgs
  rclcpp_components
  sensor_msgs
  std_msgs)

ament_package()
