cmake_minimum_required(VERSION 3.20)
project(nebula_seyond)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_ros REQUIRED)
find_package(nebula_seyond_common REQUIRED)
find_package(nebula_seyond_hw_interfaces REQUIRED)
find_package(nebula_seyond_decoders REQUIRED)

add_library(
  seyond_ros_wrapper SHARED
  src/seyond_ros_wrapper.cpp
  src/hw_interface_wrapper.cpp
  src/hw_monitor_wrapper.cpp)

target_include_directories(
  seyond_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  seyond_ros_wrapper
  PUBLIC Boost::headers
         nebula_core_common::nebula_core_common
         nebula_core_ros::nebula_core_ros
         nebula_seyond_common::nebula_seyond_common
         nebula_seyond_hw_interfaces::nebula_seyond_hw_interfaces
         nebula_seyond_decoders::nebula_seyond_decoders)

ament_target_dependencies(
  seyond_ros_wrapper
  PUBLIC
  diagnostic_updater
  rclcpp
  rclcpp_components
  sensor_msgs)

rclcpp_components_register_node(
  seyond_ros_wrapper
  PLUGIN "nebula::ros::SeyondRosWrapper"
  EXECUTABLE seyond_ros_wrapper_node)

install(
  TARGETS seyond_ros_wrapper
  EXPORT export_nebula_seyond
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_seyond_ros_wrapper tests/test_seyond_ros_wrapper.cpp)
  target_link_libraries(test_seyond_ros_wrapper seyond_ros_wrapper)
endif()

if(BUILD_EXAMPLES)
  add_executable(seyond_sensor_info examples/seyond_sensor_info.cpp)
  target_link_libraries(
    seyond_sensor_info
    PRIVATE nebula_seyond_common::nebula_seyond_common
            nebula_seyond_hw_interfaces::nebula_seyond_hw_interfaces)
  install(TARGETS seyond_sensor_info DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config schema DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_seyond)
ament_export_dependencies(
  Boost
  diagnostic_updater
  nebula_core_common
  nebula_core_ros
  nebula_seyond_common
  nebula_seyond_hw_interfaces
  nebula_seyond_decoders
  rclcpp
  rclcpp_components
  sensor_msgs)

ament_package()
