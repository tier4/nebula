cmake_minimum_required(VERSION 3.20)
project(nebula_seyond_ros)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_ros REQUIRED)
find_package(nebula_seyond_common REQUIRED)
find_package(nebula_seyond_hw_interfaces REQUIRED)
find_package(nebula_seyond_decoders REQUIRED)

add_library(seyond_ros_wrapper SHARED src/seyond_ros_wrapper.cpp)

target_include_directories(
  seyond_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  seyond_ros_wrapper
  PUBLIC nebula_core_common::nebula_core_common
         nebula_core_ros::nebula_core_ros
         nebula_seyond_common::nebula_seyond_common
         nebula_seyond_hw_interfaces::nebula_seyond_hw_interfaces
         nebula_seyond_decoders::nebula_seyond_decoders)

ament_target_dependencies(
  seyond_ros_wrapper
  PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs)

rclcpp_components_register_node(seyond_ros_wrapper PLUGIN "nebula::ros::SeyondRosWrapper" EXECUTABLE seyond_ros_wrapper_node)

install(TARGETS seyond_ros_wrapper
        EXPORT export_nebula_seyond_ros
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config schema DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_seyond_ros)
ament_export_dependencies(nebula_core_common nebula_core_ros nebula_seyond_common nebula_seyond_hw_interfaces nebula_seyond_decoders rclcpp rclcpp_components sensor_msgs)

ament_package()
