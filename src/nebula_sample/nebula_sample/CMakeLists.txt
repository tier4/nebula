cmake_minimum_required(VERSION 3.20)
project(nebula_sample)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING OR BUILD_EXAMPLES)
  find_package(PCL REQUIRED components common io)
else()
  find_package(PCL REQUIRED components common)
endif()

# Sample
add_library(
  sample_ros_wrapper SHARED
  src/sample_ros_wrapper.cpp)

target_include_directories(
  sample_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  sample_ros_wrapper
  PUBLIC nebula_sample_decoders::nebula_sample_decoders
         nebula_sample_hw_interfaces::nebula_sample_hw_interfaces
         nebula_core_ros::nebula_core_ros
         pcl_common)
ament_target_dependencies(
  sample_ros_wrapper
  PUBLIC
  sensor_msgs
  pcl_conversions)

rclcpp_components_register_node(sample_ros_wrapper PLUGIN "nebula::ros::SampleRosWrapper"
                                EXECUTABLE sample_ros_wrapper_node)

install(
  TARGETS sample_ros_wrapper
  EXPORT export_sample_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_sample_ros_wrapper)

ament_export_dependencies(
  PCL
  nebula_core_common
  nebula_sample_common
  nebula_core_decoders
  nebula_sample_decoders
  nebula_core_hw_interfaces
  nebula_sample_hw_interfaces
  nebula_core_ros
  rclcpp_components
  sensor_msgs
  pcl_conversions)

ament_package()
