cmake_minimum_required(VERSION 3.20)
project(nebula_sample)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_sample_ros_wrapper SHARED src/sample_ros_wrapper.cpp)
target_include_directories(nebula_sample_ros_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nebula_sample_ros_wrapper
  nebula_core_common
  nebula_core_ros
  nebula_sample_common
  nebula_sample_decoders
  nebula_sample_hw_interfaces
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(nebula_sample_ros_wrapper
  PLUGIN "nebula::ros::SampleRosWrapper"
  EXECUTABLE nebula_sample_ros_wrapper_node
)

install(TARGETS nebula_sample_ros_wrapper
  EXPORT export_nebula_sample_ros_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_nebula_sample_ros_wrapper)
ament_export_dependencies(
  nebula_core_common
  nebula_core_ros
  nebula_sample_common
  nebula_sample_decoders
  nebula_sample_hw_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
