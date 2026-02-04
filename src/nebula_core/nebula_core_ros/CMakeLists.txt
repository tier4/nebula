cmake_minimum_required(VERSION 3.20)
project(nebula_core_ros)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Common library for parameter descriptors
add_library(nebula_core_ros SHARED src/parameter_descriptors.cpp)

target_include_directories(
  nebula_core_ros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_core_ros PUBLIC rclcpp::rclcpp nebula_core_common::nebula_core_common rosbag2_storage::rosbag2_storage)

install(
  TARGETS nebula_core_ros
  EXPORT export_nebula_core_ros
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rcpputils REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_single_consumer_processor
                  test/test_single_consumer_processor.cpp)
  target_include_directories(test_single_consumer_processor PRIVATE include)
  target_link_libraries(test_single_consumer_processor rcpputils::rcpputils)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_core_ros)

install(DIRECTORY schema DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(diagnostic_msgs diagnostic_updater nebula_core_common
                          rclcpp rosbag2_storage)

ament_package()
