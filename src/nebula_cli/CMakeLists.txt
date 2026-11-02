cmake_minimum_required(VERSION 3.20)
project(nebula_cli)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_executable(nebula_cli src/main.cpp src/converters.cpp src/params.cpp src/model_detect.cpp)
target_include_directories(nebula_cli PRIVATE src)
target_link_libraries(
  nebula_cli
  PRIVATE ament_index_cpp::ament_index_cpp
          nebula_core_common::nebula_core_common
          nebula_core_ros::nebula_core_ros
          nebula_hesai_decoders::nebula_hesai_decoders
          nebula_velodyne_decoders::nebula_velodyne_decoders
          nebula_robosense_decoders::nebula_robosense_decoders
          nebula_robosense_decoders::nebula_robosense_decoders_info
          nebula_msgs::nebula_msgs__rosidl_typesupport_cpp
          pandar_msgs::pandar_msgs__rosidl_typesupport_cpp
          rclcpp::rclcpp
          robosense_msgs::robosense_msgs__rosidl_typesupport_cpp
          rosbag2_cpp::rosbag2_cpp
          rosbag2_storage::rosbag2_storage
          builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp
          sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
          velodyne_msgs::velodyne_msgs__rosidl_typesupport_cpp)

install(TARGETS nebula_cli DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_params test/test_params.cpp src/params.cpp)
  target_include_directories(test_params PRIVATE src)
  target_link_libraries(test_params rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

  ament_add_gtest(test_model_detect test/test_model_detect.cpp src/model_detect.cpp)
  target_include_directories(test_model_detect PRIVATE src)
  target_link_libraries(test_model_detect
                        nebula_hesai_common::nebula_hesai_common
                        nebula_hesai_decoders::nebula_hesai_decoders)
endif()

ament_package()
