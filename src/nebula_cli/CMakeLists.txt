cmake_minimum_required(VERSION 3.20)
project(nebula_cli)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_executable(
  nebula_cli src/main.cpp src/args.cpp src/convert.cpp src/converters.cpp src/params.cpp src/model_detect.cpp
             src/unpack.cpp)
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

  ament_add_gtest(test_args test/test_args.cpp src/args.cpp)
  target_include_directories(test_args PRIVATE src)

  ament_add_gtest(test_unpack test/test_unpack.cpp src/unpack.cpp)
  target_include_directories(test_unpack PRIVATE src)
  target_link_libraries(
    test_unpack
    rclcpp::rclcpp
    rosbag2_storage::rosbag2_storage
    nebula_core_common::nebula_core_common
    nebula_msgs::nebula_msgs__rosidl_typesupport_cpp
    pandar_msgs::pandar_msgs__rosidl_typesupport_cpp
    robosense_msgs::robosense_msgs__rosidl_typesupport_cpp
    velodyne_msgs::velodyne_msgs__rosidl_typesupport_cpp)

  ament_add_gtest(test_converters test/test_converters.cpp src/converters.cpp src/params.cpp)
  target_include_directories(test_converters PRIVATE src)
  target_link_libraries(
    test_converters
    ament_index_cpp::ament_index_cpp
    nebula_core_common::nebula_core_common
    nebula_core_ros::nebula_core_ros
    nebula_hesai_decoders::nebula_hesai_decoders
    nebula_velodyne_decoders::nebula_velodyne_decoders
    nebula_robosense_decoders::nebula_robosense_decoders
    nebula_robosense_decoders::nebula_robosense_decoders_info
    rclcpp::rclcpp)

  ament_add_gtest(
    test_convert test/test_convert.cpp src/convert.cpp src/converters.cpp src/params.cpp
    src/model_detect.cpp src/unpack.cpp)
  target_include_directories(test_convert PRIVATE src)
  target_link_libraries(
    test_convert
    ament_index_cpp::ament_index_cpp
    nebula_core_common::nebula_core_common
    nebula_core_ros::nebula_core_ros
    nebula_hesai_decoders::nebula_hesai_decoders
    nebula_velodyne_decoders::nebula_velodyne_decoders
    nebula_robosense_decoders::nebula_robosense_decoders
    nebula_robosense_decoders::nebula_robosense_decoders_info
    nebula_msgs::nebula_msgs__rosidl_typesupport_cpp
    pandar_msgs::pandar_msgs__rosidl_typesupport_cpp
    robosense_msgs::robosense_msgs__rosidl_typesupport_cpp
    velodyne_msgs::velodyne_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage)
endif()

ament_package()
