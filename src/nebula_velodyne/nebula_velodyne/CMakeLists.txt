cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne)

find_package(autoware_cmake REQUIRED)
autoware_package()

# # Velodyne
add_library(
  velodyne_ros_wrapper SHARED
  src/velodyne_ros_wrapper.cpp src/decoder_wrapper.cpp
  src/hw_interface_wrapper.cpp src/hw_monitor_wrapper.cpp)

target_include_directories(
  velodyne_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  velodyne_ros_wrapper
  PUBLIC diagnostic_updater::diagnostic_updater
         nebula_velodyne_decoders::nebula_velodyne_decoders
         nebula_velodyne_hw_interfaces::nebula_velodyne_hw_interfaces
         nebula_core_ros::nebula_core_ros
         pcl_common)

ament_target_dependencies(velodyne_ros_wrapper PUBLIC diagnostic_msgs
                          velodyne_msgs nebula_msgs rclcpp_components)

rclcpp_components_register_node(
  velodyne_ros_wrapper PLUGIN "VelodyneRosWrapper" EXECUTABLE
  velodyne_ros_wrapper_node)

install(
  TARGETS velodyne_ros_wrapper
  EXPORT export_velodyne_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io)

  ament_lint_auto_find_test_dependencies()

  add_definitions(
    -D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
  add_definitions(
    -D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../nebula_velodyne_decoders/calibration/"
  )

  # VLP16 tests
  add_library(velodyne_ros_decoder_test_vlp16 SHARED
              tests/velodyne_ros_decoder_test_vlp16.cpp)
  target_include_directories(velodyne_ros_decoder_test_vlp16
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    velodyne_ros_decoder_test_vlp16
    PRIVATE nebula_core_common::nebula_core_common nebula_core_ros::nebula_core_ros pcl_common pcl_io
            rosbag2_cpp::rosbag2_cpp diagnostic_updater::diagnostic_updater
            nebula_velodyne_decoders::nebula_velodyne_decoders)

  ament_add_gtest(velodyne_ros_decoder_test_main_vlp16
                  tests/velodyne_ros_decoder_test_main_vlp16.cpp)
  target_include_directories(
    velodyne_ros_decoder_test_main_vlp16
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/velodyne include tests)
  target_link_libraries(
    velodyne_ros_decoder_test_main_vlp16
    velodyne_ros_decoder_test_vlp16
    nebula_core_common::nebula_core_common
    nebula_velodyne_common::nebula_velodyne_common
    nebula_velodyne_decoders::nebula_velodyne_decoders
    diagnostic_updater::diagnostic_updater)

  # VLS128 tests
  add_library(velodyne_ros_decoder_test_vls128 SHARED
              tests/velodyne_ros_decoder_test_vls128.cpp)
  target_include_directories(velodyne_ros_decoder_test_vls128 PRIVATE include)
  target_link_libraries(
    velodyne_ros_decoder_test_vls128
    PRIVATE nebula_core_common::nebula_core_common nebula_core_ros::nebula_core_ros pcl_common pcl_io
            rosbag2_cpp::rosbag2_cpp diagnostic_updater::diagnostic_updater
            nebula_velodyne_decoders::nebula_velodyne_decoders)

  ament_add_gtest(velodyne_ros_decoder_test_main_vls128
                  tests/velodyne_ros_decoder_test_main_vls128.cpp)
  target_include_directories(velodyne_ros_decoder_test_main_vls128
                             PRIVATE src/velodyne include tests)
  target_link_libraries(
    velodyne_ros_decoder_test_main_vls128
    velodyne_ros_decoder_test_vls128
    nebula_core_common::nebula_core_common
    nebula_velodyne_common::nebula_velodyne_common
    nebula_velodyne_decoders::nebula_velodyne_decoders
    diagnostic_updater::diagnostic_updater)

  # VLP32 tests
  add_library(velodyne_ros_decoder_test_vlp32 SHARED
              tests/velodyne_ros_decoder_test_vlp32.cpp)
  target_include_directories(velodyne_ros_decoder_test_vlp32 PRIVATE include)
  target_link_libraries(
    velodyne_ros_decoder_test_vlp32
    PRIVATE nebula_core_common::nebula_core_common nebula_core_ros::nebula_core_ros pcl_common pcl_io
            rosbag2_cpp::rosbag2_cpp diagnostic_updater::diagnostic_updater
            nebula_velodyne_decoders::nebula_velodyne_decoders)

  ament_add_gtest(velodyne_ros_decoder_test_main_vlp32
                  tests/velodyne_ros_decoder_test_main_vlp32.cpp)
  target_include_directories(velodyne_ros_decoder_test_main_vlp32
                             PRIVATE src/velodyne include tests)
  target_link_libraries(
    velodyne_ros_decoder_test_main_vlp32
    velodyne_ros_decoder_test_vlp32
    nebula_core_common::nebula_core_common
    nebula_velodyne_common::nebula_velodyne_common
    nebula_velodyne_decoders::nebula_velodyne_decoders
    diagnostic_updater::diagnostic_updater)
endif()

if(BUILD_EXAMPLES)
  find_package(PCL REQUIRED COMPONENTS common io)
  find_package(rosbag2_cpp REQUIRED)

  add_library(velodyne_ros_offline_extract_bag_pcd SHARED
              examples/velodyne_ros_offline_extract_bag_pcd.cpp)
  target_include_directories(velodyne_ros_offline_extract_bag_pcd
                             PRIVATE examples include)
  target_link_libraries(
    velodyne_ros_offline_extract_bag_pcd
    PRIVATE nebula_core_common::nebula_core_common
            pcl_common
            pcl_io
            rosbag2_cpp::rosbag2_cpp
            nebula_velodyne_decoders::nebula_velodyne_decoders
            nebula_core_ros::nebula_core_ros)

  add_executable(velodyne_ros_offline_extract_bag_pcd_node
                 examples/velodyne_ros_offline_extract_bag_pcd_main.cpp)
  target_link_libraries(velodyne_ros_offline_extract_bag_pcd_node
                        PRIVATE velodyne_ros_offline_extract_bag_pcd)

  install(TARGETS velodyne_ros_offline_extract_bag_pcd_node
          DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY test_resources DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY config launch schema DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_velodyne_ros_wrapper)

ament_export_dependencies(
  PCL
  diagnostic_msgs
  diagnostic_updater
  nebula_core_common
  nebula_velodyne_common
  nebula_core_decoders
  nebula_velodyne_decoders
  nebula_core_hw_interfaces
  nebula_velodyne_hw_interfaces
  nebula_msgs
  nebula_core_ros
  rclcpp_components
  velodyne_msgs
  yaml-cpp)

ament_package()
