cmake_minimum_required(VERSION 3.20)
project(nebula_hesai)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

if("$ENV{ENABLE_AGNOCAST}" AND NOT agnocastlib_FOUND)
  message(FATAL_ERROR "agnocastlib is required when Agnocast is enabled")
endif()
if("$ENV{ENABLE_AGNOCAST}" AND NOT agnocast_components_FOUND)
  message(FATAL_ERROR "agnocast_components is required when Agnocast is enabled")
endif()

# Hesai
add_library(
  hesai_ros_wrapper SHARED
  src/hesai_ros_wrapper.cpp src/decoder_wrapper.cpp
  src/hw_interface_wrapper.cpp src/hw_monitor_wrapper.cpp)

target_include_directories(
  hesai_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  hesai_ros_wrapper
  PUBLIC Boost::headers
         diagnostic_updater::diagnostic_updater
         nebula_hesai_decoders::nebula_hesai_decoders
         nebula_hesai_hw_interfaces::nebula_hesai_hw_interfaces
         nebula_core_ros::nebula_core_ros)

ament_target_dependencies(
  hesai_ros_wrapper
  PUBLIC
  autoware_internal_debug_msgs
  autoware_utils_debug
  diagnostic_msgs
  nebula_msgs
  pandar_msgs
  sensor_msgs
  sync_tooling_msgs
  rclcpp_components)

if("$ENV{ENABLE_AGNOCAST}")
  message(STATUS "Agnocast support is enabled")
  ament_target_dependencies(hesai_ros_wrapper PUBLIC agnocastlib)
  add_definitions(-DUSE_AGNOCAST_ENABLED)
endif()

if("$ENV{ENABLE_AGNOCAST}")
  agnocast_components_register_node(
    hesai_ros_wrapper PLUGIN "HesaiRosWrapper"
    EXECUTABLE hesai_ros_wrapper_node
    EXECUTOR CallbackIsolatedAgnocastExecutor)
else()
  rclcpp_components_register_node(hesai_ros_wrapper PLUGIN "HesaiRosWrapper"
                                  EXECUTABLE hesai_ros_wrapper_node)
endif()

install(
  TARGETS hesai_ros_wrapper
  EXPORT export_hesai_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosbag2_cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_definitions(
    -D_TEST_RESOURCES_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
  add_definitions(
    -D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test_resources/")
  add_definitions(
    -D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../nebula_hesai_decoders/calibration/"
  )

  # Functional safety test
  ament_add_gtest(test_hesai_functional_safety
                  tests/test_hesai_functional_safety.cpp)
  target_include_directories(test_hesai_functional_safety PRIVATE include)
  target_link_libraries(test_hesai_functional_safety hesai_ros_wrapper)

  # Decoder tests
  add_library(hesai_ros_decoder_test SHARED tests/hesai_ros_decoder_test.cpp)
  target_link_libraries(
    hesai_ros_decoder_test
    PRIVATE nebula_core_common::nebula_core_common
            nebula_core_ros::nebula_core_ros
            rosbag2_cpp::rosbag2_cpp
            diagnostic_updater::diagnostic_updater
            nebula_hesai_decoders::nebula_hesai_decoders)
  ament_target_dependencies(hesai_ros_decoder_test PUBLIC pandar_msgs)

  ament_add_gtest(hesai_ros_decoder_test_main
                  tests/hesai_ros_decoder_test_main.cpp)
  target_include_directories(hesai_ros_decoder_test_main PRIVATE tests)
  target_link_libraries(
    hesai_ros_decoder_test_main
    hesai_ros_decoder_test
    nebula_core_common::nebula_core_common
    nebula_hesai_common::nebula_hesai_common
    nebula_hesai_decoders::nebula_hesai_decoders
    diagnostic_updater::diagnostic_updater)

  ament_add_gtest(hesai_ros_scan_cutting_test_main
                  tests/hesai_ros_scan_cutting_test_main.cpp)
  target_include_directories(hesai_ros_scan_cutting_test_main PRIVATE tests)
  target_link_libraries(
    hesai_ros_scan_cutting_test_main
    hesai_ros_decoder_test
    nebula_core_common::nebula_core_common
    nebula_core_ros::nebula_core_ros
    nebula_hesai_common::nebula_hesai_common
    nebula_hesai_decoders::nebula_hesai_decoders
    diagnostic_updater::diagnostic_updater)
endif()

if(BUILD_EXAMPLES)
  find_package(rosbag2_cpp REQUIRED)

  # Offline PCD extraction library
  add_library(hesai_ros_offline_extract_pcd SHARED
              examples/hesai_ros_offline_extract_pcd.cpp)
  target_include_directories(
    hesai_ros_offline_extract_pcd PRIVATE examples
                                          ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    hesai_ros_offline_extract_pcd
    PRIVATE nebula_core_common::nebula_core_common
            rosbag2_cpp::rosbag2_cpp
            nebula_hesai_decoders::nebula_hesai_decoders
            nebula_core_ros::nebula_core_ros)

  add_executable(hesai_ros_offline_extract_pcd_node
                 examples/hesai_ros_offline_extract_pcd_main.cpp)
  target_link_libraries(hesai_ros_offline_extract_pcd_node
                        PRIVATE hesai_ros_offline_extract_pcd)

  # Bag PCD extraction library
  add_library(hesai_ros_offline_extract_bag_pcd SHARED
              examples/hesai_ros_offline_extract_bag_pcd.cpp)
  target_include_directories(
    hesai_ros_offline_extract_bag_pcd
    PRIVATE examples ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    hesai_ros_offline_extract_bag_pcd
    PRIVATE nebula_core_common::nebula_core_common
            rosbag2_cpp::rosbag2_cpp
            nebula_hesai_decoders::nebula_hesai_decoders
            nebula_core_ros::nebula_core_ros)

  add_executable(hesai_ros_offline_extract_bag_pcd_node
                 examples/hesai_ros_offline_extract_bag_pcd_main.cpp)
  target_link_libraries(hesai_ros_offline_extract_bag_pcd_node
                        PRIVATE hesai_ros_offline_extract_bag_pcd)

  install(TARGETS hesai_ros_offline_extract_pcd_node
                  hesai_ros_offline_extract_bag_pcd_node
          DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY config launch schema DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  install(DIRECTORY test_resources DESTINATION share/${PROJECT_NAME})
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_hesai_ros_wrapper)

ament_export_dependencies(
  Boost
  diagnostic_msgs
  diagnostic_updater
  nebula_core_common
  nebula_hesai_common
  nebula_core_decoders
  nebula_hesai_decoders
  nebula_core_hw_interfaces
  nebula_hesai_hw_interfaces
  nebula_msgs
  nebula_core_ros
  pandar_msgs
  rclcpp_components
  sensor_msgs
  sync_tooling_msgs
  autoware_utils_debug
  autoware_internal_debug_msgs)

if("$ENV{ENABLE_AGNOCAST}")
  ament_export_dependencies(agnocastlib)
  ament_export_dependencies(agnocast_components)
endif()

ament_package()
