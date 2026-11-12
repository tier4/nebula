cmake_minimum_required(VERSION 3.20)
project(nebula_core_ros)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

# Common library for parameter descriptors
add_library(nebula_core_ros SHARED src/parameter_descriptors.cpp)

target_sources(
  nebula_core_ros PRIVATE src/agnocast_wrapper/node.cpp
                          src/agnocast_wrapper/runtime.cpp)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
  find_package(glog REQUIRED)
  # USE_AGNOCAST_ENABLED is PUBLIC because it picks which Node class this library compiles into
  # itself, and consumers must see the same one.
  target_compile_definitions(nebula_core_ros PUBLIC USE_AGNOCAST_ENABLED)
  # glog is for the executables nebula_agnocast_wrapper_register_node() generates, which install
  # a failure signal handler; this library is what carries it to them.
  target_link_libraries(nebula_core_ros PUBLIC agnocastlib::agnocast glog::glog)
endif()

target_include_directories(
  nebula_core_ros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_core_ros PUBLIC rclcpp::rclcpp nebula_core_common::nebula_core_common
                         rosbag2_storage::rosbag2_storage Boost::headers)

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
  target_link_libraries(test_single_consumer_processor
                        nebula_core_common::nebula_core_common rcpputils::rcpputils)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_core_ros)

install(DIRECTORY schema DESTINATION share/${PROJECT_NAME})

# The register_node macro reaches consumers through CONFIG_EXTRAS.
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "cmake/agnocast_wrapper_config_extras.cmake")

install(
  FILES cmake/agnocast_wrapper_config_extras.cmake
        cmake/nebula_agnocast_wrapper_register_node.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

install(DIRECTORY templates DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(Boost diagnostic_msgs diagnostic_updater
                          nebula_core_common rclcpp rosbag2_storage)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_export_dependencies(agnocastlib glog)
endif()

ament_package()
