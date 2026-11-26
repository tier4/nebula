cmake_minimum_required(VERSION 3.20)
project(nebula_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Pure (ros2_control-independent) helpers, split out so they can be unit tested on their own.
add_library(nebula_ros2_control_utils STATIC src/driver_config.cpp)
set_target_properties(nebula_ros2_control_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  nebula_ros2_control_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(${PROJECT_NAME} SHARED src/nebula_sensor_interface.cpp
                                   src/component_loader.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PRIVATE nebula_ros2_control_utils)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  ament_index_cpp
  class_loader
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface
                                         nebula_ros2_control_plugins.xml)

install(
  TARGETS ${PROJECT_NAME} nebula_ros2_control_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_driver_config test/test_driver_config.cpp)
  target_link_libraries(test_driver_config nebula_ros2_control_utils)

  # Loads the plugin through pluginlib and runs it through the hardware lifecycle with a test
  # driver component, without any real sensor. The component is not registered in the ament
  # index (so it is never installed as a usable component); the test points the hardware
  # interface at the library directly through the `library` parameter.
  add_library(nebula_ros2_control_test_component SHARED
              test/test_component.cpp)
  find_package(sensor_msgs REQUIRED)
  ament_target_dependencies(nebula_ros2_control_test_component rclcpp
                            rclcpp_components sensor_msgs)

  ament_add_gtest(test_nebula_sensor_interface
                  test/test_nebula_sensor_interface.cpp TIMEOUT 60)
  target_compile_definitions(
    test_nebula_sensor_interface
    PRIVATE
      TEST_COMPONENT_LIBRARY="$<TARGET_FILE:nebula_ros2_control_test_component>"
  )
  target_link_libraries(test_nebula_sensor_interface ${PROJECT_NAME})
  ament_target_dependencies(test_nebula_sensor_interface hardware_interface
                            rclcpp sensor_msgs)
  add_dependencies(test_nebula_sensor_interface
                   nebula_ros2_control_test_component)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  class_loader
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle)

ament_package()
