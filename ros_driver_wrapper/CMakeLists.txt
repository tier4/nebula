cmake_minimum_required(VERSION 3.5)
project(lidar_ros_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_driver_wrapper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/check_config.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/livox_config.cpp
)

ament_auto_add_executable(lidar_driver_node
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_driver_wrapper_main.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lidar_driver::RosDriverWrapper"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)

