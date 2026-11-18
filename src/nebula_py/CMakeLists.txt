cmake_minimum_required(VERSION 3.20)
project(nebula_py)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# Messages cross the language boundary as CDR bytes, so only this module links rclcpp (for
# rclcpp::Serialization). The decoders it wraps do not.
pybind11_add_module(_hesai_pybind src/hesai_pybind.cpp)

target_link_libraries(
  _hesai_pybind
  PRIVATE nebula_core_common::nebula_core_common
          nebula_core_ros::nebula_core_ros
          nebula_hesai_common::nebula_hesai_common
          nebula_hesai_decoders::nebula_hesai_decoders
          rclcpp::rclcpp)

ament_target_dependencies(_hesai_pybind PUBLIC nebula_msgs pandar_msgs sensor_msgs)

ament_get_python_install_dir(python_install_dir)
install(TARGETS _hesai_pybind
        DESTINATION "${python_install_dir}/${PROJECT_NAME}")
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_hesai_decoder test/test_hesai_decoder.py)
endif()

ament_package()
