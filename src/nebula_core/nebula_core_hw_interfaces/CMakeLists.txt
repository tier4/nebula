cmake_minimum_required(VERSION 3.14)
project(nebula_core_hw_interfaces)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_library(nebula_core_hw_interfaces INTERFACE)

target_include_directories(
  nebula_core_hw_interfaces
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_core_hw_interfaces
                      INTERFACE nebula_core_common::nebula_core_common)

install(TARGETS nebula_core_hw_interfaces
        EXPORT export_nebula_core_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_udp test/common/test_udp.cpp)

  target_include_directories(test_udp PRIVATE include test)
  target_link_libraries(test_udp nebula_core_common::nebula_core_common)
  ament_add_gtest(test_tcp test/common/test_tcp.cpp)
  target_include_directories(test_tcp PRIVATE include test)
  target_link_libraries(test_tcp nebula_core_common::nebula_core_common)

  ament_add_gtest(test_can test/common/test_can.cpp)
  target_include_directories(test_can PRIVATE include test)
  target_link_libraries(test_can nebula_core_common::nebula_core_common)

  ament_add_gtest(test_http_client test/common/test_http_client.cpp)
  target_include_directories(test_http_client PRIVATE include test)
  target_link_libraries(test_http_client nebula_core_common::nebula_core_common)
endif()

ament_export_targets(export_nebula_core_hw_interfaces)
ament_export_dependencies(nebula_core_common)

ament_package()
