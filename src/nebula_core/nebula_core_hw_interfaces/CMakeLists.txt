cmake_minimum_required(VERSION 3.14)
project(nebula_core_hw_interfaces)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(nebula_core_common REQUIRED)

add_library(nebula_core_hw_interfaces SHARED
  src/pcap_packet_source.cpp
  src/udp_packet_source.cpp
  src/can_packet_source.cpp
  src/tcp_packet_source.cpp
  src/http_packet_source.cpp
)

target_include_directories(
  nebula_core_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${PCAP_INCLUDE_DIRS})

target_link_libraries(nebula_core_hw_interfaces
                      PUBLIC nebula_core_common::nebula_core_common
                             Boost::headers
                      PRIVATE ${PCAP_LIBRARIES})

# Standard CMake install and config generation
include(CMakePackageConfigHelpers)
set(CPACK_PACKAGE_VERSION "1.0.0")

install(TARGETS nebula_core_hw_interfaces
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${CPACK_PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
  if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  else()
    find_package(GTest REQUIRED)
    enable_testing()
  endif()

  function(nebula_add_test name)
    if(DEFINED ENV{AMENT_PREFIX_PATH})
      ament_add_gtest(${name} ${ARGN})
    else()
      add_executable(${name} ${ARGN})
      target_link_libraries(${name} GTest::gtest_main)
      add_test(NAME ${name} COMMAND ${name})
    endif()
    target_link_libraries(${name} ${PROJECT_NAME})
    target_include_directories(${name} PRIVATE test)
  endfunction()

  nebula_add_test(test_udp test/common/test_udp.cpp)
  nebula_add_test(test_can test/common/test_can.cpp)
  nebula_add_test(test_tcp test/common/test_tcp.cpp)
  nebula_add_test(test_http_client test/common/test_http_client.cpp)
endif()

if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(nebula_core_common)
    autoware_package()
    ament_package()
endif()
