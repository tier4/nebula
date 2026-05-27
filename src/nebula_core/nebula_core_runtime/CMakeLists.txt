cmake_minimum_required(VERSION 3.20)
project(nebula_core_runtime)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nebula_core_hw_interfaces REQUIRED)

add_library(nebula_core_runtime SHARED
  src/sensor_registry.cpp
  src/packet_router.cpp
  src/replay_session_runner.cpp
  src/live_transport_graph.cpp
)

target_include_directories(
  nebula_core_runtime
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_core_runtime PUBLIC
  nlohmann_json::nlohmann_json
  Boost::filesystem
  Boost::system
  nebula_core_common::nebula_core_common
  nebula_core_decoders::nebula_core_decoders
  nebula_core_hw_interfaces::nebula_core_hw_interfaces
  ${CMAKE_DL_LIBS}
)

# Standard CMake install and config generation
include(CMakePackageConfigHelpers)
set(CPACK_PACKAGE_VERSION "1.0.0")

install(TARGETS nebula_core_runtime
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
    target_link_libraries(${name} ${PROJECT_NAME} Boost::filesystem Boost::system)
    target_compile_definitions(${name} PRIVATE
      NEBULA_TEST_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")
  endfunction()

  nebula_add_test(test_sensor_registry test/test_sensor_registry.cpp)
  nebula_add_test(test_sample_plugin_integration test/test_sample_plugin_integration.cpp)
  nebula_add_test(test_packet_router test/test_packet_router.cpp)
  nebula_add_test(test_replay_session_runner test/test_replay_session_runner.cpp)
  nebula_add_test(test_live_transport_graph test/test_live_transport_graph.cpp)
endif()

if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(Boost nlohmann_json nebula_core_common nebula_core_decoders nebula_core_hw_interfaces)
    autoware_package()
    ament_package()
endif()
