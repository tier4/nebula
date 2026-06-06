cmake_minimum_required(VERSION 3.20)
project(nebula_core_common)
# cspell: ignore CPACK ARGN

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_library(nebula_core_common SHARED src/nebula_common.cpp)

target_include_directories(
  nebula_core_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_core_common PUBLIC
  nlohmann_json::nlohmann_json
  Boost::headers)

# Standard CMake install and config generation
include(CMakePackageConfigHelpers)
set(CPACK_PACKAGE_VERSION "1.0.0")

install(TARGETS nebula_core_common
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

  nebula_add_test(test_rate_checker test/test_rate_checker.cpp)
  nebula_add_test(test_core_types test/test_core_types.cpp)

  nebula_add_test(test_pcd_io test/test_pcd_io.cpp)
  target_compile_definitions(test_pcd_io PRIVATE
    NEBULA_CORE_COMMON_TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_resources")
endif()

# ament-specific exports if building with ament
if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(Boost nlohmann_json)
    autoware_package()
    ament_package()
endif()
