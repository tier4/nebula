cmake_minimum_required(VERSION 3.22)
project(nebula_common)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/common/nebula_common.cpp
  src/ros/parameter_descriptors.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_auto_add_gtest(test_rate_checker
    test/common/test_rate_checker.cpp
  )

  ament_auto_add_gtest(test_downsample_mask
    test/decoder/point_filters/test_downsample_mask.cpp
  )

  ament_auto_add_gtest(test_single_consumer_processor
    test/ros/test_single_consumer_processor.cpp
  )
endif()

ament_auto_package()
