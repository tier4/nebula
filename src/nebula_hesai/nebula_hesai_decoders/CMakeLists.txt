cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_decoders)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_hesai_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(rclcpp REQUIRED)

# Hesai Decoder
add_library(nebula_hesai_decoders SHARED
    src/hesai_driver.cpp
)

target_include_directories(nebula_hesai_decoders PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_hesai_decoders PUBLIC
    nebula_core_common::nebula_core_common
    nebula_hesai_common::nebula_hesai_common
    nebula_core_decoders::nebula_core_decoders
    rclcpp::rclcpp
)

install(TARGETS nebula_hesai_decoders EXPORT export_nebula_hesai_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_hesai_packet
        tests/test_hesai_packet.cpp
    )
    target_link_libraries(test_hesai_packet
        nebula_hesai_decoders
    )

    ament_add_gtest(test_packet_loss_detector
        tests/test_packet_loss_detector.cpp
    )
    target_link_libraries(test_packet_loss_detector
        nebula_hesai_decoders
    )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hesai_decoders)

ament_export_dependencies(
    nebula_core_common
    nebula_hesai_common
    nebula_core_decoders
    rclcpp
)

ament_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
