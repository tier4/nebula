cmake_minimum_required(VERSION 3.20)
project(nebula_core_ros)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)

# Common library for parameter descriptors
add_library(nebula_core_ros SHARED
    src/parameter_descriptors.cpp
)

target_include_directories(nebula_core_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_core_ros PUBLIC
    rclcpp::rclcpp
    nebula_core_common::nebula_core_common
)

install(TARGETS nebula_core_ros EXPORT export_nebula_core_ros LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rcpputils REQUIRED)

    ament_lint_auto_find_test_dependencies()

    # Unit tests
    ament_add_gtest(test_single_consumer_processor
        tests/test_single_consumer_processor.cpp
    )
    target_link_libraries(test_single_consumer_processor
        rcpputils::rcpputils
        nebula_core_ros
    )

    ament_add_gtest(test_hysteresis_state_machine
        tests/test_hysteresis_state_machine.cpp
    )
    target_link_libraries(test_hysteresis_state_machine
        nebula_core_ros
    )
    ament_target_dependencies(test_hysteresis_state_machine
        diagnostic_msgs
    )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_core_ros)

install(
  DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    diagnostic_msgs
    diagnostic_updater
    nebula_core_common
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
