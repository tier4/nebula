cmake_minimum_required(VERSION 3.14)
project(nebula_ros_base)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED components common)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
    ${nebula_common_base_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
    ${nebula_common_base_TARGETS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Common library for parameter descriptors
add_library(nebula_ros_base_common SHARED
    src/common/parameter_descriptors.cpp
)

target_include_directories(nebula_ros_base_common PUBLIC
    include
    ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(nebula_ros_base_common PUBLIC
    ${rclcpp_TARGETS}
)

install(TARGETS nebula_ros_base_common EXPORT export_nebula_ros_base_common LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rcpputils REQUIRED)

    ament_lint_auto_find_test_dependencies()

    # Unit tests
    ament_add_gtest(test_single_consumer_processor
        test/test_single_consumer_processor.cpp
    )
    target_include_directories(test_single_consumer_processor
        PRIVATE
        include
        SYSTEM
        ${rcpputils_INCLUDE_DIRS}
    )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")

install(
  DIRECTORY launch schema
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    PCL
    diagnostic_msgs
    diagnostic_updater
    nebula_common_base
    nebula_msgs
    rclcpp
    rclcpp_components
    yaml-cpp
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
