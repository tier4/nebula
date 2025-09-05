cmake_minimum_required(VERSION 3.14)
project(nebula_common)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_rate_checker
        test/test_rate_checker.cpp
    )
    target_link_libraries(test_rate_checker
        nebula_common
    )
endif()

include_directories(
    include
    SYSTEM
    ${YAML_CPP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIRS}
)

link_libraries(
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${NLOHMANN_JSON_LIBRARIES}
)

add_library(nebula_common SHARED
    src/nebula_common.cpp
    src/velodyne/velodyne_calibration_decoder.cpp
)

install(TARGETS nebula_common EXPORT export_nebula_common)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_common)

ament_export_dependencies(
    PCL
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
