cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne_common)

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_library(nebula_velodyne_common SHARED
    src/velodyne_calibration_decoder.cpp
)

target_include_directories(nebula_velodyne_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_velodyne_common PUBLIC
    yaml-cpp
)

install(TARGETS nebula_velodyne_common EXPORT export_nebula_velodyne_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_definitions(-D_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../nebula_velodyne_decoders/calibration/")

    ament_add_gtest(test_velodyne_calibration
        tests/test_velodyne_calibration.cpp
    )
    target_link_libraries(test_velodyne_calibration
        nebula_velodyne_common
    )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_velodyne_common)
ament_export_dependencies(nebula_core_common yaml-cpp)

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
