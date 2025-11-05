cmake_minimum_required(VERSION 3.14)
project(nebula_tests)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

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

find_package(ament_cmake_auto REQUIRED)
find_package(nebula_common_base REQUIRED)
find_package(nebula_common_hesai REQUIRED)
find_package(nebula_common_velodyne REQUIRED)
find_package(nebula_common_continental REQUIRED)
find_package(nebula_decoders_base REQUIRED)
find_package(nebula_decoders_hesai REQUIRED)
find_package(nebula_decoders_velodyne REQUIRED)
find_package(nebula_decoders_continental REQUIRED)
find_package(nebula_ros_base REQUIRED)
find_package(nebula_ros_hesai REQUIRED)
find_package(nebula_ros_velodyne REQUIRED)
find_package(nebula_ros_continental REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rosbag2_cpp REQUIRED)
find_package(diagnostic_updater REQUIRED)


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)

    add_definitions(-D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/data/")
    # TODO: FIX this path to point to nebula_decoders/calibration
    add_definitions(-D_SRC_CALIBRATION_DIR_PATH="${PROJECT_SOURCE_DIR}/../nebula_decoders_hesai/calibration/")

    set(NEBULA_TEST_INCLUDE_DIRS
        ${nebula_common_base_INCLUDE_DIRS}
        ${nebula_common_hesai_INCLUDE_DIRS}
        ${nebula_common_velodyne_INCLUDE_DIRS}
        ${nebula_common_continental_INCLUDE_DIRS}
        ${nebula_decoders_base_INCLUDE_DIRS}
        ${nebula_decoders_hesai_INCLUDE_DIRS}
        ${nebula_decoders_velodyne_INCLUDE_DIRS}
        ${nebula_decoders_continental_INCLUDE_DIRS}
        ${nebula_ros_base_INCLUDE_DIRS}
        ${nebula_ros_hesai_INCLUDE_DIRS}
        ${nebula_ros_velodyne_INCLUDE_DIRS}
        ${nebula_ros_continental_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${diagnostic_updater_INCLUDE_DIRS}
    )

    set(NEBULA_TEST_LIBRARIES
        ${nebula_common_base_TARGETS}
        ${nebula_common_hesai_TARGETS}
        ${nebula_common_velodyne_TARGETS}
        ${nebula_common_continental_TARGETS}
        ${PCL_LIBRARIES}
        ${rosbag2_cpp_TARGETS}
        ${diagnostic_updater_TARGETS}
    )

    set(CONTINENTAL_TEST_LIBRARIES
        ${NEBULA_TEST_LIBRARIES}
        nebula_decoders_continental::nebula_decoders_continental
    )

    set(HESAI_TEST_LIBRARIES
        ${NEBULA_TEST_LIBRARIES}
        nebula_decoders_hesai::nebula_decoders_hesai
    )

    set(VELODYNE_TEST_LIBRARIES
        ${NEBULA_TEST_LIBRARIES}
        nebula_decoders_velodyne::nebula_decoders_velodyne
    )

    add_subdirectory(continental)
    add_subdirectory(hesai)
    add_subdirectory(velodyne)

endif()

ament_package()
