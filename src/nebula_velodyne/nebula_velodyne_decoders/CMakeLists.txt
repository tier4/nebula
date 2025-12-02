cmake_minimum_required(VERSION 3.20)
project(nebula_velodyne_decoders)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Velodyne Decoder
add_library(nebula_velodyne_decoders SHARED
    src/velodyne_driver.cpp
    src/decoders/vls128_decoder.cpp
    src/decoders/vlp16_decoder.cpp
    src/decoders/vlp32_decoder.cpp
)

target_include_directories(nebula_velodyne_decoders PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nebula_velodyne_decoders PUBLIC
    angles
    nebula_core_common
    nebula_velodyne_common
    nebula_core_decoders
    pcl_conversions
    rclcpp
    velodyne_msgs
)

target_link_libraries(nebula_velodyne_decoders PUBLIC
    pcl_common
)

install(TARGETS nebula_velodyne_decoders EXPORT export_nebula_velodyne_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_velodyne_decoders)

ament_export_dependencies(
    PCL
    pcl_conversions
    angles
    nebula_core_common
    nebula_velodyne_common
    nebula_core_decoders
    velodyne_msgs
)

ament_package()
