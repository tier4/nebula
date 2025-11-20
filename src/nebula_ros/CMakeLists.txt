cmake_minimum_required(VERSION 3.20)
project(nebula_ros)
set(ignoreMe "${CMAKE_EXPORT_COMPILE_COMMANDS}")

find_package(ament_cmake REQUIRED)

if (BUILD_TESTING)
    find_package(ros_testing REQUIRED)

    # Smoke tests for all sensor models
    foreach(MODEL Pandar40P Pandar64 PandarQT64 PandarQT128 Pandar128E4X PandarAT128 PandarXT16 PandarXT32 PandarXT32M ARS548 SRR520 VLP16 VLP32 VLS128 Helios Bpearl)
        string(TOLOWER ${MODEL}_smoke_test test_name)
        add_ros_test(
            test/smoke_test.py
            TARGET ${test_name}
            ARGS "launch_file_path:=${PROJECT_SOURCE_DIR}/launch/nebula_launch.py" "sensor_model:=${MODEL}"
            TIMEOUT "10"
        )
    endforeach()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
