cmake_minimum_required(VERSION 3.8)
project(robosense_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RobosensePacket.msg"
        "msg/RobosenseInfoPacket.msg"
        "msg/RobosenseScan.msg"
        DEPENDENCIES
        std_msgs
        )

ament_package()
