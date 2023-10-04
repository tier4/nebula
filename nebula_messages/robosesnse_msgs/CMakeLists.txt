cmake_minimum_required(VERSION 3.8)
project(robosesnse_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MsopPacket.msg"
        "msg/DifopPacket.msg"
        "msg/RobosenseScan.msg"
        DEPENDENCIES
        std_msgs
        )

ament_package()
