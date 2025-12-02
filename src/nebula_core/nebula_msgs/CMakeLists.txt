cmake_minimum_required(VERSION 3.5)
project(nebula_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/NebulaPacket.msg"
        "msg/NebulaPackets.msg"
        DEPENDENCIES
        std_msgs
        )

ament_package()
