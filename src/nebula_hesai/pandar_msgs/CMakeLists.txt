cmake_minimum_required(VERSION 3.14)
project(pandar_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/PandarPacket.msg"
        "msg/PandarScan.msg"
        "msg/PandarJumboPacket.msg"
        DEPENDENCIES
        std_msgs
        )

ament_package()
