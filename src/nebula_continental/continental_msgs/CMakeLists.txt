cmake_minimum_required(VERSION 3.5)
project(continental_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/ContinentalArs548Detection.msg"
        "msg/ContinentalArs548DetectionList.msg"
        "msg/ContinentalArs548Object.msg"
        "msg/ContinentalArs548ObjectList.msg"
        "msg/ContinentalSrr520Detection.msg"
        "msg/ContinentalSrr520DetectionList.msg"
        "msg/ContinentalSrr520Object.msg"
        "msg/ContinentalSrr520ObjectList.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        )

ament_package()
