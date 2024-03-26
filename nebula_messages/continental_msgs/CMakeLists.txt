cmake_minimum_required(VERSION 3.5)
project(continental_msgs)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
