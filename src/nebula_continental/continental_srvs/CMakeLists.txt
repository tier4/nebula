cmake_minimum_required(VERSION 3.5)
project(continental_srvs)

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
        "srv/ContinentalArs548SetSensorMounting.srv"
        "srv/ContinentalArs548SetVehicleParameters.srv"
        "srv/ContinentalArs548SetRadarParameters.srv"
        "srv/ContinentalArs548SetNetworkConfiguration.srv"
        "srv/ContinentalSrr520SetRadarParameters.srv"
        DEPENDENCIES
        std_msgs
        )

ament_package()
