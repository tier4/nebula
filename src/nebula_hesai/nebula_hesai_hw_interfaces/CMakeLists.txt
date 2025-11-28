cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_hw_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(boost_tcp_driver)
find_package(nebula_core_common REQUIRED)
find_package(nebula_hesai_common REQUIRED)
find_package(nebula_core_hw_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(nebula_hesai_hw_interfaces SHARED
    src/hesai_hw_interface.cpp
)

target_include_directories(nebula_hesai_hw_interfaces PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nebula_hesai_hw_interfaces PUBLIC
    boost_tcp_driver
    nebula_core_common
    nebula_hesai_common
    nebula_core_hw_interfaces
)

install(TARGETS nebula_hesai_hw_interfaces EXPORT export_nebula_hesai_hw_interfaces)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hesai_hw_interfaces)
ament_export_dependencies(
    boost_tcp_driver
    nebula_core_common
    nebula_hesai_common
    nebula_core_hw_interfaces
)

ament_package()
