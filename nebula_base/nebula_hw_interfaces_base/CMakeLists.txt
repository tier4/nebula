cmake_minimum_required(VERSION 3.14)
project(nebula_hw_interfaces_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(boost_tcp_driver)
find_package(boost_udp_driver)
find_package(nebula_common_base REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(nebula_hw_interfaces_base INTERFACE)

target_include_directories(nebula_hw_interfaces_base INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nebula_hw_interfaces_base INTERFACE
    nebula_common_base::nebula_common_base
)

install(TARGETS nebula_hw_interfaces_base EXPORT export_nebula_hw_interfaces_base)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_udp
        test/common/test_udp.cpp
    )

    target_include_directories(test_udp PRIVATE
        include
        test
    )
    target_link_libraries(test_udp
        nebula_common_base::nebula_common_base
    )
endif()

ament_export_targets(export_nebula_hw_interfaces_base)
ament_export_dependencies(
    boost_tcp_driver
    boost_udp_driver
    nebula_common_base
    nebula_msgs
)

ament_package()
