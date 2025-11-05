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

# Common includes for all targets
include_directories(
    ${nebula_common_base_INCLUDE_DIRS}
    include
    SYSTEM
)

link_libraries(
    ${nebula_common_base_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_udp
        test/common/test_udp.cpp
    )

    target_include_directories(test_udp PUBLIC
        ${nebula_common_base_INCLUDE_DIRS}
        include
        test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
    boost_tcp_driver
    boost_udp_driver
    nebula_common_base
    nebula_msgs
)

ament_package()
