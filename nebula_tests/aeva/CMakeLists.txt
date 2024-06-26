add_library(aeva_hw_interface_test SHARED
    aeva_hw_interface_test.cpp)

target_include_directories(aeva_hw_interface_test PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)

target_link_libraries(aeva_hw_interface_test
    ${AEVA_TEST_LIBRARIES}
)

ament_add_gtest(aeva_test_main
    aeva_test_main.cpp
)

target_include_directories(aeva_test_main PUBLIC
    ${PROJECT_SOURCE_DIR}/src/aeva
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)

target_link_libraries(aeva_test_main
aeva_hw_interface_test
)
