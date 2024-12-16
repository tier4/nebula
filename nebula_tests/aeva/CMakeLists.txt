ament_add_gtest(aeva_test_main
    aeva_test_main.cpp
)

target_include_directories(aeva_test_main PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)

target_link_libraries(aeva_test_main
    ${AEVA_TEST_LIBRARIES}
)