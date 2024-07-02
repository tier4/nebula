
add_library(hesai_ros_decoder_test SHARED
    hesai_ros_decoder_test.cpp
)
target_include_directories(hesai_ros_decoder_test PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(hesai_ros_decoder_test
    ${HESAI_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main
    hesai_ros_decoder_test_main.cpp
)

target_include_directories(hesai_ros_decoder_test_main PUBLIC
    ${PROJECT_SOURCE_DIR}/src/hesai
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)

target_link_libraries(hesai_ros_decoder_test_main
    hesai_ros_decoder_test
)

ament_add_gtest(hesai_scan_cutting_test
    hesai_scan_cutting_test.cpp
)

target_link_libraries(hesai_scan_cutting_test
    ${HESAI_TEST_LIBRARIES})

target_include_directories(hesai_scan_cutting_test PUBLIC
    ${PROJECT_SOURCE_DIR}/src/hesai
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)
