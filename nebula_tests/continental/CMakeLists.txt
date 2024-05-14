# Continental ARS548
add_library(continental_ros_decoder_test_ars548 SHARED
    continental_ros_decoder_test_ars548.cpp
)

target_include_directories(continental_ros_decoder_test_ars548 PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)

target_link_libraries(continental_ros_decoder_test_ars548
    ${PCL_LIBRARIES}
    ${CONTINENTAL_TEST_LIBRARIES}
)

ament_add_gtest(continental_ros_decoder_test_main_ars548
    continental_ros_decoder_test_main_ars548.cpp
)

target_include_directories(continental_ros_decoder_test_main_ars548 PUBLIC
    ${PROJECT_SOURCE_DIR}/src/continental
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(continental_ros_decoder_test_main_ars548
    ${PCL_LIBRARIES}
    continental_ros_decoder_test_ars548
)
