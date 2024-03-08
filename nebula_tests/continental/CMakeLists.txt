# Continental ARS548
ament_auto_add_library(continental_ros_decoder_test_ars548 SHARED
continental_ros_decoder_test_ars548.cpp
        )
ament_add_gtest(continental_ros_decoder_test_main_ars548
continental_ros_decoder_test_main_ars548.cpp
        )
target_link_libraries(continental_ros_decoder_test_ars548 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_target_dependencies(continental_ros_decoder_test_main_ars548
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(continental_ros_decoder_test_main_ars548 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/continental
        include
        )
target_link_libraries(continental_ros_decoder_test_main_ars548
        ${PCL_LIBRARIES}
        continental_ros_decoder_test_ars548
        )
