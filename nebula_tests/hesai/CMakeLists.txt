ament_auto_add_library(hesai_ros_decoder_test SHARED
        hesai_ros_decoder_test.cpp
        )
target_link_libraries(hesai_ros_decoder_test ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main
        hesai_ros_decoder_test_main.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test
        )
