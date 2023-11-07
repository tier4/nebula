# Innovusion
ament_auto_add_library(innovusion_ros_decoder_test SHARED
        innovusion_ros_decoder_test.cpp
        )
target_link_libraries(innovusion_ros_decoder_test ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(innovusion_ros_decoder_test_main
        innovusion_ros_decoder_test_main.cpp
        )
ament_target_dependencies(innovusion_ros_decoder_test_main
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(innovusion_ros_decoder_test_main PUBLIC
        ${PROJECT_SOURCE_DIR}/src/innovusion
        include
        )
target_link_libraries(innovusion_ros_decoder_test_main
        ${PCL_LIBRARIES}
        innovusion_ros_decoder_test
        )
