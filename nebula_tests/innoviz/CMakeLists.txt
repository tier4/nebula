# Innoviz Raven
ament_auto_add_library(innoviz_ros_decoder_test_raven SHARED
        innoviz_ros_decoder_test_raven.cpp
        )
target_link_libraries(innoviz_ros_decoder_test_raven ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(innoviz_ros_decoder_test_main_raven
        innoviz_ros_decoder_test_main_raven.cpp
        )
ament_target_dependencies(innoviz_ros_decoder_test_main_raven
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(innoviz_ros_decoder_test_main_raven PUBLIC
        ${PROJECT_SOURCE_DIR}/src/innoviz
        include
        )
target_link_libraries(innoviz_ros_decoder_test_main_raven
        ${PCL_LIBRARIES}
        innoviz_ros_decoder_test_raven
        )