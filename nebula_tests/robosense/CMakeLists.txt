# Robosense Helios16P
ament_auto_add_library(robosense_ros_decoder_test_helios16P SHARED
robosense_ros_decoder_test_helios16P.cpp
        )
target_link_libraries(robosense_ros_decoder_test_helios16P ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_helios16P
robosense_ros_decoder_test_main_helios16P.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_helios16P
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_helios16P PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_helios16P
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_helios16P
        )

# Robosense Helios
ament_auto_add_library(robosense_ros_decoder_test_helios SHARED
robosense_ros_decoder_test_helios.cpp
        )
target_link_libraries(robosense_ros_decoder_test_helios ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_helios
robosense_ros_decoder_test_main_helios.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_helios
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_helios PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_helios
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_helios
        )


