# Velodyne VLP16
ament_auto_add_library(velodyne_ros_decoder_test_vlp16 SHARED
        velodyne_ros_decoder_test_vlp16.cpp
        )
target_link_libraries(velodyne_ros_decoder_test_vlp16 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(velodyne_ros_decoder_test_main_vlp16
        velodyne_ros_decoder_test_main_vlp16.cpp
        )
ament_target_dependencies(velodyne_ros_decoder_test_main_vlp16
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(velodyne_ros_decoder_test_main_vlp16 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/velodyne
        include
        )
target_link_libraries(velodyne_ros_decoder_test_main_vlp16
        ${PCL_LIBRARIES}
        velodyne_ros_decoder_test_vlp16
        )

# Velodyne VLS128
ament_auto_add_library(velodyne_ros_decoder_test_vls128 SHARED
        velodyne_ros_decoder_test_vls128.cpp
        )
ament_add_gtest(velodyne_ros_decoder_test_main_vls128
        velodyne_ros_decoder_test_main_vls128.cpp
        )
target_link_libraries(velodyne_ros_decoder_test_vls128 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_target_dependencies(velodyne_ros_decoder_test_main_vls128
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(velodyne_ros_decoder_test_main_vls128 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/velodyne
        include
        )
target_link_libraries(velodyne_ros_decoder_test_main_vls128
        ${PCL_LIBRARIES}
        velodyne_ros_decoder_test_vls128
        )
