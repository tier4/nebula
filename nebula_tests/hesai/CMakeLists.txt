# Pandar AT128
ament_auto_add_library(hesai_ros_decoder_test_at128 SHARED
        hesai_ros_decoder_test_at128.cpp
        )
target_link_libraries(hesai_ros_decoder_test_at128 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_at128
        hesai_ros_decoder_test_main_at128.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_at128
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main_at128 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_at128
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_at128
        )

# Pandar XT32M
ament_auto_add_library(hesai_ros_decoder_test_xt32m SHARED
        hesai_ros_decoder_test_xt32m.cpp
        )
target_link_libraries(hesai_ros_decoder_test_xt32m ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_xt32m
        hesai_ros_decoder_test_main_xt32m.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_xt32m
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main_xt32m PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_xt32m
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_xt32m
        )

# Pandar 40P
ament_auto_add_library(hesai_ros_decoder_test_40p SHARED
        hesai_ros_decoder_test_40p.cpp
        )

target_link_libraries(hesai_ros_decoder_test_40p ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_40p
        hesai_ros_decoder_test_main_40p.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_40p
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main_40p PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_40p
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_40p
        )

# Pandar 64
ament_auto_add_library(hesai_ros_decoder_test_64 SHARED
        hesai_ros_decoder_test_64.cpp
        )

target_link_libraries(hesai_ros_decoder_test_64 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_64
        hesai_ros_decoder_test_main_64.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_64
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main_64 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_64
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_64
        )

# Pandar QT64
ament_auto_add_library(hesai_ros_decoder_test_qt64 SHARED
        hesai_ros_decoder_test_qt64.cpp
        )

target_link_libraries(hesai_ros_decoder_test_qt64 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_qt64
        hesai_ros_decoder_test_main_qt64.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_qt64
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(hesai_ros_decoder_test_main_qt64 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_qt64
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_qt64
        )

# Pandar XT32
ament_auto_add_library(hesai_ros_decoder_test_xt32 SHARED
        hesai_ros_decoder_test_xt32.cpp
        )

target_link_libraries(hesai_ros_decoder_test_xt32 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(hesai_ros_decoder_test_main_xt32
        hesai_ros_decoder_test_main_xt32.cpp
        )

ament_target_dependencies(hesai_ros_decoder_test_main_xt32
        ${NEBULA_TEST_DEPENDENCIES}
        )

target_include_directories(hesai_ros_decoder_test_main_xt32 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/hesai
        include
        )

target_link_libraries(hesai_ros_decoder_test_main_xt32
        ${PCL_LIBRARIES}
        hesai_ros_decoder_test_xt32
        )
