# Velodyne VLP16

add_library(velodyne_ros_decoder_test_vlp16 SHARED
    velodyne_ros_decoder_test_vlp16.cpp
)
target_include_directories(velodyne_ros_decoder_test_vlp16 PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(velodyne_ros_decoder_test_vlp16
    ${VELODYNE_TEST_LIBRARIES})

ament_add_gtest(velodyne_ros_decoder_test_main_vlp16
    velodyne_ros_decoder_test_main_vlp16.cpp
)
ament_target_dependencies(velodyne_ros_decoder_test_main_vlp16
    ${NEBULA_TEST_DEPENDENCIES}
)
target_include_directories(velodyne_ros_decoder_test_main_vlp16 PUBLIC
    ${PROJECT_SOURCE_DIR}/src/velodyne
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(velodyne_ros_decoder_test_main_vlp16
    velodyne_ros_decoder_test_vlp16
)

# Velodyne VLS128
add_library(velodyne_ros_decoder_test_vls128 SHARED
    velodyne_ros_decoder_test_vls128.cpp
)
target_include_directories(velodyne_ros_decoder_test_vls128 PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)
ament_add_gtest(velodyne_ros_decoder_test_main_vls128
    velodyne_ros_decoder_test_main_vls128.cpp
)
target_link_libraries(velodyne_ros_decoder_test_vls128
    ${VELODYNE_TEST_LIBRARIES})

target_include_directories(velodyne_ros_decoder_test_main_vls128 PUBLIC
    ${PROJECT_SOURCE_DIR}/src/velodyne
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(velodyne_ros_decoder_test_main_vls128
    velodyne_ros_decoder_test_vls128
)

# Velodyne VLP32
add_library(velodyne_ros_decoder_test_vlp32 SHARED
    velodyne_ros_decoder_test_vlp32.cpp
)
target_include_directories(velodyne_ros_decoder_test_vlp32 PUBLIC
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(velodyne_ros_decoder_test_vlp32
    ${VELODYNE_TEST_LIBRARIES}
)

ament_add_gtest(velodyne_ros_decoder_test_main_vlp32
    velodyne_ros_decoder_test_main_vlp32.cpp
)

target_include_directories(velodyne_ros_decoder_test_main_vlp32 PUBLIC
    ${PROJECT_SOURCE_DIR}/src/velodyne
    include
    ${NEBULA_TEST_INCLUDE_DIRS}
)
target_link_libraries(velodyne_ros_decoder_test_main_vlp32
    ${PCL_LIBRARIES}
    velodyne_ros_decoder_test_vlp32
)
