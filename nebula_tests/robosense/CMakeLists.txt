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


# Robosense Bpearl
ament_auto_add_library(robosense_ros_decoder_test_bpearl SHARED
robosense_ros_decoder_test_bpearl.cpp
        )
target_link_libraries(robosense_ros_decoder_test_bpearl ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_bpearl
robosense_ros_decoder_test_main_bpearl.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_bpearl
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_bpearl PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_bpearl
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_bpearl
        )

# Robosense M1Plus
ament_auto_add_library(robosense_ros_decoder_test_m1plus SHARED
robosense_ros_decoder_test_m1plus.cpp
        )
target_link_libraries(robosense_ros_decoder_test_m1plus ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_m1plus
robosense_ros_decoder_test_main_m1plus.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_m1plus
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_m1plus PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_m1plus
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_m1plus
        )

# Robosense RubyPlus
ament_auto_add_library(robosense_ros_decoder_test_rubyplus SHARED
robosense_ros_decoder_test_rubyplus.cpp
        )
target_link_libraries(robosense_ros_decoder_test_rubyplus ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_rubyplus
robosense_ros_decoder_test_main_rubyplus.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_rubyplus
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_rubyplus PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_rubyplus
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_rubyplus
        )

 # Robosense E1
ament_auto_add_library(robosense_ros_decoder_test_e1 SHARED
robosense_ros_decoder_test_e1.cpp
        )
target_link_libraries(robosense_ros_decoder_test_e1 ${PCL_LIBRARIES} ${NEBULA_TEST_LIBRARIES})

ament_add_gtest(robosense_ros_decoder_test_main_e1
robosense_ros_decoder_test_main_e1.cpp
        )
ament_target_dependencies(robosense_ros_decoder_test_main_e1
        ${NEBULA_TEST_DEPENDENCIES}
        )
target_include_directories(robosense_ros_decoder_test_main_e1 PUBLIC
        ${PROJECT_SOURCE_DIR}/src/robosense
        include
        )
target_link_libraries(robosense_ros_decoder_test_main_e1
        ${PCL_LIBRARIES}
        robosense_ros_decoder_test_e1
        )