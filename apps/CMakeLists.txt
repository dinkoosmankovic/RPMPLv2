add_executable(test_state_spaces test_state_spaces.cpp)
target_compile_features(test_state_spaces PRIVATE cxx_std_17)
target_link_libraries(test_state_spaces PUBLIC rpmpl_library)

add_executable(test_rrtconnect test_rrtconnect.cpp)
target_compile_features(test_rrtconnect PRIVATE cxx_std_17)
target_link_libraries(test_rrtconnect PUBLIC rpmpl_library ${PROJECT_LIBRARIES})


add_executable(test_nanoflann test_nanoflann.cpp)
target_compile_features(test_nanoflann PRIVATE cxx_std_17)
target_link_libraries(test_nanoflann PUBLIC rpmpl_library ${PROJECT_LIBRARIES})


add_executable(test_kdl_parser test_kdl_parser.cpp)
target_compile_features(test_kdl_parser PRIVATE cxx_std_17)
target_link_libraries(test_kdl_parser PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_planar_2dof test_planar_2dof.cpp)
target_compile_features(test_planar_2dof PRIVATE cxx_std_17)
target_link_libraries(test_planar_2dof PUBLIC rpmpl_library ${PROJECT_LIBRARIES})

add_executable(test_rrtconnect_fcl test_rrtconnect_fcl.cpp)
target_compile_features(test_rrtconnect_fcl PRIVATE cxx_std_17)
target_link_libraries(test_rrtconnect_fcl PUBLIC rpmpl_library ${PROJECT_LIBRARIES})
