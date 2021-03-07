add_executable(main_app main_app.cpp)
target_compile_features(main_app PRIVATE cxx_std_17)


target_link_libraries(main_app PUBLIC rpmpl_library)
