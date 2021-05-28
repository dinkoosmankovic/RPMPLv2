# Testing library


include_directories(include)

# Tests need to be added as executables first
add_executable(simple main_test.cpp)

# I'm using C++17 in the test
target_compile_features(simple PRIVATE cxx_std_17)

# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(simple PRIVATE rpmpl_library gtest glog::glog)

# If you register a test, then ctest and make test will run it.
# You can also run examples and check the output, as well.
add_test(NAME simple_test COMMAND simple_test) # Command can be a target
