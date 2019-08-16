add_executable(test_dummy TestDummy.cpp)
target_link_libraries(test_dummy gtest
                                 gtest_main)
add_test(NAME test_dummy COMMAND test_dummy)

add_subdirectory(TestKalman)
