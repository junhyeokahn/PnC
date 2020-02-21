add_executable(test_dummy TestDummy.cpp)
target_link_libraries(test_dummy gtest
                                 gtest_main
                                 pthread)
add_test(NAME test_dummy COMMAND test_dummy)

add_executable(test_my_yaml TestYaml.cpp)
target_link_libraries(test_my_yaml myUtils
                                   gtest
                                   gtest_main
                                   pthread)

add_test(NAME test_my_yaml COMMAND test_my_yaml)

add_subdirectory(TestKalman)
add_subdirectory(TestIHWBC)

add_subdirectory(TestMPC)