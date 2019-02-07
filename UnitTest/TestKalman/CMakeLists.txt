FILE(GLOB_RECURSE sources *.cpp)

add_executable(test_kalman ${sources})
target_link_libraries(test_kalman myFilters)
