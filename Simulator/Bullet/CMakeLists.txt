file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_test ${srcs})
target_link_libraries(run_test ${BULLET_LIBRARIES}
                                myUtils)
