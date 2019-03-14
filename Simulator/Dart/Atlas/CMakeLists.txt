file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_atlas ${srcs})
target_link_libraries(run_atlas ${DART_LIBRARIES}
                                AtlasPnC
                                myUtils)
