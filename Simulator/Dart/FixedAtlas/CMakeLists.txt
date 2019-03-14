file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_fixed_atlas ${srcs})
target_link_libraries(run_fixed_atlas ${DART_LIBRARIES}
                                FixedAtlasPnC
                                myUtils)

