file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_fixed_draco ${srcs})
target_link_libraries(run_fixed_draco ${DART_LIBRARIES}
                                      FixedDracoPnC
                                      myUtils)
