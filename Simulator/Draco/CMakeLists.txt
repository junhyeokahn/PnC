file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_draco ${srcs})
target_link_libraries(run_draco ${DART_LIBRARIES}
                                #DracoPnC
                                myUtils)
