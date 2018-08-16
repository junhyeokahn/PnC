file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_draco_hip ${srcs})
target_link_libraries(run_draco_hip ${DART_LIBRARIES}
                                    DracoHipPnC
                                    myUtils)
