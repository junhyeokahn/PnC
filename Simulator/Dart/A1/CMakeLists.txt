file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_a1 ${srcs})
target_link_libraries(run_a1 ${DART_LIBRARIES}
#                                 A1PnC
                                myUtils)
