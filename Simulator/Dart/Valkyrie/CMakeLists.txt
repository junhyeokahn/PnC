file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_valkyrie ${srcs})
target_link_libraries(run_valkyrie ${DART_LIBRARIES}
                                   ValkyriePnC
                                   myUtils)
