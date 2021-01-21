file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_laikago ${srcs})
target_link_libraries(run_laikago ${DART_LIBRARIES}
                                LaikagoPnC
                                myUtils)
