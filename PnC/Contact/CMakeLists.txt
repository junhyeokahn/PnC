file(GLOB_RECURSE srcs "*.cpp" "*.hpp")
add_library(myContact SHARED ${srcs})
target_link_libraries(myContact ${DART_LIBRARIES}
                                myPolytope)
