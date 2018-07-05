file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(myCentPlanner SHARED ${srcs})

target_link_libraries(myCentPlanner myUtils
                                    myContact)
