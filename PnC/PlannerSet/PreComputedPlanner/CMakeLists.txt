file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(myPreComputedPlanner SHARED ${srcs})

target_link_libraries(myPreComputedPlanner myUtils)
