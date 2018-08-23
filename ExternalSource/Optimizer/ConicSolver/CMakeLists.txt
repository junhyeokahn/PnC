file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(myConicSolver SHARED ${srcs})

target_link_libraries(myConicSolver myYaml)
