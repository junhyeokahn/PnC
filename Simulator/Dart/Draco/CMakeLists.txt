file(GLOB srcs "*.cpp" "*.hpp")

add_library(worldNode SHARED ${srcs})
target_link_libraries(worldNode ${DART_LIBRARIES}
        DracoPnC
        myUtils)

add_executable(run_draco ${srcs})
target_link_libraries(run_draco ${DART_LIBRARIES}
                                DracoPnC
                                myUtils)
