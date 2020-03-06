file(GLOB srcs "*.cpp" "*.hpp")

add_library(worldNode SHARED ${srcs})
target_link_libraries(worldNode ${DART_LIBRARIES}
        ScorpioPnC
        DracoPnC
        myUtils)

add_executable(run_scorpio ${srcs})
target_link_libraries(run_scorpio ${DART_LIBRARIES}
                                  dracoWrapper
                                  ScorpioPnC
                                  DracoPnC
                                  myUtils)

add_library(adeWrapper SHARED ${srcs})
target_link_libraries(adeWrapper ${DART_LIBRARIES}
        dracoWrapper
        ScorpioPnC
        DracoPnC
        myUtils)