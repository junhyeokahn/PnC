file(GLOB srcs "*.cpp" "*.hpp")

ADD_LIBRARY(dracoWrapper SHARED ${srcs})
target_link_libraries(dracoWrapper ${DART_LIBRARIES}
        DracoPnC
        worldNode
        myUtils)

add_executable(run_draco_wrapper Main.cpp)
target_link_libraries(run_draco_wrapper ${DART_LIBRARIES}
        DracoPnC
        myUtils
        worldNode
        dracoWrapper)