file(GLOB srcs "*.cpp" "*.hpp")

add_executable(pendulum ${srcs})
target_link_libraries(pendulum ${DART_LIBRARIES}
                               PendulumPnC)
