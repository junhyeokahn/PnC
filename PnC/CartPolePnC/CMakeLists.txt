file(GLOB_RECURSE srcs "*.cpp" "*.hpp" REMOVE OfflineTrajGen/src.cpp)

add_library(CartPolePnC SHARED ${srcs})
target_link_libraries(CartPolePnC drake::drake
                                  ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils)

add_executable(gen_cartpole_offline_traj OfflineTrajGen/src.cpp)
target_link_libraries(gen_cartpole_offline_traj CartPolePnC)
