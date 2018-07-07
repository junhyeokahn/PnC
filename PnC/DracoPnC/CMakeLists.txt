file(GLOB_RECURSE srcs "*.cpp" "*.hpp" REMOVE OfflineTrajGen/src.cpp)

add_library(DracoPnC SHARED ${srcs})
target_link_libraries(DracoPnC drake::drake
                               ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myCentPlanner)

add_executable(gen_draco_offline_traj OfflineTrajGen/src.cpp)
target_link_libraries(gen_draco_offline_traj DracoPnC)
