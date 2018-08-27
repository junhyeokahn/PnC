file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

list(REMOVE_ITEM srcs ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/CentroidDynOpt.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/KinematicOpt.cpp)

add_library(DracoPnC SHARED ${srcs})
target_link_libraries(DracoPnC drake::drake
                               ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myCentPlanner)


add_executable(cent_dyn_opt OfflineTrajGen/CentroidDynOpt.cpp)
target_link_libraries(cent_dyn_opt DracoPnC)

add_executable(kin_opt OfflineTrajGen/KinematicOpt.cpp)
target_link_libraries(kin_opt DracoPnC)
