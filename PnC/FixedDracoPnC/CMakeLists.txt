file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

#list(REMOVE_ITEM srcs ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/CentroidDynOpt.cpp
                      #${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/KinematicOpt.cpp)

add_library(FixedDracoPnC SHARED ${srcs})
target_link_libraries(FixedDracoPnC drake::drake
                                    ${DART_LIBRARIES}
                                    myRobotSystem
                                    myUtils
                                    myWBC
                                    myPlanner)


#add_executable(opt_cent_dyn OfflineTrajGen/CentroidDynOpt.cpp)
#target_link_libraries(opt_cent_dyn FixedDracoPnC)

#add_executable(opt_kin OfflineTrajGen/KinematicOpt.cpp)
#target_link_libraries(opt_kin FixedDracoPnC)
