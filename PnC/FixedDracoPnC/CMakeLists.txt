file(GLOB_RECURSE sources "*.cpp")
file(GLOB fixed_draco_pnc_headers "*.hpp")
file(GLOB fixed_draco_testset_headers "TestSet/*.hpp")

#list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/CentroidDynOpt.cpp
                      #${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/KinematicOpt.cpp)

add_library(FixedDracoPnC SHARED ${sources} ${fixed_draco_pnc_headers}
    ${fixed_draco_testset_headers})

target_link_libraries(FixedDracoPnC drake::drake
                                    ${DART_LIBRARIES}
                                    myRobotSystem
                                    myUtils
                                    myWBC
                                    myPlanner)

install(TARGETS FixedDracoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${fixed_draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/FixedDracoPnC")
install(FILES ${fixed_draco_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/FixedDracoPnC/TestSet")

#add_executable(opt_cent_dyn OfflineTrajGen/CentroidDynOpt.cpp)
#target_link_libraries(opt_cent_dyn FixedDracoPnC)

#add_executable(opt_kin OfflineTrajGen/KinematicOpt.cpp)
#target_link_libraries(opt_kin FixedDracoPnC)
