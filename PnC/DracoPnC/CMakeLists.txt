file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_testset_headers "TestSet/*.hpp")
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/CentroidDynOpt.cpp
                         ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/KinematicOpt.cpp)

add_library(DracoPnC SHARED ${sources})
target_link_libraries(DracoPnC drake::drake
                               ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myPlanner)

install(TARGETS DracoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC")
install(FILES ${draco_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/TestSet")

add_executable(opt_cent_dyn OfflineTrajGen/CentroidDynOpt.cpp)
target_link_libraries(opt_cent_dyn DracoPnC)

add_executable(opt_kin OfflineTrajGen/KinematicOpt.cpp)
target_link_libraries(opt_kin DracoPnC)
