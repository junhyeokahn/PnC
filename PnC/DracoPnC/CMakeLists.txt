file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_testset_headers "TestSet/*.hpp")
file(GLOB draco_ctrlset_headers "CtrlSet/*.hpp")
file(GLOB draco_contactset_headers "ContactSet/*.hpp")

list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/CentroidDynOpt.cpp
                         ${CMAKE_CURRENT_SOURCE_DIR}/OfflineTrajGen/KinematicOpt.cpp)

if( NOT (ZMQ_FOUND AND Protobuf_FOUND) )
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/TestSet/RLWalkingTest.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootLearningCtrl.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootPolicyCtrl.cpp)
endif()

add_library(DracoPnC SHARED ${sources})
target_link_libraries(DracoPnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myFilters
                               myPlanner
                               myRL)

install(TARGETS DracoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC")
install(FILES ${draco_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/TestSet")
install(FILES ${draco_ctrlset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/CtrlSet")
install(FILES ${draco_contactset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/ContactSet")

add_executable(opt_cent_dyn OfflineTrajGen/CentroidDynOpt.cpp)
target_link_libraries(opt_cent_dyn DracoPnC)

add_executable(opt_kin OfflineTrajGen/KinematicOpt.cpp)
target_link_libraries(opt_kin DracoPnC)
