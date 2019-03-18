file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

if( NOT (ZMQ_FOUND AND Protobuf_FOUND) )
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/TestSet/RLWalkingTest.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootLearningCtrl.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootPolicyCtrl.cpp)
endif()

add_library(AtlasPnC ${sources})
target_link_libraries(AtlasPnC ${DART_LIBRARIES}
                                myRobotSystem
                                myUtils
                                myFilters
                                myPlanner
                                myWBC
                                myRL)
