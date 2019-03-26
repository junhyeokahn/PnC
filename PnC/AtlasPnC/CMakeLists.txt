file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

<<<<<<< HEAD
add_library(AtlasPnC SHARED ${sources})
=======
if( NOT (ZMQ_FOUND AND Protobuf_FOUND) )
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/TestSet/RLWalkingTest.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootLearningCtrl.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/BodyFootPolicyCtrl.cpp)
endif()

add_library(AtlasPnC ${sources})
>>>>>>> cdb8ed94b069484c84da0e22aa7d4eb6ebe213ef
target_link_libraries(AtlasPnC ${DART_LIBRARIES}
                                myRobotSystem
                                myUtils
                                myFilters
                                myPlanner
                                myWBC
                                myRL)
