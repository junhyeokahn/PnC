file(GLOB_RECURSE sources "*.cpp")
file(GLOB cartpole_pnc_headers "*.hpp")
file(GLOB cartpole_testset_headers "TestSet/*.hpp")
file(GLOB cartpole_ctrlset_headers "TestSet/*.hpp")

if( NOT ( ZMQ_FOUND AND Protobuf_FOUND) )
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/TestSet/RLTest.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/CtrlSet/NeuralNetCtrl.cpp)
endif()

add_library(CartPolePnC SHARED ${sources})
target_link_libraries(CartPolePnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  RLProtobuf
                                  myNN
                                  ${Protobuf_LIBRARIES}
                                  ${ZMQ_LIBRARIES})
