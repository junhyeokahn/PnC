file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(CartPolePnC SHARED ${sources})
target_link_libraries(CartPolePnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  RLProtobuf
                                  myRL
                                  ${Protobuf_LIBRARIES}
                                  ${ZMQ_LIBRARIES})
