FILE(GLOB sources "*.cpp")
FILE(GLOB rl_headers "*.hpp")

if(NOT (ZMQ_FOUND AND Protobuf_FOUND))
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/RLInterface.cpp)
    list(REMOVE_ITEM rl_headers ${CMAKE_CURRENT_SOURCE_DIR}/RLInterface.hpp)
endif()

add_library(myRL SHARED ${sources})
target_link_libraries(myRL myUtils)

if(ZMQ_FOUND AND Protobuf_FOUND)
    target_link_libraries(myRL RLProtobuf
                               ${Protobuf_LIBRARIES}
                               ${ZMQ_LIBRARIES})
endif()

install(TARGETS myRL DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${rl_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ReinforcementLearning/RLInterface")
