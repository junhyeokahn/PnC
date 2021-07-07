PROTOBUF_GENERATE_CPP(pnc-msg-draco-src pnc-msg-draco-header draco.proto)
add_library(pnc-msg-draco ${pnc-msg-draco-src} ${pnc-msg-draco-header})
target_include_directories(pnc-msg-draco PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(pnc-msg-draco-py draco.proto)
add_custom_target(pnc-msg-draco-py-target ALL DEPENDS ${pnc-msg-draco-py})

install(TARGETS pnc-msg-draco DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages/")
