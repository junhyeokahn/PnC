PROTOBUF_GENERATE_CPP(pnc-msg-draco-src pnc-msg-draco-header draco.proto)
add_library(pnc-msg-draco ${pnc-msg-draco-src} ${pnc-msg-draco-header})
target_include_directories(pnc-msg-draco PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(pnc-msg-draco-py draco.proto)
add_custom_target(pnc-msg-draco-py-target ALL DEPENDS ${pnc-msg-draco-py})

PROTOBUF_GENERATE_CPP(pnc-msg-fixed-draco-src pnc-msg-fixed-draco-header fixed_draco.proto)
add_library(pnc-msg-fixed-draco ${pnc-msg-fixed-draco-src} ${pnc-msg-fixed-draco-header})
target_include_directories(pnc-msg-fixed-draco PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(pnc-msg-fixed-draco-py fixed_draco.proto)
add_custom_target(pnc-msg-fixed-draco-py-target ALL DEPENDS ${pnc-msg-fixed-draco-py})

PROTOBUF_GENERATE_PYTHON(pnc-msg-draco-bullet-sensors-py draco_pybullet_sensors.proto)
add_custom_target(pnc-msg-draco-bullet-sensors-py-target ALL DEPENDS ${pnc-msg-draco-bullet-sensors-py})

install(TARGETS pnc-msg-draco DESTINATION "${INSTALL_LIB_DIR}")
install(TARGETS pnc-msg-fixed-draco DESTINATION "${INSTALL_LIB_DIR}")

install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages/")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/fixed_draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages/")
