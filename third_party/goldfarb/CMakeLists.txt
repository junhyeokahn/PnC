FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (pnc-goldfarb SHARED ${headers} ${sources})

install(TARGETS pnc-goldfarb DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/goldfarb")
