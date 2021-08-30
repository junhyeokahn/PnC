FILE(GLOB headers "*.hh" "*.h" "*.hpp")
add_library(pnc-dubins SHARED dubins.cpp)

install(TARGETS pnc-dubins DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/dubins")
