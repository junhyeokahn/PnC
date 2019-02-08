FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB rl_headers "*.hpp")

add_library(myRL SHARED ${sources})

install(TARGETS myRL DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${rl_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC/ReinforcementLearning")
