FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB nn_headers "*.hpp")

add_library(myNN SHARED ${sources})

install(TARGETS myNN DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${nn_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC/ReinforcementLearning")
