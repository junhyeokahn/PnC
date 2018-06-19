file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(CartPolePnC SHARED ${srcs})
target_link_libraries(CartPolePnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  drake::drake)
