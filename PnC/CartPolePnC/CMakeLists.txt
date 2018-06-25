file(GLOB_RECURSE srcs "*.cpp" "*.hpp" REMOVE OfflineTrajectoryGeneration/src.cpp)

add_library(CartPolePnC SHARED ${srcs})
target_link_libraries(CartPolePnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  drake::drake)

add_executable(gen_offline_trajectory OfflineTrajectoryGeneration/src.cpp)
target_link_libraries(gen_offline_trajectory CartPolePnC)
