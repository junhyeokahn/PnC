FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(myWBC SHARED ${sources} ${headers})
target_link_libraries(myWBC myUtils
                            myGoldfarb
                            myContact
                            myRobotSystem)
