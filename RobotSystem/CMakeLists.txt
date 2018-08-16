file(GLOB_RECURSE srcs "*.cpp" "*.hpp")
add_library(myRobotSystem SHARED ${srcs})

include_directories("${PROJECT_SOURCE_DIR}/RobotSystem/RobotModel/Robot/Draco/DracoFixedGen/dyn_eigen/include")
include_directories("${PROJECT_SOURCE_DIR}/RobotSystem/RobotModel/Robot/Draco/DracoFixedGen/kin_eigen/include")

target_link_libraries(myRobotSystem ${DART_LIBRARIES})
