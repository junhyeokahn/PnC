#file(GLOB srcs "*.cpp")

set(srcs
        DracoWrapper.cpp
        DracoSim.cpp
        Main.cpp
        )


include_directories(.)
include_directories(../PnC/DracoPnC)
include_directories(../PnC/ScorpioPnC)
include_directories(../Simulator/Dart/Scorpio)
include_directories(../Utils/General)
include_directories(../Utils/IO)
include_directories(../Utils/Math)
include_directories(${DART_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${JNI_INCLUDE_DIRS})

ADD_LIBRARY(dracoWrapper SHARED ${srcs})
target_link_libraries(dracoWrapper
        DracoPnC
        ScorpioPnC
        worldNode
        myUtils
        dracoWrapper_swig)

add_executable(run_draco_wrapper ${srcs})
target_link_libraries(run_draco_wrapper ${DART_LIBRARIES}
        DracoPnC
        ScorpioPnC
        worldNode
        myUtils
        dracoWrapper)