#file(GLOB srcs "*.cpp")

set(srcs
        DracoWrapper.cpp
        DracoSim.cpp
        )


include_directories(.)
include_directories(../PnC/DracoPnC)
include_directories(../Simulator/Dart/Draco)
include_directories(../Utils/General)
include_directories(../Utils/IO)
include_directories(../Utils/Math)
include_directories(${DART_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${JNI_INCLUDE_DIRS})

ADD_LIBRARY(interface SHARED ${srcs})
#target_link_libraries(interface ${DART_LIBRARIES}
        #DracoPnC
        #worldNode
        #myUtils)

ADD_LIBRARY(dracoWrapper SHARED ${srcs})
#target_link_directories(dracoWrapper BEFORE PRIVATE /home/will/workspace/PnC/build/lib)
target_link_libraries(dracoWrapper
        DracoPnC
        worldNode
        myUtils
        dracoWrapper_swig)

add_executable(run_draco_wrapper Main.cpp)
target_link_libraries(run_draco_wrapper ${DART_LIBRARIES}
        DracoPnC
        myUtils
        worldNode
        dracoWrapper)