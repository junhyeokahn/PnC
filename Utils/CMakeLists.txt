FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library (myUtils SHARED ${headers} ${sources})
target_link_libraries(myUtils myYaml
                              drake::drake)

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(myUtils PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(myUtils "${CMAKE_THREAD_LIBS_INIT}")
endif()
