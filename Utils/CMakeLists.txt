FILE(GLOB my_utils_headers "*.hpp" "*.h")
FILE(GLOB my_utils_sources "*.cpp")
FILE(GLOB my_third_party_utils_headers "ThirdPartyUtils/*.hpp" "ThirdPartyUtils/*.h")
FILE(GLOB my_third_party_utils_sources "ThirdPartyUtils/*.cpp")

add_library(myUtils SHARED ${my_utils_headers} ${my_utils_sources})
target_link_libraries(myUtils myYaml)

if(USE_DRAKE)
  add_library(myThirdPartyUtils SHARED ${my_third_party_utils_headers}
                                       ${my_third_party_utils_sources})
  target_link_libraries(myThirdPartyUtils drake::drake)
endif()

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(myUtils PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(myUtils "${CMAKE_THREAD_LIBS_INIT}")
endif()

install(TARGETS myUtils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${my_utils_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils")

if(USE_DRAKE)
  install(TARGETS myThirdPartyUtils DESTINATION "${INSTALL_LIB_DIR}")
  install(FILES ${my_third_party_utils_headers} DESTINATION
      "${INSTALL_INCLUDE_DIR}/Utils/ThirdPartyUtils")
endif()
