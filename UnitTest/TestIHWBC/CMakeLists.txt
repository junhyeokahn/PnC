add_executable(test_ihwbc_draco test_ihwbc_draco.cpp)
target_link_libraries(test_ihwbc_draco ${DART_LIBRARIES}
    		                            DracoPnC
	        	                        myUtils
                                        gtest
                                        gtest_main
                                        pthread)