find_package(PkgConfig)

include(FetchContent)       # <--- note: requires CMake version > 3.11
#
# sciplot
#
#if (B_USE_SCIPLOT)
#    FetchContent_Declare(
#            sciplot_content
#            GIT_REPOSITORY https://github.com/sciplot/sciplot.git
#            GIT_TAG master)
#
#    FetchContent_GetProperties(sciplot_content)
#    if(NOT sciplot_content_POPULATED)
#        FetchContent_Populate(sciplot_content)
#    endif()
#
#    include_directories(${sciplot_content_SOURCE_DIR})
#endif ()

# tests that DO NOT DEPEND on GTest
add_executable(qp_test qp_test.cpp)
target_link_libraries(qp_test pnc-goldfarb pnc-utils)

add_executable(draco_interface_test draco_interface_test.cpp)
target_link_libraries(draco_interface_test pnc-draco-pnc)

#
# Gtests
#
if(B_BUILD_GTESTS)
    if(B_FETCH_GTEST_PKG)
        FetchContent_Declare(
                googletest
                GIT_REPOSITORY https://github.com/google/googletest.git
                GIT_TAG        e2239ee6043f73722e7aa812a459f54a28552929 # release-1.11.0
                FIND_PACKAGE_ARGS NAMES GTest
        )
        FetchContent_MakeAvailable(googletest)
    endif()
    pkg_check_modules(GTEST "gtest" REQUIRED)

    add_executable(hermite_curves_test hermite_curves_test.cpp)
    target_link_libraries(hermite_curves_test ${GTEST_LIBRARIES} pnc-utils)
endif()