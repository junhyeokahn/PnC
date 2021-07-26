# - Try to find DracoPNC
#  Once done this will define
#  DRACO_PNC_FOUND - System has Gurobi
#  DRACO_PNC_INCLUDE_DIRS - The Gurobi include directories
#  DRACO_PNC_LIBRARIES - The libraries needed to use Gurobi

find_path(DRACO_PNC_INCLUDE_DIR
          NAMES configuration.hpp
          PATHS "/usr/local/include/PnC"
          )

find_library(DRACO_PNC_LIBRARY
             NAMES pnc-draco-pnc
             PATHS "/usr/local/lib"
             )

#find_library(FIXED_DRACO_PNC_LIBRARY
             #NAMES pnc-fixed-draco-pnc
             #PATHS "/usr/local/lib"
             #)

include(FindPackageHandleStandardArgs)

if(DRACO_PNC_INCLUDE_DIR)
    set(DRACO_PNC_INCLUDE_DIRS "${DRACO_PNC_INCLUDE_DIR}" )
    set(DRACO_PNC_LIBRARIES "${DRACO_PNC_LIBRARY}" )
    set(DRACO_PNC_FOUND TRUE)
    message("-- Found DracoPnC: TRUE")
else()
    message("-- Found DracoPnC: FALSE, Build without DracoPnC")
endif()

mark_as_advanced( DRACO_PNC_INCLUDE_DIR
                  DRACO_PNC_LIBRARY )
