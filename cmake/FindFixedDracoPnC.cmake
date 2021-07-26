# - Try to find FixedDracoPNC
#  Once done this will define
#  FIXED_DRACO_PNC_FOUND - System has Gurobi
#  FIXED_DRACO_PNC_INCLUDE_DIRS - The Gurobi include directories
#  FIXED_DRACO_PNC_LIBRARIES - The libraries needed to use Gurobi

find_path(FIXED_DRACO_PNC_INCLUDE_DIR
          NAMES configuration.hpp
          PATHS "/usr/local/include/PnC"
          )

find_library(FIXED_DRACO_PNC_LIBRARY
             NAMES pnc-fixed-draco-pnc
             PATHS "/usr/local/lib"
             )

include(FindPackageHandleStandardArgs)

if(FIXED_DRACO_PNC_INCLUDE_DIR)
    set(FIXED_DRACO_PNC_INCLUDE_DIRS "${FIXED_DRACO_PNC_INCLUDE_DIR}" )
    set(FIXED_DRACO_PNC_LIBRARIES "${FIXED_DRACO_PNC_LIBRARY}" )
    set(FIXED_DRACO_PNC_FOUND TRUE)
    message("-- Found FixedDracoPnC: TRUE")
else()
    message("-- Found FixedDracoPnC: FALSE, Build without FixedDracoPnC")
endif()

mark_as_advanced( FIXED_DRACO_PNC_INCLUDE_DIR
                  FIXED_DRACO_PNC_LIBRARY )
