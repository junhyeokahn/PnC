#
# Try to find MOSEK
# Once done this will define
#
# MOSEK_FOUND           - system has MOSEK
# MOSEK_INCLUDE_DIRS    - the MOSEK include directories
# MOSEK_LIBRARIES       - Link these to use MOSEK
#

# Hardcoded search paths
find_path(MOSEK_INCLUDE_DIR mosek.h
    PATHS $ENV{MOSEK_HOME}
    PATH_SUFFIXES h
    )

set(MOSEK_LIBRARIES)
set(FUSION_LIBRARIES)

find_library(MOSEK_LIBRARIES NAMES mosek64
    HINT
    "${MOSEK_INCLUDE_DIR}"
    "${MOSEK_INCLUDE_DIR}/../bin"
    "${MOSEK_INCLUDE_DIR}/lib"
    PATHS
    $ENV{MOSEK_HOME}
    NO_DEFAULT_PATH
    PATH_SUFFIXES a bin lib dylib)

find_library(FUSION_LIBRARIES NAMES fusion64
    HINT
    "${MOSEK_INCLUDE_DIR}"
    "${MOSEK_INCLUDE_DIR}/../bin"
    "${MOSEK_INCLUDE_DIR}/lib"
    PATHS
    $ENV{MOSEK_HOME}
    NO_DEFAULT_PATH
    PATH_SUFFIXES a bin lib dylib)

set(MOSEK_FUSION_LIBRARIES "${MOSEK_LIBRARIES};${FUSION_LIBRARIES}")
# Check that Mosek was successfully found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    MOSEK DEFAULT_MSG MOSEK_LIBRARIES FUSION_LIBRARIES MOSEK_INCLUDE_DIR
    )
set(MOSEK_INCLUDE_DIRS ${MOSEK_INCLUDE_DIR})

# Hide variables from CMake-Gui options
mark_as_advanced(MOSEK_LIBRARIES FUSION_LIBRARIES MOSEK_INCLUDE_DIRS MOSEK_INCLUDE_DIR
                 MOSEK_FUSION_LIBRARIES)

