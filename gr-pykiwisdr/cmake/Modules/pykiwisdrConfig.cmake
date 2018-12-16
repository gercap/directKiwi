INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_PYKIWISDR pykiwisdr)

FIND_PATH(
    PYKIWISDR_INCLUDE_DIRS
    NAMES pykiwisdr/api.h
    HINTS $ENV{PYKIWISDR_DIR}/include
        ${PC_PYKIWISDR_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    PYKIWISDR_LIBRARIES
    NAMES gnuradio-pykiwisdr
    HINTS $ENV{PYKIWISDR_DIR}/lib
        ${PC_PYKIWISDR_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PYKIWISDR DEFAULT_MSG PYKIWISDR_LIBRARIES PYKIWISDR_INCLUDE_DIRS)
MARK_AS_ADVANCED(PYKIWISDR_LIBRARIES PYKIWISDR_INCLUDE_DIRS)

