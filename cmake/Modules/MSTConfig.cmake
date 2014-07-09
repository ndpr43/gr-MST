INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_MST MST)

FIND_PATH(
    MST_INCLUDE_DIRS
    NAMES MST/api.h
    HINTS $ENV{MST_DIR}/include
        ${PC_MST_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    MST_LIBRARIES
    NAMES gnuradio-MST
    HINTS $ENV{MST_DIR}/lib
        ${PC_MST_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MST DEFAULT_MSG MST_LIBRARIES MST_INCLUDE_DIRS)
MARK_AS_ADVANCED(MST_LIBRARIES MST_INCLUDE_DIRS)

