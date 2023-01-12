find_path(NLOPT_INCLUDE_DIR NAMES
  nlopt.f
  nlopt.h
  nlopt.hpp
  HINTS /usr/local/include # UNCOMMENT THIS PART FOR DRAGONFLY8
  # HINTS /usr/include  #UNCOMMENT THIS FOR A PC
)


#set(LIB_DIR /usr/lib/x86_64-linux-gnu/) # x86 PC HARDWARE
set(LIB_DIR /usr/local/lib/arm-linux-gnueabihf/) # Dragonfly ARM HARDWARE
find_library(NLOPT_BASE NAMES nlopt HINTS ${LIB_DIR})

set(NLOPT_LIBRARY ${NLOPT_BASE} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OOQP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(nlopt  DEFAULT_MSG
                                 NLOPT_LIBRARY NLOPT_INCLUDE_DIR)

mark_as_advanced(NLOPT_LIBRARY NLOPT_INCLUDE_DIR)
set(NLOPT_LIBRARIES ${NLOPT_LIBRARY} )
set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR} )
MESSAGE( STATUS "NLOPT_INCLUDE_DIRS: " ${NLOPT_INCLUDE_DIRS} )
MESSAGE( STATUS "NLOOPT_LIBRARIES: " ${NLOPT_LIBRARIES} )

