# - Try to find openmht
# Once done, this will define
#
#  openmht_FOUND - system has openmht
#  openmht_INCLUDE_DIRS - the openmht include directories
#  openmht_LIBRARIES - link these to use openmht
set(openmht_INCLUDE_DIR "home/teama/openmht/include/")
set(openmht_PKGCONF_LIBRARY_DIRS "/openmht/lib/")
include_directories(${openmht_INCLUDE_DIRS})

include(LibFindMacros)

# Dependencies (forward, required or quietly)
#libfind_package(openmht_SIM openmht)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(openmht_PKGCONF openmht)

# Include dir
find_path(openmht_INCLUDE_DIR
  NAMES openmht/multi/MHT.h
  PATHS ${openmht_PKGCONF_INCLUDE_DIRS}
)

message("==================================================")
message("openmht_INCLUDE_DIR: ${openmht_INCLUDE_DIR}")
message("openmht_PKGCONF_LIBRARY_DIRS: ${openmht_PKGCONF_LIBRARY_DIRS}")
message("openmht_PKGCONF_INCLUDE_DIRS: ${openmht_PKGCONF_INCLUDE_DIRS}")
message("==================================================")

# Find all the relevant openmht libraries
find_library(common_LIBRARY
  NAMES openmht-common
  PATHS ${openmht_PKGCONF_LIBRARY_DIRS}
)
message("1=================================================")

find_library(filter_LIBRARY
  NAMES openmht-filter
  PATHS ${openmht_PKGCONF_LIBRARY_DIRS}
)
message("2=================================================")

find_library(multi_LIBRARY
  NAMES openmht-multi
  PATHS ${openmht_PKGCONF_LIBRARY_DIRS}
)
message("3=================================================")

find_library(sim_LIBRARY
  NAMES openmht-sim
  PATHS ${openmht_PKGCONF_LIBRARY_DIRS}
)
message("4=================================================")


# target_link_libraries( mht_test 
#   ${openmht_LIBRARIES}
# )  


message("11openmht_PROCESS_INCLUDES: ${openmht_PROCESS_INCLUDES}")
message("11openmht_INCLUDE_DIR: ${openmht_INCLUDE_DIR}")
message("11openmht_PROCESS_LIBS: ${openmht_PROCESS_LIBS}")
message("11common_LIBRARY: ${common_LIBRARY}")
message("11filter_LIBRARY: ${filter_LIBRARY}")
# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(openmht_PROCESS_INCLUDES openmht_INCLUDE_DIR )
set(openmht_PROCESS_LIBS 
  common_LIBRARY
  filter_LIBRARY
  multi_LIBRARY
  sim_LIBRARY
)
message("11openmht_PROCESS_INCLUDES: ${openmht_PROCESS_INCLUDES}")
message("11openmht_INCLUDE_DIR: ${openmht_INCLUDE_DIR}")
message("11openmht_PROCESS_LIBS: ${openmht_PROCESS_LIBS}")
message("11common_LIBRARY: ${common_LIBRARY}")
message("11filter_LIBRARY: ${filter_LIBRARY}")

libfind_process(openmht)
