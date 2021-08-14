cmake_minimum_required(VERSION 3.14.0)
project(Corklib VERSION 0.9.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
include(cmake/find_avx.cmake)

check_for_avx()

message( STATUS "AVX Status: ${HAVE_AVX_EXTENSIONS}" )
message( STATUS "AVX2 Status: ${HAVE_AVX2_EXTENSIONS}" )

if( HAVE_AVX_EXTENSIONS )
  SET( AVX_FLAGS "${AVX_FLAGS} -D__HAVE_AVX_EXTENSIONS__" )
endif ()

if( HAVE_AVX2_EXTENSIONS )
  SET( AVX_FLAGS "${AVX_FLAGS} -D__HAVE_AVX2_EXTENSIONS__" )
endif ()

message( STATUS "AVX Flags: ${AVX_FLAGS}" )

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${AVX_FLAGS} -Wno-deprecated-declarations")

add_compile_definitions( TRILIBRARY )

# Determine if Cork is built as a subproject (using add_subdirectory)
# or if it is the master project.
if (NOT DEFINED CORK_IS_MASTER_PROJECT)
  set(CORK_IS_MASTER_PROJECT OFF)
  if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(CORK_IS_MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
    message(STATUS "Cork is master project")
  endif ()
endif ()

include_directories(
  )

add_subdirectory(CorkLib)
add_subdirectory(RegressionTest)
add_subdirectory(UnitTest)
