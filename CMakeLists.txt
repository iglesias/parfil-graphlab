project(parfil)

cmake_minimum_required(VERSION 2.8)

message(STATUS "Compiling parfil-graphlab sources...")
message(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}" )
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}" )
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_CXX_FLAGS "-D HAS_MPI")

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories("/home/iglesias/workspace/boost_1_53_0")
include_directories("/usr/include/mpi")

# find libraries
find_library(GRAPHLAB_LIBRARY NAMES graphlab libgraphlab)
find_library(MPI_LIBRARY NAMES mpi libmpi)
find_library(MPIXX_LIBRARY NAMES mpi++ libmpi++)
find_library(TCMALLOC_LIBRARY NAMES tcmalloc libtcmalloc)
find_library(ZOOKEEPER_MT_LIBRARY NAMES zookeeper_mt libzookeeper_mt)
find_library(ZOOKEEPER_ST_LIBRRARY NAMES zookeeper_st libzookeeper_st)
find_library(EVENT_LIBRARY NAMES event libevent)
find_library(EVENT_PTHREADS_LIBRARY NAMES event_pthreads libevent_pthreads)
#FIXME make machine independent
find_library(BOOST_CONTEXT_LIBRARY NAMES boost_context libboost_context
  HINTS "/home/iglesias/workspace/boost_1_53_0/stage/lib")
find_library(BOOST_PROGRAM_OPTIONS NAMES boost_program_options libboost_program_options
  HINTS "/home/iglesias/workspace/boost_1_53_0/stage/lib")
find_library(BOOST_SYSTEM NAMES boost_system libboost_system
  HINTS "/home/iglesias/workspace/boost_1_53_0/stage/lib")

set(LIBS popt pthread
  ${MPI_LIBRARY} ${MPIXX_LIBRARY} ${TCMALLOC_LIBRARY}
  ${EVENT_LIBRARY} ${EVENT_PTHREADS_LIBRARY}
  ${ZOOKEEPER_MT_LIBRARY} ${ZOOKEEPER_ST_LIBRRARY}
  ${BOOST_CONTEXT_LIBRARY} ${BOOST_PROGRAM_OPTIONS} ${BOOST_SYSTEM}
  ${GRAPHLAB_LIBRARY})

set(SOURCES distributed_filter.cc filter.cc particle.cc test.cc main.cc)
set(TARGET parfil-graphlab)
add_executable(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} ${LIBS})
