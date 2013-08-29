project(parfil)

cmake_minimum_required(VERSION 2.8)

# Set paths.
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/bin CACHE PATH "Install path prefix" FORCE)

# Boost trial.
# set(CMAKE_INCLUDE_PATH "/home/iglesias/workspace/boost_1_54_0/ ${CMAKE_INCLUDE_PATH}")
# find_package(Boost COMPONENTS random)
# message(${CMAKE_INCLUDE_PATH})
# include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")

set (SOURCES src/parfil/filter.cc src/parfil/robot.cc src/parfil/main.cc)

include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories("/home/iglesias/workspace/boost_1_54_0")
add_executable(parfil ${SOURCES})
set(CMAKE_CXX_FLAGS "-Wall -Wextra -g --std=c++0x ${CMAKE_CXX_FLAGS}")
install(TARGETS parfil RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
