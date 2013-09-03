project(parfil)

cmake_minimum_required(VERSION 2.8)

# Set paths.
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/bin CACHE PATH "Install path prefix" FORCE)


set(SOURCES src/parfil/filter.cc src/parfil/robot.cc src/parfil/test.cc src/parfil/main.cc)

include_directories(${CMAKE_SOURCE_DIR}/src)
add_executable(parfil ${SOURCES})
set(CMAKE_CXX_FLAGS "-Wall -Wextra -g --std=c++0x ${CMAKE_CXX_FLAGS}")
install(TARGETS parfil RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
