project(parfil)

set(SOURCES filter.cc robot.cc test.cc main.cc)
add_graphlab_executable(parfil ${SOURCES})
set(CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS}")
