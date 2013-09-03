project(parfil)
set(SOURCES distributed_filter.cc filter.cc robot.cc test.cc main.cc)
add_graphlab_executable(parfil ${SOURCES})
