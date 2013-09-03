#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>

#include "particle.h"
#include "test.h"
#include "distributed_filter.h"

#include <graphlab.hpp>

int main(int argc, char** argv) {
  std::srand(std::time(0));

  std::vector<parfil::Motion> motions;
  std::vector<parfil::Measurement> measurements;
  parfil::Particle robot_pose;
  parfil::test::Case2(motions,measurements,robot_pose);

  graphlab::mpi_tools::init(argc,argv);
  graphlab::distributed_control dc;

  int num_particles = 5000;
  parfil::DistributedFilter* filter = new parfil::DistributedFilter(dc,num_particles);
/*
  filter->Run(motions,measurements);

  double x,y,heading;
  filter->GetPose(x,y,heading);
  std::cout << "Ground truth: x: " << robot_pose.x() << " y: " robot_pose.y() << " heading: " << robot_pose.Heading() << std::endl;
  std::cout << "Particle filter: x: " << x << " y: " << y << " heading: " << heading << std::endl;
  std::cout << "Delta: x: " << fabs(x-robot.x()) << " y: " << fabs(y-robot.y()) << " heading: " <<
                fmod(fabs(heading-robot.heading())+M_PI, 2*M_PI)-M_PI << std::endl;

  delete filter;
*/
  graphlab::mpi_tools::finalize();

  return 0;
}
