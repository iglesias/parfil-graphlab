#include <vector>
#include <iostream>

#include "particle.h"
#include "test.h"
#include "distributed_filter.h"

#include <graphlab.hpp>

int main(int argc, char** argv) {
  graphlab::random::seed();

  std::vector<parfil::Motion> motions;
  std::vector<parfil::Measurement> measurements;
  parfil::Particle robot_pose;
  parfil::test::Case2(motions,measurements,robot_pose);

  graphlab::command_line_options clopts("Particle filter.");

  if(!clopts.parse(argc, argv)) return EXIT_FAILURE;

  graphlab::mpi_tools::init(argc,argv);
  graphlab::distributed_control dc;
  parfil::graph_type graph(dc,clopts);

  int num_particles = 5000;
  parfil::DistributedFilter filter(num_particles,graph,dc);
  filter.Run(motions,measurements);

  double x,y,heading;
  filter.GetPose(x,y,heading);
  std::cout << "Ground truth: x: " << robot_pose.x() << " y: " << robot_pose.y() << " heading: " <<
                robot_pose.heading() << std::endl;
  std::cout << "Particle filter: x: " << x << " y: " << y << " heading: " << heading << std::endl;
  std::cout << "Delta: x: " << fabs(x-robot_pose.x()) << " y: " << fabs(y-robot_pose.y()) << " heading: " <<
                fmod(fabs(heading-robot_pose.heading())+M_PI, 2*M_PI)-M_PI << std::endl;

  graphlab::mpi_tools::finalize();

  return 0;
}
