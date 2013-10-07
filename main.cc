/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#include <vector>
#include <iostream>
#include <ctime>

#include "particle.h"
#include "test.h"
#include "distributed_filter.h"

#include <graphlab.hpp>

int main(int argc, char** argv) {
  graphlab::random::seed();

  std::vector<parfil::Motion> motions;
  std::vector<parfil::Measurement> measurements;
  parfil::Particle robot_pose;
  parfil::test::Case2(motions,measurements,robot_pose,10);

  graphlab::command_line_options clopts("Particle filter.");

  if(!clopts.parse(argc, argv)) return EXIT_FAILURE;

  graphlab::mpi_tools::init(argc,argv);
  graphlab::distributed_control dc;
  parfil::graph_type graph(dc,clopts);

  int num_particles = 500;
  parfil::DistributedFilter filter(num_particles,graph);
  std::time_t start = std::time(NULL);
  filter.Run(motions,measurements);
  std::cout << "Wall time passed: " << std::difftime(std::time(NULL),start)
            << " (s).\n";

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
