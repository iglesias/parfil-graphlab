/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

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

  int num_iterations = 0;
  if (argc > 1) num_iterations = atoi(argv[1]);
  if (num_iterations == 0) num_iterations = 1000;

  parfil::test::Case2(motions, measurements, robot_pose, num_iterations);

  graphlab::command_line_options clopts("Particle filter.");

  if(!clopts.parse(argc, argv)) return EXIT_FAILURE;

  graphlab::mpi_tools::init(argc,argv);
  graphlab::distributed_control dc;
  parfil::graph_type graph(dc,clopts);

  int num_particles = 1000;
  parfil::DistributedFilter filter(num_particles,graph);
  graphlab::timer timer;
  timer.start();
  filter.Run(motions,measurements);
  dc.cout() << "Wall time passed: " << timer.current_time() << " (s).\n";

  double x,y,heading;
  filter.GetPose(x,y,heading);
  dc.cout() << "Ground truth: x: " << robot_pose.x() << " y: " << robot_pose.y() << " heading: " << robot_pose.heading() << std::endl;
  dc.cout() << "Particle filter: x: " << x << " y: " << y << " heading: " << heading << std::endl;
  dc.cout() << "Delta: x: " << fabs(x-robot_pose.x()) << " y: " << fabs(y-robot_pose.y()) << " heading: " << fmod(fabs(heading-robot_pose.heading())+M_PI, 2*M_PI)-M_PI << std::endl;

  graphlab::mpi_tools::finalize();

  return 0;
}
