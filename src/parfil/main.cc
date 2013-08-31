#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>
#include <parfil/robot.h>
#include <parfil/filter.h>
#include <ctime>
#include <parfil/test.h>
#include <iostream>

int main(int, char**) {
  std::srand(std::time(0));
  int num_particles = 5000;
  std::unique_ptr<parfil::Filter> filter(new parfil::Filter(num_particles));

  std::vector<parfil::Motion> motions;
  std::vector<parfil::Measurement> measurements;
  parfil::Robot robot;
  parfil::test::Case2(motions,measurements,robot);

  filter->Run(motions,measurements);

  double x,y,heading;
  filter->GetPose(x,y,heading);
  std::cout << "Ground truth: ";
  robot.Print();
  std::cout << "Particle filter: x: " << x << " y: " << y << " heading: " << heading << std::endl;
  std::cout << "Delta: x: " << fabs(x-robot.x()) << " y: " << fabs(y-robot.y()) << " heading: " <<
                fmod(fabs(heading-robot.heading())+M_PI, 2*M_PI)-M_PI << std::endl;

  return 0;
}
