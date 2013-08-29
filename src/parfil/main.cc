#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>
#include <parfil/robot.h>
#include <parfil/filter.h>
#include <ctime>
#include <parfil/test.h>

int main(int, char**) {
  std::srand(std::time(0));
  int num_particles = 500;
  std::unique_ptr<parfil::Filter> filter(new parfil::Filter(num_particles));

  std::vector<parfil::Motion> motions;
  std::vector<parfil::Measurement> measurements;
  parfil::test::Case1(motions,measurements);

  filter->Run(motions,measurements);

  double x,y,heading;
  filter->GetPose(x,y,heading);
  std::cout << "x: " << x << " y: " << y << " heading: " << heading << std::endl;

  return 0;
}
