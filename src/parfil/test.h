/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_TEST_H__
#define PARFIL_TEST_H__

#include <parfil/robot.h>
#include <vector>

namespace parfil {

namespace test {

void Case1(std::vector<parfil::Motion>& motions, std::vector<parfil::Measurement>& measurements);

void Case2(std::vector<parfil::Motion>& motions, std::vector<parfil::Measurement>& measurements, parfil::Robot& robot, int num_iterations=100);

} // namespace test

} // namespace parfil

#endif // PARFIL_TEST_H__
