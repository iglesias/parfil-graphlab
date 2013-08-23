/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_FILTER_H__
#define PARFIL_FILTER_H__

#include <vector>
#include <parfil/robot.h>

namespace parfil {

// Class which contains the set of particles and performs the particle filtering
// algorithm.
class Filter {
  public:
    // Create a filter with the specified number of particles randomly initialized.
    explicit Filter(int num_particles);

    // Destroy the filter.
    ~Filter();

    // Extract position from the particle set.
    void GetPose(double& x, double& y, double& heading) const;

    // Perform particle filtering using the given chain of motions and
    // measurements.
    void Run(const std::vector<Motion>& motions, const std::vector<Measurement>& measurements);

  private:
    // The particle set.
    std::vector<Robot> m_particles;

}; // class Filter

} // namespace parfil
