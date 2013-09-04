/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_FILTER_H__
#define PARFIL_FILTER_H__

#include <vector>
#include "particle.h"

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
    // Motion update for each of the particles.
    void Predict(const Motion& motion);

    // Measurement update for each of the particles.
    void MeasurementUpdate(const Measurement& measurement);

    // Weights resampling.
    void Resample();

  private:
    // The particle set.
    std::vector<Particle> m_particles;

    // The weights of the particles.
    std::vector<double> m_weights;

}; // class Filter

} // namespace parfil

#endif // PARFIL_FILTER_H__
