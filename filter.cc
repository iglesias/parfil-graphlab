/* This software is distributed under the MIT license (see LICENSE file in the
* root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#include "filter.h"
#include <algorithm>
#include <cstdlib>
#include <cassert>

namespace parfil {

extern double WORLD_SIZE;

// Constructor from the number of particles.
Filter::Filter(int num_particles) {
  assert(num_particles>0);

  // Particles allocation and initialization.

  m_particles.reserve(num_particles);

  for (int i=0; i<num_particles; ++i)
    m_particles.push_back(Particle());
}

// Destructor.
Filter::~Filter() {
}

// Extract position from particle set.
void Filter::GetPose(double& x, double& y, double& heading) const {
  x = 0;
  y = 0;
  heading = 0.0;

  for (unsigned int i=0; i<m_particles.size(); ++i) {
    x += m_particles[i].x();
    y += m_particles[i].y();
    // Normalize around the first particle.
    heading += fmod(m_particles[i].heading()-m_particles[0].heading()+M_PI, 2.0*M_PI) +
                m_particles[0].heading() - M_PI;
  }

  // Normalize using the number of particles.
  x /= m_particles.size();
  y /= m_particles.size();
  heading /= m_particles.size();
}

// Run particle filter.
void Filter::Run(const std::vector<Motion>& motions, const std::vector<Measurement>& measurements) {
  assert(motions.size()==measurements.size());

  for (unsigned int t = 0; t<motions.size(); ++t) {
    Predict(motions[t]);
    MeasurementUpdate(measurements[t]);
    Resample();
  }
}

void Filter::Predict(const Motion& motion) {
  for (unsigned int i=0; i<m_particles.size(); ++i)
    m_particles[i].Move(motion);
}

void Filter::MeasurementUpdate(const Measurement& measurement) {
  for (unsigned int i=0; i<m_particles.size(); ++i)
    m_particles[i].UpdateWeight(measurement);
}

// Circular resampling.
void Filter::Resample() {
  int index = int(1.0*std::rand()/RAND_MAX*m_particles.size());
  double beta = 0.0;
  double max_weight = std::max_element(m_particles.begin(),m_particles.end(),Particle::WeightComparator)->weight();
  std::vector<Particle> new_particles;
  new_particles.reserve(m_particles.size());

  for (unsigned int i=0; i<m_particles.size(); ++i) {
    beta += 1.0*std::rand()/RAND_MAX*2*max_weight;

    while (beta > m_particles[index].weight()) {
      beta -= m_particles[index].weight();
      index = (index+1)%m_particles.size();
    }

    new_particles.push_back(m_particles[index]);
  }

  m_particles = new_particles;
}

} // namespace parfil
