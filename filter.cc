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

const double STEERING_NOISE = 0.1;
const double DISTANCE_NOISE = 5.0;
const double BEARING_NOISE  = 0.1;

// Constructor from the number of particles.
Filter::Filter(int num_particles) {
  assert(num_particles>0);

  // Particle pose allocation and initialization.

  m_particles.reserve(num_particles);

  for (int i=0; i<num_particles; ++i) {
    Robot particle = Robot();
    particle.SetNoise(STEERING_NOISE,DISTANCE_NOISE,BEARING_NOISE);
    m_particles.push_back(particle);
  }

  // Particle weights allocation.
  m_weights.resize(num_particles);
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
    Filter::Predict(motions[t]);
    Filter::MeasurementUpdate(measurements[t]);
    Filter::Resample();
  }
}

void Filter::Predict(const Motion& motion) {
  for (unsigned int i=0; i<m_particles.size(); ++i)
    m_particles[i].Move(motion);
}

void Filter::MeasurementUpdate(const Measurement& measurement) {
  for (unsigned int i=0; i<m_particles.size(); ++i)
    m_weights[i] = m_particles[i].ComputeMeasurementProbability(measurement);
}

// Circular resampling.
void Filter::Resample() {
  int index = int(1.0*std::rand()/RAND_MAX*m_particles.size());
  double beta = 0.0;
  double max_weight = *std::max_element(m_weights.begin(),m_weights.end());
  std::vector<Robot> new_particles;
  new_particles.reserve(m_particles.size());

  for (unsigned int i=0; i<m_weights.size(); ++i) {
    beta += 1.0*std::rand()/RAND_MAX*2*max_weight;

    while (beta > m_weights[index]) {
      beta -= m_weights[index];
      index = (index+1)%m_particles.size();
    }

    new_particles.push_back(m_particles[index]);
  }

  m_particles = new_particles;
}

} // namespace parfil
