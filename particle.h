/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_PARTICLE_H__
#define PARFIL_PARTICLE_H__

#include <vector>
#include <graphlab.hpp>

namespace parfil {

typedef std::vector<double> Measurement;

// Class which contains the information to move a particle.
struct Motion {
  public:
    Motion();
    Motion(double angle, double distance);

  public:
    double steering_angle;
    double forward_distance;
}; // struct Motion

// Class used to represent the particles of the filter.
class Particle {
  public:
    // Create a particle, initializing randomly its initial position and orientation,
    // and weight equal to zero.
    Particle();

    // Destroy the particle.
    ~Particle();

    // Assign operator.
    Particle& operator=(const Particle& rhs);

    // Write data to output stream.
    void save(graphlab::oarchive& oarc) const;

    // Read data from input stream.
    void load(graphlab::iarchive& iarc);

    // Get x.
    double x() const { return m_x; }

    // Get y.
    double y() const { return m_y; }

    // Get heading.
    double heading() const { return m_h; }

    // Set the particle's position and heading.
    void SetPose(double x, double y, double heading);

    // Apply motion to the particle, adding noise.
    void Move(const Motion& motion);

    // Measure bearing to the landmarks using the particle's pose.
    void Sense(Measurement& measurement, bool use_noise=true);

    // Compute the probability of a measurement given the particle's pose and
    // update the particle's weight.
    double ComputeMeasurementProbability(const Measurement& measurement);

    // Simulate movement, filling in measurements accordingly.
    void GenerateGroundTruth(std::vector<Measurement>& measurements, const std::vector<Motion>& motions);

  private:
    // Position in x measured in metres.
    double m_x;

    // Position in y measure in metres.
    double m_y;

    // Heading or orientation in radians.
    double m_h;

}; // class Particle

} // namespace parfil

#endif // PARFIL_PARTICLE_H__
