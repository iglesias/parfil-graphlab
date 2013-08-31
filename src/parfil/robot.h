/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_ROBOT_H__
#define PARFIL_ROBOT_H__

#include <vector>
#include <random>

namespace parfil {

typedef std::vector<double> Measurement;

// Class which contains the information to move the robot.
struct Motion {
  public:
    Motion();
    Motion(double angle, double distance);

  public:
    double steering_angle;
    double forward_distance;
}; // struct Motion

// Class which contains the state of the robot and can be used to represent the
// particles of the filter.
class Robot {
  public:
    // Create a robot, initializing randomly its initial position and orientation.
    Robot();

    // Destroy the robot.
    ~Robot();

    // Get x.
    double x() const { return m_x; }

    // Get y.
    double y() const { return m_y; }

    // Get heading.
    double heading() const { return m_h; }

    // Set the robot's position and heading.
    void Set(double x, double y, double heading);

    // Set the robot's noise parameters.
    void SetNoise(double steering_noise, double distance_noise, double bearing_noise);

    // Apply motion to the robot, adding noise.
    void Move(const Motion& motion);

    // Measure bearing to the landmarks using the robot's pose.
    void Sense(Measurement& measurement, bool use_noise=true);

    // Compute the probability of a measurement given the robot's pose.
    double ComputeMeasurementProbability(const Measurement& measurement);

    // Print the robot's pose.
    void Print() const;

    // Simulate robot movement, filling in measurements as it moves.
    void GenerateGroundTruth(std::vector<Measurement>& measurements, const std::vector<Motion>& motions);

  private:
    // Position in x measured in metres.
    double m_x;

    // Position in y measure in metres.
    double m_y;

    // Heading or orientation in radians.
    double m_h;

    // Steering noise used during movement.
	std::normal_distribution<> m_steering_noise;

    // Distance noise used during movement.
    std::normal_distribution<> m_distance_noise;

    // Bearing noise used during sensing.
    std::normal_distribution<> m_bearing_noise;

}; // class Robot

} // namespace parfil

#endif // PARFIL_ROBOT_H__
