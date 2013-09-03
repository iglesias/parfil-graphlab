/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#include "robot.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace parfil {

const double STEERING_NOISE = 0.1;
const double DISTANCE_NOISE = 5.0;
const double BEARING_NOISE  = 0.1;

const double WORLD_SIZE = 100.0;

// Number of landmarks in the world.
const int NUM_LANDMARKS = 4;
// Position of four landmarks in (y,x) format.
const double LANDMARKS[NUM_LANDMARKS][2] = {{0, 100}, {0,0}, {100,0}, {100,100}};

// Default motion constructor.
Motion::Motion() {
  steering_angle   = 0.0;
  forward_distance = 0.0;
}

// Motion constructor.
Motion::Motion(double angle, double distance)
{
  steering_angle   = angle;
  forward_distance = distance;
}

// Default constructor.
Robot::Robot() {
  // Set pose.
  m_x = WORLD_SIZE*std::rand()/RAND_MAX;
  m_y = WORLD_SIZE*std::rand()/RAND_MAX;
  m_h = 2.0*M_PI*std::rand()/RAND_MAX;
}

// Destructor.
Robot::~Robot() {
}

Robot& Robot::operator=(const Robot& rhs) {
  m_x = rhs.x();
  m_y = rhs.y();
  m_h = rhs.heading();

  return *this;
}

void Robot::save(graphlab::oarchive& oarc) const {
  oarc << m_x << m_y << m_h;
}

void Robot::load(graphlab::iarchive& iarc) {
  iarc >> m_x >> m_y >> m_h;
}

// Set robot's position and heading.
void Robot::Set(double x, double y, double heading) {
  if (heading < 0 || heading >= 2.0*M_PI) {
    std::cerr << "Robot::Set: The heading must be in [0,2*Pi), given "
              << heading <<  '.' << std::endl;
    std::exit(1);
  }
  
  m_x = x;
  m_y = y;
  m_h = heading;
}

// Move robot.
void Robot::Move(const Motion& motion) {
  if (motion.forward_distance < 0) {
    std::cerr << "Robot::Move: The robot cannot move backwards, forward distance is "
              << motion.forward_distance << '.' << std::endl;
    std::exit(1);
  }

  // Turn, adding randomness to the steering angle.
  m_h += graphlab::random::normal(motion.steering_angle,STEERING_NOISE);
  m_h  = fmod(m_h, 2*M_PI);

  // Move, adding randomness to the motion.
  double distance = graphlab::random::normal(motion.forward_distance,DISTANCE_NOISE);
  m_x += distance*cos(m_h);
  m_y += distance*sin(m_h);
}

// Measure bearing to landmarks.
void Robot::Sense(Measurement& measurement, bool use_noise) {
  measurement.resize(NUM_LANDMARKS);

  for (int i=0; i<NUM_LANDMARKS; ++i) {
    // Measure angle or bearing between the robot pose and each landmark.
    double bearing = atan2(LANDMARKS[i][0]-m_y, LANDMARKS[i][1]-m_x)-m_h;

    if (use_noise)
      bearing += graphlab::random::normal(0,BEARING_NOISE);

    // Normalize the angle to the interval [0,2*pi)
    bearing = fmod(bearing, 2*M_PI);

    measurement[i] = bearing;
  }
}

// Computes the probability of a mesurement of the bearings.
double Robot::ComputeMeasurementProbability(const Measurement& measurement) {
  // Compute the noiseless measurement.
  Measurement true_measurement;
  Sense(true_measurement,false);

  assert(measurement.size()==true_measurement.size());

  // Compute errors in the given measurement.
  double total_error = 1.0;
  for (unsigned int i=0; i<measurement.size(); ++i) {
    double error = fabs(measurement[i]-true_measurement[i]);
    // Normalize the angle error.
    error = fmod(error+M_PI, 2*M_PI) - M_PI;

    // Update the total error using Gaussian noise.
    total_error *= exp(-pow(error,2) / pow(BEARING_NOISE,2) / 2) / sqrt(2.0*M_PI*pow(BEARING_NOISE,2));
  }

  return total_error;
}

// Print robot's pose.
void Robot::Print() const {
  std::cout << "x: " << m_x << " y: " << m_y << " heading: " << m_h << std::endl;
}

// Simulate robot movement and fill in measurements.
void Robot::GenerateGroundTruth(std::vector<Measurement>& measurements, const std::vector<Motion>& motions)
{
  measurements.clear();
  measurements.resize(motions.size());

  for (unsigned int i=0; i<motions.size(); ++i) {
    Move(motions[i]);
    Sense(measurements[i]);
  }
}

} // namespace parfil
