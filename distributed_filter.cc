/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#include "distributed_filter.h"
#include <cmath>

namespace parfil {

// Set of particles required to perform resampling.
std::vector<Particle> PARTICLES;
// Particle weights.
std::vector<double> WEIGHTS;
// Maximum particle weight.
double MAX_WEIGHT;
// Pointer to the motion currently processed.
const Motion* MOTION = NULL;
// Pointer to the measurement currently processed.
const Measurement* MEASUREMENT = NULL;

// Constructor from the number of particles.
DistributedFilter::DistributedFilter(int num_particles, graph_type& graph) {
  assert(num_particles>0);
  // Allocate memory for the particle set.
  PARTICLES.resize(num_particles);
  WEIGHTS.resize(num_particles);

  for (int id=0; id<num_particles; ++id)
    graph.add_vertex(id, Particle());

  // Commit the graph structure, marking that it is no longer to be modified.
  graph.finalize();

  m_graph = &graph;
}

// Destructor.
DistributedFilter::~DistributedFilter() {
}

// Extract position from the particle set reducing the vertices of the graph.
void DistributedFilter::GetPose(double& x, double& y, double& heading) const {
  // Sum all the poses (taking care of adding up the heading appropriately).
  pose_reducer r = m_graph->map_reduce_vertices<pose_reducer>(pose_reducer::get_pose);

  // Maximum likelihood estimation of the pose.
  x = r.x/PARTICLES.size();
  y = r.y/PARTICLES.size();
  heading = std::atan2(r.heading_y,r.heading_x);
}

// Run Particle filter.
void DistributedFilter::Run(const std::vector<Motion>& motions, const std::vector<Measurement>& measurements) {
  assert(motions.size()==measurements.size());

  for (unsigned int t = 0; t<motions.size(); ++t) {
    Predict(motions[t]);
    MeasurementUpdate(measurements[t]);
    Resample();
  }
}

// Weights resampling.
void DistributedFilter::Resample() {
  // Find the largest particle weight.
  MAX_WEIGHT = m_graph->map_reduce_vertices<max_weight_reducer>(max_weight_reducer::get_max_weight).max_weight;
  // Resample particles.
  m_graph->transform_vertices(ParticleResample);
}

void DistributedFilter::Predict(const Motion& motion) {
  MOTION = &motion;
  m_graph->transform_vertices(ParticlePredict);
}

void DistributedFilter::MeasurementUpdate(const Measurement& measurement) {
  MEASUREMENT = &measurement;
  m_graph->transform_vertices(ParticleMeasurementUpdate);
}

void ParticlePredict(graph_type::vertex_type& v) {
  v.data().Move(*MOTION);
  PARTICLES[v.id()] = v.data();
}

void ParticleMeasurementUpdate(graph_type::vertex_type& v) {
  WEIGHTS[v.id()] = v.data().ComputeMeasurementProbability(*MEASUREMENT);
}

void ParticleResample(graph_type::vertex_type& v) {
  size_t index = graphlab::random::uniform(size_t(0),PARTICLES.size()-1);
  double beta = graphlab::random::uniform(0.0,2*MAX_WEIGHT);

  while (beta > WEIGHTS[index]) {
    beta -= WEIGHTS[index];
    index = (index+1)%PARTICLES.size();
  }

  v.data() = PARTICLES[index];
}

max_weight_reducer::max_weight_reducer() : max_weight(0.0) { }

max_weight_reducer::max_weight_reducer(double weight) : max_weight(weight) { }

max_weight_reducer max_weight_reducer::get_max_weight(const graph_type::vertex_type& v) {
  return max_weight_reducer(WEIGHTS[v.id()]);
}

max_weight_reducer& max_weight_reducer::operator+=(const max_weight_reducer& other) {
  max_weight = std::max(max_weight, other.max_weight);
  return *this;
}

pose_reducer pose_reducer::get_pose(const graph_type::vertex_type& v) {
  pose_reducer r;

  r.x = v.data().x();
  r.y = v.data().y();

  r.heading_x = std::cos(v.data().heading());
  r.heading_y = std::sin(v.data().heading());

  return r;
}

pose_reducer& pose_reducer::operator+=(const pose_reducer& other) {
  x += other.x;
  y += other.y;

  heading_x += other.heading_x;
  heading_y += other.heading_y;

  return *this;
}

} // namespace parfil
