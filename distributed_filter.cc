/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#include "distributed_filter.h"

namespace parfil {

const double STEERING_NOISE = 0.1;
const double DISTANCE_NOISE = 5.0;
const double BEARING_NOISE  = 0.1;

graphlab::atomic<graphlab::vertex_id_type> NEXT_VID;

// Pointer to the motion currently processed.
const Motion* MOTION = NULL;
// Pointer to the measurement currently processed.
const Measurement* MEASUREMENT = NULL;

Particle::Particle() {
  pose = Robot();
  weight = 0.0;
}

void Particle::save(graphlab::oarchive& oarc) const {
  oarc << pose << weight;
}

void Particle::load(graphlab::iarchive& iarc) {
  iarc >> pose >> weight;
}

// Constructor from the number of particles.
DistributedFilter::DistributedFilter(graphlab::distributed_control dc, int num_particles) {
  assert(num_particles>0);

  // Create the distributed graph.
  m_graph = new DistributedGraph(dc);
  NEXT_VID = (((graphlab::vertex_id_type)1 << 31) / dc.numprocs()) * dc.procid();

  //FIXME can this be distributed?
  for (int i=0; i<num_particles; ++i) {
    Particle particle;
    particle.pose.SetNoise(STEERING_NOISE,DISTANCE_NOISE,BEARING_NOISE);
    m_graph->add_vertex(NEXT_VID.inc_ret_last(1), particle);
  }

  // Commit the graph structure, marking that it is no longer to be modified.
  m_graph->finalize();
}

// Destructor.
DistributedFilter::~DistributedFilter() {
  delete m_graph;
}

// Extract position from the particle set using reducing the vertices of the
// graph.
void DistributedFilter::GetPose(double& x, double& y, double& heading) const {
  //TODO
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
  //TODO
}

void DistributedFilter::Predict(const Motion& motion) {
  MOTION = &motion;
  m_graph->transform_vertices(ParticlePredict);
}

void DistributedFilter::MeasurementUpdate(const Measurement& measurement) {
  MEASUREMENT = &measurement;
  m_graph->transform_vertices(ParticleMeasurementUpdate);
}

void ParticlePredict(DistributedGraph::vertex_type& v) {
  v.data().pose.Move(*MOTION);
}

void ParticleMeasurementUpdate(DistributedGraph::vertex_type& v) {
  v.data().weight = v.data().pose.ComputeMeasurementProbability(*MEASUREMENT);
}

} // namespace parfil
