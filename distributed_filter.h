/* This software is distributed under the MIT license (see LICENSE file in the
 * root directory)
 *
 * Copyright (c) 2013 Fernando J. Iglesias Garcia
 */

#ifndef PARFIL_DISTRIBUTED_FILTER_H__
#define PARFIL_DISTRIBUTED_FILTER_H__

#include <graphlab.hpp>
#include "particle.h"

namespace parfil {

typedef graphlab::distributed_graph<Particle, graphlab::empty> DistributedGraph;

// Class which contains the set of particles and performs the particle filtering
// algorithm in a distributed fashion using GraphLab.
class DistributedFilter {
  public:
    // Create a distributed filter with the specified number of particles
    // randomly initialized.
    DistributedFilter(graphlab::distributed_control dc, int num_particles);

    // Destroy the filter.
    ~DistributedFilter();

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
    // The distributed graph.
    DistributedGraph* m_graph;

}; // class DistributedFilter

// Motion update for one particle.
void ParticlePredict(DistributedGraph::vertex_type& v);

// Measurement update for one particle.
void ParticleMeasurementUpdate(DistributedGraph::vertex_type& v);

} // namespace parfil

#endif // PARFIL_DISTRIBUTED_FILTER_H__
