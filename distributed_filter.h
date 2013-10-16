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

typedef graphlab::distributed_graph<Particle, graphlab::empty> graph_type;

// Class which contains the set of particles and performs the particle filtering
// algorithm in a distributed fashion using GraphLab.
class DistributedFilter {
  public:
    // Create a distributed filter with the specified number of particles
    // randomly initialized.
    DistributedFilter(int num_particles, graph_type& graph);

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
    graph_type* m_graph;

}; // class DistributedFilter

// Motion update for one particle.
void ParticlePredict(graph_type::vertex_type& v);

// Measurement update for one particle.
void ParticleMeasurementUpdate(graph_type::vertex_type& v);

// Resample for one particle.
void ParticleResample(graph_type::vertex_type& v);

// MapReduce to compute the maximum particle weight.
struct max_weight_reducer : public graphlab::IS_POD_TYPE {
  double max_weight;

  max_weight_reducer();

  explicit max_weight_reducer(double weight);

  static max_weight_reducer get_max_weight(const graph_type::vertex_type& v);

  max_weight_reducer& operator+=(const max_weight_reducer& other);
}; // struct max_weight_reducer

// MapReduce to compute the pose applying using maximum likelihood over the
// particle set.
struct pose_reducer : public graphlab::IS_POD_TYPE {
  double x;
  double y;
  double heading_x;
  double heading_y;

  static pose_reducer get_pose(const graph_type::vertex_type& v);

  pose_reducer& operator+=(const pose_reducer& other);
}; // struct pose_reducer

} // namespace parfil

#endif // PARFIL_DISTRIBUTED_FILTER_H__
