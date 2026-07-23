#pragma once

#include <Eigen/Dense>
#include <array>

#include "mit_controller/pipeline_constants.hpp"

// Ground reaction forces produced by the force optimisation stage and consumed
// by the whole-body controller. Part of the exported shared pipeline type
// surface — see doc/modularity/pipeline_types.md.
struct WrenchSequence {
  std::array<std::array<Eigen::Vector3d, N_LEGS>, MPC_PREDICTION_HORIZON> forces;
};