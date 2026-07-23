#pragma once

#include <Eigen/Core>
#include <array>

#include "mit_controller/pipeline_constants.hpp"

// Per-leg swing targets produced by the swing leg controller and consumed by the
// whole-body controller. Part of the exported shared pipeline type surface —
// see doc/modularity/pipeline_types.md.
struct FeetTargets {
  std::array<Eigen::Vector3d, N_LEGS> positions;
  std::array<Eigen::Vector3d, N_LEGS> velocities;
  std::array<Eigen::Vector3d, N_LEGS> accelerations;
};