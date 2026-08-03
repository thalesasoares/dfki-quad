#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

#include "mit_controller/pipeline_constants.hpp"

// Predicted body trajectory produced alongside the wrench sequence by the force
// optimisation stage. The whole-body controller tracks knot index 1 (one step
// ahead). Part of the exported shared pipeline type surface — see
// doc/modularity/pipeline_types.md.
struct MPCPrediction {
  std::array<Eigen::Quaterniond, MPC_PREDICTION_HORIZON + 1> orientation;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> position;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> angular_velocity;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> linear_velocity;
  std::array<Eigen::Matrix<double, 13, 1, Eigen::ColMajor>, MPC_PREDICTION_HORIZON + 1> raw_data;
};