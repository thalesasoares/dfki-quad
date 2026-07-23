#pragma once
#include "Eigen/Core"
// Only the pipeline size constants (N_LEGS, N_JOINTS_PER_LEG) are needed here.
// Include them directly rather than the host-only mit_controller_params.hpp: this
// header travels with wbc_interface.hpp on the exported surface (issue #6, M2.1),
// and mit_controller_params.hpp carries host-only switches plus USE_WBC, which
// depends on the ROBOT_MODEL compile definition only this package's targets set.
#include "mit_controller/pipeline_constants.hpp"

struct JointTorqueVelocityPositionCommands {
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> torque;
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> position;
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> velocity;
};

struct JointTorqueCommands {
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> torque;
};

struct CartesianCommands {
  std::array<Eigen::Vector3d, N_LEGS> position;
  std::array<Eigen::Vector3d, N_LEGS> velocity;
  std::array<Eigen::Vector3d, N_LEGS> force;
};