#pragma once

#include <fmt/core.h>
#include <fmt/format.h>

#include <common/sequence_containers.hpp>
#include <memory>
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <wbc/robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <wbc/scenes/acceleration_reduced_tsid/AccelerationSceneReducedTSID.hpp>
#include <wbc/scenes/acceleration_tsid/AccelerationSceneTSID.hpp>
#include <wbc/tasks/CoMAccelerationTask.hpp>
#include <wbc/tasks/ContactForceTask.hpp>
#include <wbc/tasks/SpatialAccelerationTask.hpp>
#include <wbc/tools/JointIntegrator.hpp>
#include <wbc/types/JointCommand.hpp>
#include <wbc/types/JointState.hpp>
#include <wbc/types/RigidBodyState.hpp>
#include <wbc/types/Wrench.hpp>

#include "mit_controller_params.hpp"
#include "wbc_interface.hpp"

#define FEET_FORCE_TASK
#define COM_TASK
#define FEET_POS_TASK

class WBCArcOPT : public WBCInterface {
 private:
  std::unique_ptr<StateInterface> quad_state_;  // TODO: maybe remove!
  std::array<std::string, ModelInterface::N_LEGS> feet_names_;
  std::array<std::array<unsigned int, ModelInterface::N_JOINTS_PER_LEG>, ModelInterface::N_LEGS> joint_idxs_;

  // WBC
  wbc::RobotModelPtr wbc_robot_model_;
  wbc::QPSolverPtr wbc_solver_;
  std::unique_ptr<wbc::Scene> wbc_scene_;
  wbc::types::JointCommand joint_cmd_;

  // WBC Tasks
  wbc::SpatialAccelerationTaskPtr wbc_com_task_;
  std::array<wbc::ContactForceTaskPtr, ModelInterface::N_LEGS> wbc_foot_contact_tasks_;
  std::array<wbc::SpatialAccelerationTaskPtr, ModelInterface::N_LEGS> wbc_foot_pose_tasks_;

  // WBC states
  std::vector<wbc::types::Contact> active_foot_contacts_;
  wbc::types::RigidBodyState floating_base_state_;
  wbc::types::JointState joint_state_;

  // WBC targets
  wbc::types::RigidBodyState com_target_;
  std::array<wbc::types::RigidBodyState, ModelInterface::N_LEGS> feet_targets_;

  // WBC PD controllers
  wbc::CartesianPosPDController com_cart_pd_controller_;
  std::array<wbc::CartesianPosPDController, ModelInterface::N_LEGS> feet_cart_pd_controllers_;
  // wbc::JointIntegrator joint_integrator_;

  double last_wbc_model_update_time_;

 public:
  WBCArcOPT(std::unique_ptr<StateInterface> state,
            std::string solver,
            std::string scene,
            const std::string& model_urdf,
            const std::array<std::string, ModelInterface::N_LEGS>& feet_names,
            const std::array<std::string, ModelInterface::NUM_JOINTS>& joint_names,
            double mu,
            const Eigen::Matrix<double, 6, 1>& com_pose_weight,
            const Eigen::Matrix<double, 3, 1>& foot_pose_weight,
            const Eigen::Matrix<double, 3, 1>& foot_force_weight,
            const Eigen::Matrix<double, 6, 1>& com_pose_p_gain,
            const Eigen::Matrix<double, 6, 1>& com_pose_d_gain,
            const Eigen::Matrix<double, 3, 1>& feet_pose_p_gain,
            const Eigen::Matrix<double, 3, 1>& feet_pose_d_gain,
            const Eigen::Matrix<double, 6, 1>& com_pose_saturation,
            const Eigen::Matrix<double, 3, 1>& feet_pose_saturation,
            double solver_tolerances);

  void UpdateState(const StateInterface& quad_state) override;
  void UpdateModel(const ModelInterface& quad_model) override;
  void UpdateFeetTarget(const FeetTargets& feet_targets) override;
  void UpdateWrenches(const Wrenches& wrenches) override;
  void UpdateFootContact(const FootContact& foot_contact) override;
  void UpdateTarget(const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& position,
                    const Eigen::Vector3d& lin_vel,
                    const Eigen::Vector3d& ang_vel) override;
  // The command family this controller produces. A class constant as well as the virtual, so the
  // stock plugin adapter can report it on a default-constructed plugin (before its Init has built a
  // WBCArcOPT) without the two answers being able to drift apart.
  static constexpr WBCCommandMode kCommandMode = WBCCommandMode::kJoint;
  WBCCommandMode SupportedCommandMode() const override { return kCommandMode; }
  WBCReturn GetJointCommand(JointTorqueVelocityPositionCommands& joint_command) override;
  // Not this controller's command family — the host validates the mode at bring-up and never
  // calls this (wbc_interface.hpp).
  WBCReturn GetCartesianCommand(CartesianCommands& cartesian_command) override {
    (void)cartesian_command;
    return {false, 0.0, 0.0};
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override;
};
