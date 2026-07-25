// Stock whole-body-controller plugins (issue #8, M2.3).
//
// Thin adapters (plugin_lifecycle.md §4, Option B) around the two existing WBC
// implementations. Init mirrors the host's create_wbc lambda
// (mit_controller_node.cpp:516-547): the true branch is wbc_arc_opt, the false
// branch is inverse_dynamics.
//
// WBCInterface is still a class template (gap G8, #13), so the two WBCs implement
// different instantiations and therefore register under two different plugin
// bases in the SAME library (stage_plugin_bases::kWBC for the Go2
// JointTorqueVelocityPositionCommands path, kWBCCartesian for the Cartesian
// path). #13 (M3.2) collapses both into one once the interface stops being a
// template. See plugin_lifecycle.md §6.
//
// This translation unit links fmt as header-only (FMT_HEADER_ONLY, set by the
// build) because wbc_arc_opt.cpp uses fmt::format/print and the image's static
// libfmt.a is not position-independent — it cannot go into a shared object
// (stage_loading.md §6). Header-only fmt keeps the archive off the link line
// with identical runtime formatting.

#include <pluginlib/class_list_macros.hpp>

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/sequence_containers.hpp"
#include "mit_controller/inverse_dynamics.hpp"
#include "mit_controller/joint_commands.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/wbc_arc_opt.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

/**
 * Stock plugin wrapping WBCArcOPT (WBCInterface<JointTorqueVelocityPositionCommands>,
 * the Go2 / joint-command path). Init mirrors mit_controller_node.cpp:516-537.
 */
class WbcArcOptPlugin final : public StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>> {
 public:
  void Init(StageInit init) override {
    // Named locals: as_eigen_vector returns an Eigen::Map view, so the backing
    // vectors must outlive the WBCArcOPT constructor call below.
    const auto feet_names = Require<std::vector<std::string>>(init, "wbc.arc_opt.feet_names");
    const auto joint_names = Require<std::vector<std::string>>(init, "wbc.arc_opt.joint_names");
    const auto com_pose_weight = Require<std::vector<double>>(init, "wbc.arc_opt.com_pose_weight");
    const auto foot_pose_weight = Require<std::vector<double>>(init, "wbc.arc_opt.foot_pose_weight");
    const auto foot_force_weight = Require<std::vector<double>>(init, "wbc.arc_opt.foot_force_weight");
    const auto com_pose_kp = Require<std::vector<double>>(init, "wbc.arc_opt.com_pose_Kp");
    const auto com_pose_kd = Require<std::vector<double>>(init, "wbc.arc_opt.com_pose_Kd");
    const auto feet_pose_kp = Require<std::vector<double>>(init, "wbc.arc_opt.feet_pose_Kp");
    const auto feet_pose_kd = Require<std::vector<double>>(init, "wbc.arc_opt.feet_pose_Kd");
    const auto com_pose_saturation = GetOr<std::vector<double>>(
        init, "wbc.arc_opt.com_pose_saturation",
        {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});
    const auto feet_pose_saturation = GetOr<std::vector<double>>(
        init, "wbc.arc_opt.feet_pose_saturation",
        {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max()});

    impl_ = std::make_unique<WBCArcOPT>(
        std::move(init.state),
        GetOr<std::string>(init, "wbc.arc_opt.solver", "EiquadprogSolver"),
        GetOr<std::string>(init, "wbc.arc_opt.scene", "AccelerationSceneReducedTSID"),
        Require<std::string>(init, "wbc.arc_opt.model_urdf"),
        to_array<ModelInterface::N_LEGS>(feet_names),
        to_array<ModelInterface::NUM_JOINTS>(joint_names),
        Require<double>(init, "wbc.arc_opt.mu"),
        as_eigen_vector<6>(com_pose_weight),
        as_eigen_vector<3>(foot_pose_weight),
        as_eigen_vector<3>(foot_force_weight),
        as_eigen_vector<6>(com_pose_kp),
        as_eigen_vector<6>(com_pose_kd),
        as_eigen_vector<3>(feet_pose_kp),
        as_eigen_vector<3>(feet_pose_kd),
        as_eigen_vector<6>(com_pose_saturation),
        as_eigen_vector<3>(feet_pose_saturation),
        GetOr<double>(init, "wbc_solver_tolerances", -1.0));
  }

  void UpdateState(const StateInterface& quad_state) override { impl_->UpdateState(quad_state); }
  void UpdateModel(const ModelInterface& quad_model) override { impl_->UpdateModel(quad_model); }
  void UpdateFeetTarget(const FeetTargets& feet_targets) override { impl_->UpdateFeetTarget(feet_targets); }
  void UpdateWrenches(const Wrenches& wrenches) override { impl_->UpdateWrenches(wrenches); }
  void UpdateFootContact(const FootContact& foot_contact) override { impl_->UpdateFootContact(foot_contact); }
  void UpdateTarget(const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& position,
                    const Eigen::Vector3d& lin_vel,
                    const Eigen::Vector3d& ang_vel) override {
    impl_->UpdateTarget(orientation, position, lin_vel, ang_vel);
  }
  WBCReturn GetJointCommand(JointTorqueVelocityPositionCommands& joint_command) override {
    return impl_->GetJointCommand(joint_command);
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<WBCArcOPT> impl_;
};

/**
 * Stock plugin wrapping InverseDynamics (WBCInterface<CartesianCommands>, the
 * ULab / Cartesian path). Init mirrors mit_controller_node.cpp:539-545.
 */
class InverseDynamicsPlugin final : public StagePlugin<WBCInterface<CartesianCommands>> {
 public:
  void Init(StageInit init) override {
    impl_ = std::make_unique<InverseDynamics>(
        std::move(init.model),
        std::move(init.state),
        Require<bool>(init, "wbc.inverse_dynamics.foot_position_based_on_target_height"),
        Require<bool>(init, "wbc.inverse_dynamics.foot_position_based_on_target_orientation"),
        GetOr<int64_t>(init, "wbc.inverse_dynamics.transformation_filter_size", 20),
        GetOr<double>(init, "wbc.inverse_dynamics.target_velocity_blend", 0.0));
  }

  void UpdateState(const StateInterface& quad_state) override { impl_->UpdateState(quad_state); }
  void UpdateModel(const ModelInterface& quad_model) override { impl_->UpdateModel(quad_model); }
  void UpdateFeetTarget(const FeetTargets& feet_targets) override { impl_->UpdateFeetTarget(feet_targets); }
  void UpdateWrenches(const Wrenches& wrenches) override { impl_->UpdateWrenches(wrenches); }
  void UpdateFootContact(const FootContact& foot_contact) override { impl_->UpdateFootContact(foot_contact); }
  void UpdateTarget(const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& position,
                    const Eigen::Vector3d& lin_vel,
                    const Eigen::Vector3d& ang_vel) override {
    impl_->UpdateTarget(orientation, position, lin_vel, ang_vel);
  }
  WBCReturn GetJointCommand(CartesianCommands& joint_command) override {
    return impl_->GetJointCommand(joint_command);
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<InverseDynamics> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::WbcArcOptPlugin, StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>)
PLUGINLIB_EXPORT_CLASS(stock_plugins::InverseDynamicsPlugin, StagePlugin<WBCInterface<CartesianCommands>>)
