// Stock swing-leg-controller plugin (issue #8, M2.3).
//
// Thin adapter (plugin_lifecycle.md §4, Option B) around SwingLegController.
// Init mirrors the host's SLC construction (mit_controller_node.cpp:503-508).
// The stock id is `bezier_swing`: the swing trajectories are Bezier splines
// (SwingTrajectory / FootSwingTrajectory).

#include <pluginlib/class_list_macros.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>

#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

/**
 * Stock plugin wrapping SwingLegController.
 */
class BezierSwingPlugin final : public StagePlugin<SwingLegControllerInterface> {
 public:
  void Init(StageInit init) override {
    impl_ = std::make_unique<SwingLegController>(
        Require<double>(init, "slc_swing_height"),
        Require<double>(init, "maximum_swing_leg_progress_to_update_target"),
        GetOr<double>(init, "slc_world_blend", 1.0),
        std::move(init.model),
        std::move(init.state));
  }

  void UpdateGaitSequence(const GaitSequence& gs) override { impl_->UpdateGaitSequence(gs); }
  void UpdateState(const StateInterface& state) override { impl_->UpdateState(state); }
  void UpdateModel(const ModelInterface& model) override { impl_->UpdateModel(model); }
  void GetFeetTargets(FeetTargets& feet_targets) override { impl_->GetFeetTargets(feet_targets); }
  void GetProgress(std::array<double, N_LEGS>& progress, std::array<LegState, N_LEGS>& swing_states) override {
    impl_->GetProgress(progress, swing_states);
  }
  void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS>& start_pos,
                       std::array<Eigen::Vector3d, N_LEGS>& end_pos) override {
    impl_->GetCurrentTrajs(start_pos, end_pos);
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<SwingLegController> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::BezierSwingPlugin, StagePlugin<SwingLegControllerInterface>)
