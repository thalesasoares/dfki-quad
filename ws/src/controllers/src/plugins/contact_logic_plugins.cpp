// Stock contact-logic plugin (issue #12, M3.1).
//
// Thin adapter (plugin_lifecycle.md §4, Option B) around DefaultContactLogic,
// the FSM this milestone lifted out of MITController::ControlLoopCallback. Unlike
// the M2.3 wrappers there is no host factory body to relocate — the host never
// constructed a contact stage — so Init is just the four ratified toggles plus
// the model/state clones.
//
// The stock id is `default_contact_logic`: the reconciliation policy the
// controller has always run.

#include <pluginlib/class_list_macros.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>

#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/default_contact_logic.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

/**
 * Stock plugin wrapping DefaultContactLogic.
 */
class DefaultContactLogicPlugin final : public StagePlugin<ContactLogicInterface> {
 public:
  void Init(StageInit init) override {
    // All four keys are Require<bool>, not GetOr: the host declares every one of
    // them (deriving the default from the pre-M3.1 flat spelling where a config
    // still uses it), so an absent key means the stage is being loaded by
    // something that has not configured it — a fail-fast, not a default.
    impl_ = std::make_unique<DefaultContactLogic>(
        Require<bool>(init, contact_logic_params::kEarlyContactDetection),
        Require<bool>(init, contact_logic_params::kLateContactDetection),
        Require<bool>(init, contact_logic_params::kLostContactDetection),
        Require<bool>(init, contact_logic_params::kLateContactRescheduleSwingPhase),
        std::move(init.model),
        std::move(init.state));
  }

  void UpdateState(const StateInterface& state) override { impl_->UpdateState(state); }
  void UpdateGaitSequence(const GaitSequence& gs) override { impl_->UpdateGaitSequence(gs); }
  void UpdateWrenchSequence(const WrenchSequence& ws) override { impl_->UpdateWrenchSequence(ws); }
  void UpdateSwingLegState(const FeetTargets& feet_targets,
                           const std::array<double, N_LEGS>& swing_progress,
                           const std::array<SwingLegControllerInterface::LegState, N_LEGS>& swing_states) override {
    impl_->UpdateSwingLegState(feet_targets, swing_progress, swing_states);
  }
  void UpdateModel(const ModelInterface& model) override { impl_->UpdateModel(model); }
  void Reconcile(FootContacts& contacts, Wrenches& wrenches, FeetTargets& feet_targets) override {
    impl_->Reconcile(contacts, wrenches, feet_targets);
  }
  void GetLegContactStates(std::array<LegContactState, N_LEGS>& states) const override {
    impl_->GetLegContactStates(states);
  }
  void GetContactEvents(ContactEvents& events) const override { impl_->GetContactEvents(events); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<DefaultContactLogic> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::DefaultContactLogicPlugin, StagePlugin<ContactLogicInterface>)
