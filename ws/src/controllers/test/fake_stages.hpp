// Fake stage stubs for the control-pipeline contract test (issue #5, M1.5).
//
// One minimal `final` implementation per stage interface. Their only job is to
// *satisfy the contract*: every method is `override`, so if a signature on an
// interface changes, is removed or renamed, the matching stub stops compiling;
// if a new pure virtual is added, the `final` stub turns abstract and the
// instantiation in test_stage_contracts.cpp fails. That is the API-break
// detection issue #5 asks for, and it fires at compile time — i.e. inside the
// ordinary `colcon build`, before anyone runs `colcon test`.
//
// These are deliberately behaviour-free (the concrete stages have their own
// behaviour; this is a shape/plumbing guard, see doc/modularity/contract_tests.md).
// They are written to be reused: the pluginlib stage base and loader test in
// M2.2 (issue #7) can build on these fakes, and `FakeContactLogic`'s pass-through
// `Reconcile` is the skeleton the M4.2 example passthrough plugin (issue #17)
// starts from.
//
// The stubs are held by `std::unique_ptr<Interface>` in the test exactly as the
// host owns its stages (mit_controller_node.hpp:111-117), so they also exercise
// deletion through the base pointer — the virtual-destructor property that gap
// G4 (M1.1) added.

#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/feet_targets.hpp"
#include "mit_controller/gait_interface.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/gait_sequencer_types.hpp"
#include "mit_controller/joint_commands.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/mpc_prediction.hpp"
#include "mit_controller/pipeline_constants.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/target.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "mit_controller/wrench_sequence.hpp"

namespace contract_test {

// --- Gait sequencer (§4.1) ---------------------------------------------------
class FakeGaitSequencer final : public GaitSequencerInterface {
 public:
  void GetGaitSequence(GaitSequence& gait_sequence) override { gait_sequence = GaitSequence{}; }
  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateTarget(const Target& new_target) override { (void)new_target; }
  void GetGaitState(interfaces::msg::GaitState& state) override { state = interfaces::msg::GaitState{}; }
  GS_Type GetType() const override { return GS_Type::SIMPLE; }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

// --- Force optimisation / MPC (§4.2) -----------------------------------------
class FakeMPC final : public MPCInterface {
 public:
  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateGaitSequence(const GaitSequence& gait_sequence) override { (void)gait_sequence; }
  // Contract: all three outputs must be left usable even when the solve did not
  // converge (§4.2). The fake reports success and hands back defaulted plans.
  void GetWrenchSequence(WrenchSequence& wrench_sequence,
                         MPCPrediction& state_prediction,
                         SolverInformation& solver_information) override {
    wrench_sequence = WrenchSequence{};
    state_prediction = MPCPrediction{};
    solver_information = SolverInformation{};
    solver_information.success = true;
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

// --- Swing leg controller (§4.3) ---------------------------------------------
class FakeSwingLegController final : public SwingLegControllerInterface {
 public:
  void UpdateGaitSequence(const GaitSequence& gs) override { (void)gs; }
  void UpdateState(const StateInterface& state) override { (void)state; }
  void UpdateModel(const ModelInterface& model) override { (void)model; }
  void GetFeetTargets(FeetTargets& feet_targets) override { feet_targets = FeetTargets{}; }
  void GetProgress(std::array<double, N_LEGS>& progress, std::array<LegState, N_LEGS>& swing_states) override {
    progress.fill(0.0);
    swing_states.fill(LegState::STANCE);
  }
  void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS>& start_pos,
                       std::array<Eigen::Vector3d, N_LEGS>& end_pos) override {
    start_pos.fill(Eigen::Vector3d::Zero());
    end_pos.fill(Eigen::Vector3d::Zero());
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

// --- Whole-body control (§4.4) -----------------------------------------------
// One fake for a now non-template interface (gap G8 closed by issue #13, M3.2).
// The command family is a runtime property, so the fake takes it as a
// constructor argument and — like every real implementation — solves in its own
// mode and stubs the other. Both getters are exercised by the contract test.
class FakeWBC final : public WBCInterface {
 public:
  explicit FakeWBC(WBCCommandMode mode = WBCCommandMode::kJoint) : mode_(mode) {}

  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateFeetTarget(const FeetTargets& feet_targets) override { (void)feet_targets; }
  void UpdateWrenches(const Wrenches& wrenches) override { (void)wrenches; }
  void UpdateFootContact(const FootContact& foot_contact) override { (void)foot_contact; }
  void UpdateTarget(const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& position,
                    const Eigen::Vector3d& lin_vel,
                    const Eigen::Vector3d& ang_vel) override {
    (void)orientation;
    (void)position;
    (void)lin_vel;
    (void)ang_vel;
  }
  WBCCommandMode SupportedCommandMode() const override { return mode_; }
  WBCReturn GetJointCommand(JointTorqueVelocityPositionCommands& joint_command) override {
    if (mode_ != WBCCommandMode::kJoint) {
      return WBCReturn{false, 0.0, 0.0};
    }
    joint_command = JointTorqueVelocityPositionCommands{};
    return WBCReturn{true, 0.0, 0.0};
  }
  WBCReturn GetCartesianCommand(CartesianCommands& cartesian_command) override {
    if (mode_ != WBCCommandMode::kCartesian) {
      return WBCReturn{false, 0.0, 0.0};
    }
    cartesian_command = CartesianCommands{};
    return WBCReturn{true, 0.0, 0.0};
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }

 private:
  WBCCommandMode mode_;
};

// --- Model adaptation (§4.5) -------------------------------------------------
class FakeModelAdaptation final : public ModelAdaptationInterface {
 public:
  void UpdateState(const StateInterface& state) override { (void)state; }
  void UpdateGaitSequence(const GaitSequence& gs) override { (void)gs; }
  // Contract: the only stage method that may mutate shared pipeline data; on
  // false the host does not broadcast a new model (§4.5). The fake never adapts.
  bool DoModelAdaptation(ModelInterface& model) override {
    (void)model;
    return false;
  }
  Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const override {
    return Eigen::Vector<double, NUM_PARAMS>::Zero();
  }
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const override {
    return Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS>::Zero();
  }
  Eigen::Vector<double, NUM_PARAMS> GetDelta() const override { return Eigen::Vector<double, NUM_PARAMS>::Zero(); }
  Eigen::Vector<double, 6> GetTotalForceTorque() const override { return Eigen::Vector<double, 6>::Zero(); }
  Eigen::Vector<double, NUM_PARAMS> GetSV() const override { return Eigen::Vector<double, NUM_PARAMS>::Zero(); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

// --- Contact reconciliation (§4.6) -------------------------------------------
// Pass-through: `Reconcile` leaves the seeded in/out values untouched, which is
// the identity contact logic and the shape the M4.2 passthrough plugin starts
// from.
class FakeContactLogic final : public ContactLogicInterface {
 public:
  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateGaitSequence(const GaitSequence& gait_sequence) override { (void)gait_sequence; }
  void UpdateWrenchSequence(const WrenchSequence& wrench_sequence) override { (void)wrench_sequence; }
  void UpdateSwingLegState(const FeetTargets& feet_targets,
                           const std::array<double, N_LEGS>& swing_progress,
                           const std::array<SwingLegControllerInterface::LegState, N_LEGS>& swing_states) override {
    (void)feet_targets;
    (void)swing_progress;
    (void)swing_states;
  }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void Reconcile(FootContacts& contacts, Wrenches& wrenches, FeetTargets& feet_targets) override {
    (void)contacts;
    (void)wrenches;
    (void)feet_targets;
  }
  void GetLegContactStates(std::array<LegContactState, N_LEGS>& states) const override {
    states.fill(LegContactState::SWING);
  }
  void GetContactEvents(ContactEvents& events) const override { events = ContactEvents{}; }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

// --- Gait seam (§7) ----------------------------------------------------------
class FakeGait final : public GaitInterface {
 public:
  double get_t_stance(unsigned int leg) const override {
    (void)leg;
    return 0.0;
  }
};

// --- Plugin lifecycle layer (plugin_lifecycle.md) ----------------------------
// A stage as the M2 loader will see it: default-constructed, then handed its
// model/state clones and parameters through Init. One representative
// instantiation guards the shape of the whole `StagePlugin<Interface>`
// template. The fake takes real ownership of the clones and requires one key,
// exercising the StageInitError path the M2.2 loader helper must translate
// into the fail-fast bring-up error.
class FakePluginGaitSequencer final : public StagePlugin<GaitSequencerInterface> {
 public:
  static constexpr const char* kRequiredKey = "contract_test.required";

  void Init(StageInit init) override {
    init.Require(kRequiredKey);
    model_ = std::move(init.model);
    state_ = std::move(init.state);
  }
  bool Initialized() const { return model_ != nullptr && state_ != nullptr; }

  void GetGaitSequence(GaitSequence& gait_sequence) override { gait_sequence = GaitSequence{}; }
  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateTarget(const Target& new_target) override { (void)new_target; }
  void GetGaitState(interfaces::msg::GaitState& state) override { state = interfaces::msg::GaitState{}; }
  GS_Type GetType() const override { return GS_Type::SIMPLE; }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }

 private:
  std::unique_ptr<ModelInterface> model_;
  std::unique_ptr<StateInterface> state_;
};

}  // namespace contract_test
