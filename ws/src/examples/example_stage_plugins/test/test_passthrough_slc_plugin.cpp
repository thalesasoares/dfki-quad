// Tests for the example out-of-package SLC plugin (issue #17, M4.2).
//
// These run the plugin through exactly the machinery the host uses: a production
// `StageLoader` built on the ament index, with no explicit XML path. That is the
// point of the file. `controllers`' own test_stock_plugins.cpp proves the same
// for the in-package stock plugins; this one proves it across a package boundary,
// which is the property M4 exists to establish and which nothing in the
// `controllers` test suite can assert on its own.
//
// The model/state doubles below are written from scratch rather than reused:
// `controllers` keeps BrickModel/BrickState (potato_sim) private, so a third
// party cannot borrow them. Writing the fakes here is therefore not a shortcut
// around a missing export — it is the honest demonstration that the exported
// pure-virtual interfaces are sufficient to test a stage in isolation, and the
// fakes themselves are material for the #18 contributor guide.

#include <gtest/gtest.h>

#include <rclcpp/parameter_value.hpp>

#include <array>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "mit_controller/feet_targets.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"

namespace {

using ParamMap = std::map<std::string, rclcpp::ParameterValue>;

// The stage selection key. The production vocabulary is `slc.type`
// (controllers' stage_selection.hpp), and it is used here rather than an
// arbitrary string so the test also documents the real launch-time spelling.
constexpr const char* kTypeKey = "slc.type";
constexpr const char* kPluginId = "example_passthrough_slc";
constexpr const char* kLogPeriodKey = "example_passthrough_slc.log_period";

// -----------------------------------------------------------------------------
// Test doubles
// -----------------------------------------------------------------------------

// A minimal StateInterface. Everything is a fixed, inspectable constant: this
// stage reads the state only by handing it to the model's forward kinematics, so
// nothing here needs to be physically consistent — it needs to be *identifiable*
// when it comes back out the other side.
class FakeState final : public StateInterface {
 public:
  const Eigen::Vector3d& GetPositionInWorld() const override { return position_; }
  const Eigen::Quaterniond& GetOrientationInWorld() const override { return orientation_; }
  const Eigen::Vector3d& GetLinearVelInWorld() const override { return zero_; }
  const Eigen::Vector3d& GetAngularVelInWorld() const override { return zero_; }
  const Eigen::Vector3d& GetLinearAccInWorld() const override { return zero_; }
  const Eigen::Vector3d& GetAngularAccInWorld() const override { return zero_; }
  const TimePoint& GetTime() const override { return time_; }
  const std::array<bool, NUM_FEET>& GetFeetContacts() const override { return contacts_; }
  const std::array<Eigen::Vector3d, NUM_FEET>& GetContactForces() const override { return forces_; }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointPositions() const override {
    return joints_;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointVelocities() const override {
    return joints_;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointAccelerations() const override {
    return joints_;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointTorques() const override {
    return joints_;
  }
  void SetVelocitiesToZero() override {}
  void SetAccelerationsToZero() override {}
  StateInterface& operator=(const StateInterface& other) override {
    if (this != &other) {
      position_ = other.GetPositionInWorld();
      orientation_ = other.GetOrientationInWorld();
    }
    return *this;
  }

  void SetPosition(const Eigen::Vector3d& position) { position_ = position; }

 private:
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d zero_{Eigen::Vector3d::Zero()};
  TimePoint time_{};
  std::array<bool, NUM_FEET> contacts_{{true, true, true, true}};
  std::array<Eigen::Vector3d, NUM_FEET> forces_{};
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joints_{};
};

// A minimal ModelInterface. Only CalcFootPositionInWorld carries meaning: it
// returns the body position offset by a distinct per-leg vector, so a foot target
// coming back out of the plugin can be attributed to a specific leg and a
// specific state. Every other method is an unused stub — the plugin calls none of
// them, and that fact is itself part of what the test pins.
class FakeModel final : public ModelInterface {
 public:
  // The per-leg offsets the expectations below are written against.
  static Eigen::Vector3d LegOffset(unsigned int leg) {
    return Eigen::Vector3d(0.1 * static_cast<double>(leg + 1), -0.2 * static_cast<double>(leg + 1), -0.3);
  }

  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx, const StateInterface& state) const override {
    return state.GetPositionInWorld() + LegOffset(foot_idx);
  }

  // ---- Unused by this stage: stubs. -----------------------------------------
  void CalcFootForceVelocityInBodyFrame(int,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        Eigen::Ref<Eigen::Vector3d>,
                                        Eigen::Ref<Eigen::Vector3d>) const override {}
  void CalcFootForceVelocityBodyFrame(int, const StateInterface&, Eigen::Vector3d&, Eigen::Vector3d&) const override {}
  void CalcLegInverseKinematicsInBody(int,
                                      const Eigen::Vector3d&,
                                      const Eigen::Vector3d&,
                                      Eigen::Vector3d&) const override {}
  void CalcLegDiffKinematicsBodyFrame(int,
                                      const StateInterface&,
                                      Eigen::Vector3d&,
                                      Eigen::Vector3d&,
                                      Eigen::Vector3d&,
                                      Eigen::Vector3d&) const override {}
  void CalcJacobianLegBase(int, Eigen::Vector3d, Eigen::Matrix3d&) const override {}
  void CalcFwdKinLegBody(int, const Eigen::Vector3d&, Eigen::Matrix4d&, Eigen::Vector3d&) const override {}
  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx,
                                          const Eigen::Vector3d& body_pos,
                                          const Eigen::Quaterniond&,
                                          const Eigen::Vector3d&) const override {
    return body_pos + LegOffset(foot_idx);
  }
  Eigen::Vector3d CalcFootPositionInBodyFrame(unsigned int foot_idx, const Eigen::Vector3d&) const override {
    return LegOffset(foot_idx);
  }
  Eigen::Matrix3d GetInertia() const override { return Eigen::Matrix3d::Identity(); }
  double GetInertia(const int, const int) const override { return 1.0; }
  Eigen::Matrix3d getBaseInertia() const override { return Eigen::Matrix3d::Identity(); }
  void getLegInertia(const std::array<Eigen::Vector<double, N_JOINTS_PER_LEG>, N_LEGS>&,
                     Eigen::Ref<Eigen::Matrix3d>) const override {}
  double GetMass() const override { return 1.0; }
  double GetLegMass(int) const override { return 0.1; }
  double getBaseMass() const override { return 0.6; }
  double GetG() const override { return 9.81; }
  void CalcBaseHeight(const std::array<bool, 4>&,
                      const std::array<const Eigen::Vector3d, 4>&,
                      const Eigen::Quaterniond&,
                      double& base_height) const override {
    base_height = 0.0;
  }
  bool IsLyingDown(const std::array<const Eigen::Vector3d, N_LEGS>&,
                   const std::array<double, N_LEGS>&) const override {
    return false;
  }
  double ComputeKineticEnergy(int, const Eigen::Vector3d&, const Eigen::Vector3d&) const override { return 0.0; }
  double ComputeEnergyDerivative(int, const Eigen::Vector3d&, const Eigen::Vector3d&) const override { return 0.0; }
  void ComputeMomentumSignal(const StateInterface&, Eigen::Vector<double, NUM_JOINTS + 6>&) const override {}
  void ComputeGeneralizedMomentum(const Eigen::Vector<double, NUM_JOINTS + 7>&,
                                  const Eigen::Vector<double, NUM_JOINTS + 6>&,
                                  Eigen::Vector<double, NUM_JOINTS + 6>&) const override {}
  void ComputeEstimatedForces(int,
                              const Eigen::Vector<double, N_JOINTS_PER_LEG>&,
                              const Eigen::Vector<double, N_JOINTS_PER_LEG>&,
                              Eigen::Vector3d&) const override {}
  void ComputeRegressorMatrix(const StateInterface&,
                              Eigen::Matrix<double, NUM_JOINTS + 6, (NUM_JOINTS + 1) * 10>&) const override {}
  void SetInertia(const Eigen::Matrix3d&) override {}
  void SetInertia(const int, const int, const double) override {}
  void SetMass(double) override {}
  void SetCOM(const Eigen::Vector3d&) override {}
  void SetCOM(const int, const double) override {}
  const Eigen::Vector<double, (1 + NUM_JOINTS) * 10>& GetAllDynamicParameters() override { return dynamic_params_; }
  Eigen::Translation3d GetBodyToIMU() const override { return Eigen::Translation3d::Identity(); }
  Eigen::Translation3d GetBodyToBellyBottom() const override { return Eigen::Translation3d::Identity(); }
  Eigen::Translation3d GetBodyToBellyBottom(unsigned int) const override { return Eigen::Translation3d::Identity(); }
  Eigen::Translation3d GetBodyToCOM() const override { return Eigen::Translation3d::Identity(); }
  Eigen::Translation3d GetBodyToLegCOM(int) const override { return Eigen::Translation3d::Identity(); }
  Eigen::Translation3d GetBodyToLegBase(int) const override { return Eigen::Translation3d::Identity(); }
  Eigen::Vector3d getBaseCOM() const override { return Eigen::Vector3d::Zero(); }
  ModelInterface& operator=(const ModelInterface&) override { return *this; }

 private:
  Eigen::Vector<double, (1 + NUM_JOINTS) * 10> dynamic_params_{
      Eigen::Vector<double, (1 + NUM_JOINTS) * 10>::Zero()};
};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

StageInit MakeInit(ParamMap params, const Eigen::Vector3d& body_position = Eigen::Vector3d::Zero()) {
  StageInit init;
  init.model = std::make_unique<FakeModel>();
  auto state = std::make_unique<FakeState>();
  state->SetPosition(body_position);
  init.state = std::move(state);
  params.emplace(kTypeKey, rclcpp::ParameterValue(std::string(kPluginId)));
  init.params = std::move(params);
  return init;
}

// A gait sequence whose *current* step (index 0) schedules the given per-leg
// stance flags. Everything else stays value-initialised; this stage reads only
// contact_sequence[0].
GaitSequence MakeGaitSequence(const std::array<bool, N_LEGS>& stance_now) {
  GaitSequence gs{};
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    gs.contact_sequence[0][leg] = stance_now[leg];
  }
  return gs;
}

// -----------------------------------------------------------------------------
// 1. Discovery across the package boundary — the M4.2 headline.
// -----------------------------------------------------------------------------

// The host's own loader, constructed exactly as the host constructs it (package
// `controllers`, base `StagePlugin<SwingLegControllerInterface>`, ament index
// discovery), finds a plugin that ships from a different package. No allowlist
// was edited and no `controllers` source was touched to make this pass.
TEST(ExamplePassthroughSlc, IsDiscoverableThroughTheProductionLoader) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), kPluginId), declared.end())
      << "the example plugin is not declared for base " << stage_plugin_bases::kSwingLegController
      << " — is example_stage_plugins installed and its description XML exported?";
}

// The stock SLC is still declared for the same base. Adding a stage must be
// purely additive: an example plugin that displaced `bezier_swing` would be a
// regression of the stock path, not an extension of it.
TEST(ExamplePassthroughSlc, DoesNotDisplaceTheStockSwingLegController) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), "bezier_swing"), declared.end());
}

// -----------------------------------------------------------------------------
// 2. Full lifecycle: load, Init, one interface cycle.
// -----------------------------------------------------------------------------

TEST(ExamplePassthroughSlc, LoadsInitialisesAndHoldsFeetAtTheirCurrentPosition) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{}));
  ASSERT_NE(plugin, nullptr);

  // One full cycle in the documented order: gait sequence, state, then the
  // getters (stage_contracts.md §4).
  const Eigen::Vector3d body(1.0, 2.0, 0.3);
  FakeState state;
  state.SetPosition(body);
  plugin->UpdateGaitSequence(MakeGaitSequence({{true, true, true, true}}));
  plugin->UpdateState(state);

  FeetTargets targets{};
  plugin->GetFeetTargets(targets);
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    // The identity: target == where the foot already is.
    EXPECT_TRUE(targets.positions[leg].isApprox(body + FakeModel::LegOffset(leg)))
        << "leg " << leg << " target is not the current foot position";
    EXPECT_TRUE(targets.velocities[leg].isZero()) << "leg " << leg << " should be commanded no velocity";
    EXPECT_TRUE(targets.accelerations[leg].isZero()) << "leg " << leg << " should be commanded no acceleration";
  }
}

// The stance/swing mapping that keeps this stage benign downstream: a scheduled
// stance leg is STANCE at progress 0; a scheduled swing leg is a swing that is
// already complete (REACHED at progress 1) rather than one in flight.
TEST(ExamplePassthroughSlc, ReportsScheduledStanceAsStanceAndScheduledSwingAsReached) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{}));
  ASSERT_NE(plugin, nullptr);

  plugin->UpdateGaitSequence(MakeGaitSequence({{true, false, false, true}}));
  plugin->UpdateState(FakeState{});

  std::array<double, N_LEGS> progress{};
  std::array<SwingLegControllerInterface::LegState, N_LEGS> states{};
  plugin->GetProgress(progress, states);

  EXPECT_EQ(states[0], SwingLegControllerInterface::STANCE);
  EXPECT_EQ(states[1], SwingLegControllerInterface::REACHED);
  EXPECT_EQ(states[2], SwingLegControllerInterface::REACHED);
  EXPECT_EQ(states[3], SwingLegControllerInterface::STANCE);
  EXPECT_DOUBLE_EQ(progress[0], 0.0);
  EXPECT_DOUBLE_EQ(progress[1], 1.0);
  EXPECT_DOUBLE_EQ(progress[2], 1.0);
  EXPECT_DOUBLE_EQ(progress[3], 0.0);
}

// A held foot has a zero-length swing trajectory. Pinned because this getter is
// the one the host may call concurrently from the control loop, so its
// behaviour should be stated rather than assumed.
TEST(ExamplePassthroughSlc, ReportsADegenerateSwingTrajectory) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{}));
  ASSERT_NE(plugin, nullptr);

  const Eigen::Vector3d body(0.5, -0.5, 0.25);
  FakeState state;
  state.SetPosition(body);
  plugin->UpdateState(state);

  std::array<Eigen::Vector3d, N_LEGS> start{};
  std::array<Eigen::Vector3d, N_LEGS> end{};
  plugin->GetCurrentTrajs(start, end);
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    EXPECT_TRUE(start[leg].isApprox(end[leg])) << "leg " << leg << " should not be swinging anywhere";
    EXPECT_TRUE(start[leg].isApprox(body + FakeModel::LegOffset(leg)));
  }
}

// Init seeds the held positions from the state clone the host provides, so a
// GetFeetTargets that arrives before the first UpdateState returns real foot
// positions rather than the origin. Without the seed the first commanded targets
// would be (0,0,0) — a stage that yanks the feet to the world origin for one
// cycle at bring-up.
TEST(ExamplePassthroughSlc, SeedsHeldPositionsFromTheInitStateClone) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  const Eigen::Vector3d body_at_init(3.0, -1.0, 0.4);
  auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{}, body_at_init));
  ASSERT_NE(plugin, nullptr);

  FeetTargets targets{};
  plugin->GetFeetTargets(targets);  // deliberately no UpdateState first
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    EXPECT_TRUE(targets.positions[leg].isApprox(body_at_init + FakeModel::LegOffset(leg)))
        << "leg " << leg << " was not seeded from the Init state clone";
  }
}

// -----------------------------------------------------------------------------
// 3. Fail-fast on bad configuration.
// -----------------------------------------------------------------------------

// The optional key is genuinely optional: absent means logging off, not an error.
TEST(ExamplePassthroughSlc, InitialisesWithNoParametersAtAll) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  EXPECT_NO_THROW({ auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{})); });
}

// A malformed value is a StageInitError that names the key — the same fail-fast
// contract the stock plugins keep (plugin_lifecycle.md §1 rule 4). The two cases
// differ in kind: a negative period is a well-typed but meaningless value, a
// string is the wrong type entirely, and both must be refused rather than
// silently coerced.
TEST(ExamplePassthroughSlc, NegativeLogPeriodIsAFatalNamedError) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  ParamMap params;
  params.emplace(kLogPeriodKey, rclcpp::ParameterValue(-1.0));
  try {
    auto plugin = loader.Load(kTypeKey, MakeInit(std::move(params)));
    ADD_FAILURE() << "a negative log period should not initialise";
  } catch (const StageInitError& error) {
    EXPECT_NE(std::string(error.what()).find(kLogPeriodKey), std::string::npos)
        << "the error must name the offending key, got: " << error.what();
  }
}

TEST(ExamplePassthroughSlc, WronglyTypedLogPeriodIsAFatalNamedError) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  ParamMap params;
  params.emplace(kLogPeriodKey, rclcpp::ParameterValue(std::string("often")));
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit(std::move(params))), StageInitError);
}

// -----------------------------------------------------------------------------
// 4. Runtime reconfiguration.
// -----------------------------------------------------------------------------

// The host uses SetParameter's return value to decide whether to warn that a
// change did not apply, so both answers are part of the contract. `bio_gait`
// (#16) pinned the all-false case; this pins the true case and the rejection of
// a well-named but invalid value.
TEST(ExamplePassthroughSlc, AppliesTheLogPeriodAtRuntimeAndRejectsAnythingElse) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  auto plugin = loader.Load(kTypeKey, MakeInit(ParamMap{}));
  ASSERT_NE(plugin, nullptr);

  EXPECT_TRUE(plugin->SetParameter(kLogPeriodKey, rclcpp::ParameterValue(2.0)));
  EXPECT_TRUE(plugin->SetParameter(kLogPeriodKey, rclcpp::ParameterValue(0.0)));  // 0 disables

  // Recognised key, unusable value: rejected rather than applied, and reported
  // as not-applied so the host warns.
  EXPECT_FALSE(plugin->SetParameter(kLogPeriodKey, rclcpp::ParameterValue(-2.0)));
  EXPECT_FALSE(plugin->SetParameter(kLogPeriodKey, rclcpp::ParameterValue(std::string("often"))));

  // Keys belonging to the stock SLC are not silently swallowed just because this
  // stage occupies the same slot.
  EXPECT_FALSE(plugin->SetParameter("slc_swing_height", rclcpp::ParameterValue(0.05)));
  EXPECT_FALSE(plugin->SetParameter("not.a.key", rclcpp::ParameterValue(1)));
}

// -----------------------------------------------------------------------------
// 5. Loader diagnostics.
// -----------------------------------------------------------------------------

// A misspelled selection is a StageLoadError (never a silent fallback), and the
// message lists what *was* available — including this package's plugin, which is
// what turns "my out-of-package plugin isn't being found" into a one-line check.
TEST(ExamplePassthroughSlc, UnknownSelectionIsFatalAndListsThisPluginAmongTheAlternatives) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  ParamMap params;
  StageInit init = MakeInit(std::move(params));
  init.params[kTypeKey] = rclcpp::ParameterValue(std::string("example_pasthrough_slc"));  // typo, deliberately
  try {
    auto plugin = loader.Load(kTypeKey, std::move(init));
    ADD_FAILURE() << "a misspelled slc.type should be fatal";
  } catch (const StageLoadError& error) {
    EXPECT_NE(std::string(error.what()).find(kPluginId), std::string::npos)
        << "the error should list the declared plugins, got: " << error.what();
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
