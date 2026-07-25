// Stock stage plugin tests (issue #8, M2.3).
//
// M2.2's test_stage_loader.cpp proved the loader mechanics against a synthetic
// test plugin. This test proves the real thing the milestone ships: the stock
// wrappers around GS / MPC / SLC / WBC / MA are discoverable through production
// ament discovery (no explicit XML path), each dlopens and constructs, each Init
// runs the relocated host-factory body, and the fail-fast contract holds — a
// missing required parameter or an unknown enum value is a StageInitError that
// names the offending key/value, never a silent substitution.
//
// The test executable itself links none of acados / ARC-OPT / fmt: the algorithm
// code lives in the plugin .so files, which pluginlib dlopens. A successful
// Create() of libwbc_plugins is therefore also the runtime proof that the
// non-PIC-libfmt link story (stage_loading.md §6) actually resolves at load time.
//
// Model/state clones come from potato_sim's BrickModel/BrickState, the same
// lightweight ModelInterface/StateInterface the loader test uses. The ament
// resource is in the install space, so the CMake target appends
// CMAKE_INSTALL_PREFIX to AMENT_PREFIX_PATH (see CMakeLists.txt).

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "mit_controller/feet_targets.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/gait_sequencer_types.hpp"
#include "mit_controller/joint_commands.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/mpc_prediction.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/target.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "mit_controller/wrench_sequence.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "potato_sim/potato_model.hpp"

namespace {

// A generic selection key. The vocabulary belongs to M2.5 (#10); the loader takes
// it as an argument, so this test names it whatever it likes.
constexpr const char* kTypeKey = "stage.type";

using ParamMap = std::map<std::string, rclcpp::ParameterValue>;

// A parameter map with every key the stock wrappers *require* (the type-only
// declares in the host), with representative values. Optional keys are omitted so
// the wrappers exercise their in-stage defaults (plugin_lifecycle.md §3).
ParamMap BaseParams() {
  ParamMap p;
  // Gait sequencer required keys.
  p.emplace("gs_shoulder_positions", rclcpp::ParameterValue(std::vector<double>{
                                         0.2, 0.15, 0.0, 0.2, -0.15, 0.0, -0.2, 0.15, 0.0, -0.2, -0.15, 0.0}));
  p.emplace("fix_standing_position", rclcpp::ParameterValue(false));
  p.emplace("early_contact_detection", rclcpp::ParameterValue(false));
  // MPC required keys (12 = MPC::STATE_SIZE - 1).
  const std::vector<double> weights(12, 1.0);
  p.emplace("mpc_alpha", rclcpp::ParameterValue(1e-3));
  p.emplace("mpc_state_weights_stand", rclcpp::ParameterValue(weights));
  p.emplace("mpc_state_weights_move", rclcpp::ParameterValue(weights));
  p.emplace("mpc_mu", rclcpp::ParameterValue(0.6));
  p.emplace("mpc_fmin", rclcpp::ParameterValue(5.0));
  p.emplace("mpc_fmax", rclcpp::ParameterValue(300.0));
  // SLC required keys.
  p.emplace("slc_swing_height", rclcpp::ParameterValue(0.05));
  p.emplace("maximum_swing_leg_progress_to_update_target", rclcpp::ParameterValue(0.9));
  // Inverse-dynamics WBC required keys.
  p.emplace("wbc.inverse_dynamics.foot_position_based_on_target_height", rclcpp::ParameterValue(false));
  p.emplace("wbc.inverse_dynamics.foot_position_based_on_target_orientation", rclcpp::ParameterValue(false));
  return p;
}

// Builds the StageInit the host hands a stage: model/state clones + the parameter
// map, with the selection key set to `type_value`.
StageInit MakeInit(const std::string& type_value, ParamMap params) {
  StageInit init;
  init.model = std::make_unique<BrickModel>(Eigen::Matrix3d::Identity(), 1.0);
  init.state = std::make_unique<BrickState>();
  params.emplace(kTypeKey, rclcpp::ParameterValue(type_value));
  init.params = std::move(params);
  return init;
}

// -----------------------------------------------------------------------------
// Success: each stock plugin is discovered, constructed and initialised through
// production ament discovery, then driven one cycle through its frozen interface.
// -----------------------------------------------------------------------------

TEST(StockPlugins, SimpleGaitLoadsInitialisesAndRuns) {
  StageLoader<GaitSequencerInterface> loader(stage_plugin_bases::kGaitSequencer);
  auto plugin = loader.Load(kTypeKey, MakeInit("simple_gait", BaseParams()));
  ASSERT_NE(plugin, nullptr);
  plugin->UpdateTarget(Target{});
  plugin->UpdateState(BrickState{});
  GaitSequence sequence{};
  plugin->GetGaitSequence(sequence);
  EXPECT_EQ(plugin->GetType(), GS_Type::SIMPLE);
}

TEST(StockPlugins, AdaptiveGaitLoadsInitialisesAndRuns) {
  StageLoader<GaitSequencerInterface> loader(stage_plugin_bases::kGaitSequencer);
  auto plugin = loader.Load(kTypeKey, MakeInit("adaptive_gait", BaseParams()));
  ASSERT_NE(plugin, nullptr);
  plugin->UpdateTarget(Target{});
  plugin->UpdateState(BrickState{});
  GaitSequence sequence{};
  plugin->GetGaitSequence(sequence);
  EXPECT_EQ(plugin->GetType(), GS_Type::ADAPTIVE);
}

// acados_mpc: like wbc_arc_opt, a full Init sets up the acados solver, which
// belongs to the sim regression (M2.6), not a fast unit test. Here we prove the
// class is declared and its library dlopens and constructs (the runtime check
// that libmpc_plugins links acados); the wrapper's MPC-specific logic — the
// solver-name mapping and fail-fast — is covered by AcadosMpcUnknownSolverThrows
// below, which throws during argument evaluation before any acados setup runs.
TEST(StockPlugins, AcadosMpcLibraryLoadsAndConstructs) {
  StageLoader<MPCInterface> loader(stage_plugin_bases::kMPC);
  auto plugin = loader.Create("acados_mpc");
  EXPECT_NE(plugin, nullptr);
}

TEST(StockPlugins, BezierSwingLoadsInitialisesAndRuns) {
  StageLoader<SwingLegControllerInterface> loader(stage_plugin_bases::kSwingLegController);
  auto plugin = loader.Load(kTypeKey, MakeInit("bezier_swing", BaseParams()));
  ASSERT_NE(plugin, nullptr);
  plugin->UpdateState(BrickState{});
  plugin->UpdateGaitSequence(GaitSequence{});
  std::array<double, N_LEGS> progress{};
  std::array<SwingLegControllerInterface::LegState, N_LEGS> states{};
  plugin->GetProgress(progress, states);
}

TEST(StockPlugins, InverseDynamicsLoadsAndInitialises) {
  StageLoader<WBCInterface<CartesianCommands>> loader(stage_plugin_bases::kWBCCartesian);
  auto plugin = loader.Load(kTypeKey, MakeInit("inverse_dynamics", BaseParams()));
  ASSERT_NE(plugin, nullptr);
}

TEST(StockPlugins, KfAdaptationLoadsAndInitialises) {
  StageLoader<ModelAdaptationInterface> loader(stage_plugin_bases::kModelAdaptation);
  auto plugin = loader.Load(kTypeKey, MakeInit("kf_adaptation", BaseParams()));
  ASSERT_NE(plugin, nullptr);
  // GetParameterVector is documented side-effect-free.
  (void)plugin->GetParameterVector();
}

TEST(StockPlugins, RlsAdaptationLoadsAndInitialises) {
  StageLoader<ModelAdaptationInterface> loader(stage_plugin_bases::kModelAdaptation);
  auto plugin = loader.Load(kTypeKey, MakeInit("rls_adaptation", BaseParams()));
  ASSERT_NE(plugin, nullptr);
  (void)plugin->GetParameterVector();
}

// The ARC-OPT WBC needs a real URDF and solver setup for a full Init, which
// belongs to the sim regression (M2.6). Here we prove the class is declared and
// its library dlopens and constructs — the runtime confirmation that libwbc_plugins
// links (ARC-OPT shared libs + header-only fmt, no non-PIC libfmt.a).
TEST(StockPlugins, WbcArcOptLibraryLoadsAndConstructs) {
  StageLoader<WBCInterface<JointTorqueVelocityPositionCommands>> loader(stage_plugin_bases::kWBC);
  auto plugin = loader.Create("wbc_arc_opt");
  EXPECT_NE(plugin, nullptr);
}

// -----------------------------------------------------------------------------
// Fail-fast: missing required keys and unknown enum values are named errors.
// -----------------------------------------------------------------------------

TEST(StockPlugins, SimpleGaitMissingRequiredKeyThrows) {
  StageLoader<GaitSequencerInterface> loader(stage_plugin_bases::kGaitSequencer);
  ParamMap params = BaseParams();
  params.erase("gs_shoulder_positions");
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("simple_gait", params)), StageInitError);
}

TEST(StockPlugins, SimpleGaitUnknownGaitStringThrows) {
  StageLoader<GaitSequencerInterface> loader(stage_plugin_bases::kGaitSequencer);
  ParamMap params = BaseParams();
  params.emplace("simple_gait_sequencer.gait", rclcpp::ParameterValue(std::string("NOT_A_GAIT")));
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("simple_gait", params)), StageInitError);
}

TEST(StockPlugins, AcadosMpcUnknownSolverThrows) {
  StageLoader<MPCInterface> loader(stage_plugin_bases::kMPC);
  ParamMap params = BaseParams();
  params.emplace("mpc_solver", rclcpp::ParameterValue(std::string("NOT_A_SOLVER")));
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("acados_mpc", params)), StageInitError);
}

TEST(StockPlugins, InverseDynamicsMissingRequiredKeyThrows) {
  StageLoader<WBCInterface<CartesianCommands>> loader(stage_plugin_bases::kWBCCartesian);
  ParamMap params = BaseParams();
  params.erase("wbc.inverse_dynamics.foot_position_based_on_target_height");
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("inverse_dynamics", params)), StageInitError);
}

// A misspelled selection value is a StageLoadError (the loader), distinct from a
// stage that cannot configure itself (StageInitError) — never a silent fallback.
TEST(StockPlugins, UnknownSelectionIsFatal) {
  StageLoader<GaitSequencerInterface> loader(stage_plugin_bases::kGaitSequencer);
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("no_such_gait", BaseParams())), StageLoadError);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
