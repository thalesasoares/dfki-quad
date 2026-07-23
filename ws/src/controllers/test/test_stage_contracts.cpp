// Contract test / compile smoke for the control-pipeline stage interfaces
// (issue #5, M1.5 — the last issue of milestone M1, "Contracts").
//
// This translation unit freezes the stage APIs before the M2 plugin-host work
// (issue #9) starts moving them behind pluginlib. It has two layers:
//
//   1. Compile-time `static_assert`s that pin the shape every stage contract
//      promises — abstract base, virtual destructor, non-constructible base,
//      and the `N_LEGS`-sized aggregates. These assert the §4 method tables of
//      doc/modularity/stage_contracts.md and, together with fake_stages.hpp,
//      fail the *build* on any breaking signature change. They therefore run in
//      CI's ordinary `colcon build` (`cbg`) without a `colcon test` step.
//
//   2. gtest cases that drive each fake stage once through a
//      `std::unique_ptr<Interface>` — the exact ownership the host uses
//      (mit_controller_node.hpp:111-117) — in the documented Update*→Get* call
//      order, then destroy it through the base pointer.
//
// It supersedes the two narrower checks that preceded it:
//   * src/tools/contact_logic_interface_check.cpp (M1.4) — fully absorbed here
//     and removed; its N_LEGS sizing asserts are migrated verbatim below.
//   * src/tools/pipeline_types_surface_check.cpp (M1.3) — *kept*: it guards a
//     different axis (that the exported type headers stay self-contained under a
//     restricted, export-only include path), which this test does not.
//
// The stage interface headers themselves join the exported plugin surface in
// M2.1 (issue #6); until then this test uses the package's normal include paths.
//
// Deps are common/interfaces/rclcpp/Eigen3 only — no drake, acados, ARC-OPT or
// hardware — so it builds and runs headless on both CI lanes (x86_64 and the
// WITHOUT_DRAKE aarch64 onboard build). See doc/modularity/contract_tests.md.

#include "fake_stages.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <utility>

#include "potato_sim/potato_model.hpp"

namespace {

using contract_test::FakeContactLogic;
using contract_test::FakeGait;
using contract_test::FakeGaitSequencer;
using contract_test::FakeModelAdaptation;
using contract_test::FakeMPC;
using contract_test::FakePluginGaitSequencer;
using contract_test::FakeSwingLegController;
using contract_test::FakeWBC;

// -----------------------------------------------------------------------------
// Layer 1: compile-time contract shape.
// -----------------------------------------------------------------------------

// Every stage interface is an abstract base with a virtual destructor that
// cannot be constructed on its own — so the host can own it as
// `unique_ptr<Interface>` and delete through the base pointer without UB
// (gap G4, M1.1). This is the guard the M1.4 contact check applied to one
// interface, applied here to all of them.
template <class Interface>
constexpr bool IsStageContract() {
  return std::is_abstract_v<Interface> && std::has_virtual_destructor_v<Interface> &&
         !std::is_default_constructible_v<Interface>;
}

static_assert(IsStageContract<GaitSequencerInterface>());
static_assert(IsStageContract<MPCInterface>());
static_assert(IsStageContract<SwingLegControllerInterface>());
static_assert(IsStageContract<ModelAdaptationInterface>());
static_assert(IsStageContract<ContactLogicInterface>());
static_assert(IsStageContract<GaitInterface>());

// `WBCInterface` is still a class template (gap G8, issue #13). Pin the contract
// for each of the three joint command types in joint_commands.hpp; none may rot
// before #13 de-templates the interface.
static_assert(IsStageContract<WBCInterface<JointTorqueVelocityPositionCommands>>());
static_assert(IsStageContract<WBCInterface<JointTorqueCommands>>());
static_assert(IsStageContract<WBCInterface<CartesianCommands>>());

// The fakes are concrete: they satisfy the full method table. If an interface
// gains a pure virtual, its `final` fake turns abstract and these fail.
static_assert(!std::is_abstract_v<FakeGaitSequencer>);
static_assert(!std::is_abstract_v<FakeMPC>);
static_assert(!std::is_abstract_v<FakeSwingLegController>);
static_assert(!std::is_abstract_v<FakeModelAdaptation>);
static_assert(!std::is_abstract_v<FakeContactLogic>);
static_assert(!std::is_abstract_v<FakeGait>);
static_assert(!std::is_abstract_v<FakeWBC<JointTorqueVelocityPositionCommands>>);
static_assert(!std::is_abstract_v<FakeWBC<JointTorqueCommands>>);
static_assert(!std::is_abstract_v<FakeWBC<CartesianCommands>>);

// Contact-reconciliation aggregates are per-leg, sized off pipeline_constants.hpp.
// Migrated verbatim from the retired contact_logic_interface_check.cpp so the
// guard is not lost. A change to N_LEGS not mirrored here breaks the build.
static_assert(std::tuple_size_v<ContactLogicInterface::FootContacts> == N_LEGS);
static_assert(std::tuple_size_v<ContactLogicInterface::Wrenches> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::early_contact_detected)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::late_contact_detected)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::lost_contact_detected)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::contact_regained)> == N_LEGS);
static_assert(
    std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::swing_scheduled_before_slc_started)> == N_LEGS);

// The plugin lifecycle layer (doc/modularity/plugin_lifecycle.md) is additive:
// each `StagePlugin<Interface>` base derives from the frozen stage interface and
// stays an abstract contract itself, so putting a stage behind pluginlib reopens
// none of the M1 contracts.
static_assert(IsStageContract<StagePlugin<GaitSequencerInterface>>());
static_assert(IsStageContract<StagePlugin<MPCInterface>>());
static_assert(IsStageContract<StagePlugin<SwingLegControllerInterface>>());
static_assert(IsStageContract<StagePlugin<ModelAdaptationInterface>>());
static_assert(IsStageContract<StagePlugin<ContactLogicInterface>>());
static_assert(IsStageContract<StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>>());
static_assert(IsStageContract<StagePlugin<WBCInterface<JointTorqueCommands>>>());
static_assert(IsStageContract<StagePlugin<WBCInterface<CartesianCommands>>>());
static_assert(std::is_base_of_v<GaitSequencerInterface, StagePlugin<GaitSequencerInterface>>);
static_assert(!std::is_abstract_v<FakePluginGaitSequencer>);

// The host passes contact flags and wrenches straight from ContactLogic into the
// WBC (§4.6). That is only sound while the layouts match; the interfaces restate
// the types rather than share them because WBCInterface is a template (issue #13).
// This pins the compatibility until they are unified.
static_assert(
    std::is_same_v<ContactLogicInterface::FootContacts,
                   WBCInterface<JointTorqueVelocityPositionCommands>::FootContact>);
static_assert(std::is_same_v<ContactLogicInterface::Wrenches,
                             WBCInterface<JointTorqueVelocityPositionCommands>::Wrenches>);

// -----------------------------------------------------------------------------
// Layer 2: runtime plumbing through the base pointer.
// -----------------------------------------------------------------------------
// A concrete StateInterface / ModelInterface the Update* methods can take by
// reference, using the codebase's own "potato" doubles (drake-free).

BrickState MakeState() { return BrickState{}; }

BrickModel MakeModel() { return BrickModel(Eigen::Matrix3d::Identity(), 1.0); }

TEST(StageContracts, GaitSequencerCallOrder) {
  std::unique_ptr<GaitSequencerInterface> gs = std::make_unique<FakeGaitSequencer>();
  const BrickState state = MakeState();
  const BrickModel model = MakeModel();

  gs->UpdateTarget(Target{});
  gs->UpdateState(state);
  gs->UpdateModel(model);
  GaitSequence sequence{};
  gs->GetGaitSequence(sequence);
  interfaces::msg::GaitState gait_state;
  gs->GetGaitState(gait_state);

  EXPECT_EQ(gs->GetType(), GS_Type::SIMPLE);
  // Unrecognised keys return false — the documented "no runtime params" reply.
  EXPECT_FALSE(gs->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

TEST(StageContracts, MpcCallOrder) {
  std::unique_ptr<MPCInterface> mpc = std::make_unique<FakeMPC>();
  const BrickState state = MakeState();
  const BrickModel model = MakeModel();

  mpc->UpdateState(state);
  mpc->UpdateModel(model);
  mpc->UpdateGaitSequence(GaitSequence{});
  WrenchSequence wrenches{};
  MPCPrediction prediction{};
  SolverInformation info{};
  mpc->GetWrenchSequence(wrenches, prediction, info);

  EXPECT_TRUE(info.success);
  EXPECT_FALSE(mpc->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

TEST(StageContracts, SwingLegControllerCallOrder) {
  std::unique_ptr<SwingLegControllerInterface> slc = std::make_unique<FakeSwingLegController>();
  const BrickState state = MakeState();
  const BrickModel model = MakeModel();

  slc->UpdateGaitSequence(GaitSequence{});
  slc->UpdateState(state);
  slc->UpdateModel(model);
  FeetTargets targets{};
  slc->GetFeetTargets(targets);
  std::array<double, N_LEGS> progress{};
  std::array<SwingLegControllerInterface::LegState, N_LEGS> swing_states{};
  slc->GetProgress(progress, swing_states);
  std::array<Eigen::Vector3d, N_LEGS> start_pos{};
  std::array<Eigen::Vector3d, N_LEGS> end_pos{};
  slc->GetCurrentTrajs(start_pos, end_pos);

  EXPECT_EQ(swing_states[0], SwingLegControllerInterface::LegState::STANCE);
  EXPECT_FALSE(slc->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

// One templated body covering every joint command type; the WBC is the only
// stage keyed on its command type (issue #13).
template <class JointCommandType>
void ExerciseWbc() {
  std::unique_ptr<WBCInterface<JointCommandType>> wbc = std::make_unique<FakeWBC<JointCommandType>>();
  const BrickState state = MakeState();
  const BrickModel model = MakeModel();

  wbc->UpdateState(state);
  wbc->UpdateModel(model);
  wbc->UpdateFeetTarget(FeetTargets{});
  wbc->UpdateWrenches(typename WBCInterface<JointCommandType>::Wrenches{});
  wbc->UpdateFootContact(typename WBCInterface<JointCommandType>::FootContact{});
  wbc->UpdateTarget(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
  JointCommandType command{};
  const WBCReturn ret = wbc->GetJointCommand(command);

  EXPECT_TRUE(ret.success);
  EXPECT_FALSE(wbc->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

TEST(StageContracts, WbcTorqueVelocityPositionCallOrder) { ExerciseWbc<JointTorqueVelocityPositionCommands>(); }
TEST(StageContracts, WbcTorqueCallOrder) { ExerciseWbc<JointTorqueCommands>(); }
TEST(StageContracts, WbcCartesianCallOrder) { ExerciseWbc<CartesianCommands>(); }

TEST(StageContracts, ModelAdaptationCallOrder) {
  std::unique_ptr<ModelAdaptationInterface> ma = std::make_unique<FakeModelAdaptation>();
  BrickState state = MakeState();
  BrickModel model = MakeModel();

  ma->UpdateState(state);
  ma->UpdateGaitSequence(GaitSequence{});
  const bool adapted = ma->DoModelAdaptation(model);

  EXPECT_FALSE(adapted);  // fake never adapts, so the host would not broadcast
  EXPECT_EQ(ma->GetParameterVector(), (Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS>::Zero()));
  EXPECT_FALSE(ma->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

TEST(StageContracts, ContactLogicCallOrderAndPassthrough) {
  std::unique_ptr<ContactLogicInterface> cl = std::make_unique<FakeContactLogic>();
  const BrickState state = MakeState();
  const BrickModel model = MakeModel();

  cl->UpdateState(state);
  cl->UpdateGaitSequence(GaitSequence{});
  cl->UpdateWrenchSequence(WrenchSequence{});
  cl->UpdateSwingLegState(FeetTargets{}, std::array<double, N_LEGS>{},
                          std::array<SwingLegControllerInterface::LegState, N_LEGS>{});
  cl->UpdateModel(model);

  // The host seeds the in/out params, then Reconcile edits them in place. The
  // fake is the identity, so seeded values must survive unchanged.
  ContactLogicInterface::FootContacts contacts{};
  contacts.fill(true);
  ContactLogicInterface::Wrenches wrenches{};
  wrenches[0] = Eigen::Vector3d(1.0, 2.0, 3.0);
  FeetTargets targets{};
  cl->Reconcile(contacts, wrenches, targets);
  EXPECT_TRUE(contacts[0]);
  EXPECT_EQ(wrenches[0], Eigen::Vector3d(1.0, 2.0, 3.0));

  std::array<ContactLogicInterface::LegContactState, N_LEGS> states{};
  cl->GetLegContactStates(states);
  ContactLogicInterface::ContactEvents events{};
  cl->GetContactEvents(events);
  EXPECT_EQ(states[0], ContactLogicInterface::LegContactState::SWING);
  EXPECT_FALSE(events.early_contact_detected[0]);
  EXPECT_FALSE(cl->SetParameter("contract_test.unknown", rclcpp::ParameterValue(0.0)));
}

TEST(StageContracts, GaitSeam) {
  std::unique_ptr<GaitInterface> gait = std::make_unique<FakeGait>();
  EXPECT_EQ(gait->get_t_stance(0), 0.0);
}

// -----------------------------------------------------------------------------
// Plugin lifecycle (doc/modularity/plugin_lifecycle.md): create → Init → run →
// destroy, with clone ownership handed over in Init and the fail-fast error
// path the M2.2 loader must surface.
// -----------------------------------------------------------------------------

TEST(StagePluginLifecycle, InitHandsOverOwnershipThenRuns) {
  auto plugin = std::make_unique<FakePluginGaitSequencer>();  // as the loader creates it

  StageInit init;
  init.model = std::make_unique<BrickModel>(MakeModel());
  init.state = std::make_unique<BrickState>();
  init.params.emplace(FakePluginGaitSequencer::kRequiredKey, rclcpp::ParameterValue(true));
  plugin->Init(std::move(init));
  EXPECT_TRUE(plugin->Initialized());

  // After Init the host owns and drives the stage through the frozen interface.
  std::unique_ptr<GaitSequencerInterface> gs = std::move(plugin);
  gs->UpdateTarget(Target{});
  GaitSequence sequence{};
  gs->GetGaitSequence(sequence);
}

TEST(StagePluginLifecycle, MissingRequiredParameterFailsInit) {
  FakePluginGaitSequencer plugin;
  EXPECT_THROW(plugin.Init(StageInit{}), StageInitError);
}

TEST(StagePluginLifecycle, RequireNamesTheMissingKey) {
  const StageInit init{};
  try {
    init.Require("contract_test.absent");
    FAIL() << "Require must throw for an absent key";
  } catch (const StageInitError& error) {
    EXPECT_NE(std::string(error.what()).find("contract_test.absent"), std::string::npos);
  }
}

}  // namespace
