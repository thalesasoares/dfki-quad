// Stock contact logic behaviour test (issue #12, M3.1).
//
// The extraction of the contact FSM out of MITController::ControlLoopCallback is
// only worth anything if the policy survived it unchanged, and nothing pinned
// that policy before: it was a pair of switch statements inside a 500 Hz control
// loop, reachable only by running a robot. This test is that pin. It drives
// DefaultContactLogic through ContactLogicInterface alone — the same call
// sequence the host makes, in the same order — and asserts the reconciled WBC
// inputs, the per-leg state and the transition events for every branch of both
// switches.
//
// The fixture below mirrors the host's control loop deliberately (see Cycle()):
// seed contacts/wrenches/targets from the plan and the SLC, Reconcile in place,
// read the events. If the host ever stops doing exactly this, the difference is
// visible here as a diff, not as a walking robot behaving oddly.
//
// Model/state doubles come from potato_sim. BrickModel::CalcFootPositionInWorld
// returns BrickState::virt_feet_positions_, which is what the early-contact hold
// records; CalcFootPositionInBodyFrame throws there ("doesnt work" for a brick),
// so TestModel overrides it with a settable value — that is the datum the
// late/lost hold records.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>

#include "mit_controller/default_contact_logic.hpp"
#include "mit_controller/feet_targets.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wrench_sequence.hpp"
#include "potato_sim/potato_model.hpp"

namespace {

using LegContactState = ContactLogicInterface::LegContactState;
using LegState = SwingLegControllerInterface::LegState;

constexpr unsigned int kLeg = 0;  // the leg every scenario drives; the others stay in stance

// Eigen's operator== is coefficient-wise, so it cannot feed EXPECT_EQ. These two
// keep the assertions readable and print both vectors on failure.
::testing::AssertionResult Equals(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected) {
  if ((actual.array() == expected.array()).all()) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "got [" << actual.transpose() << "], expected [" << expected.transpose()
                                       << "]";
}

::testing::AssertionResult Approx(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected) {
  if (actual.isApprox(expected)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "got [" << actual.transpose() << "], expected [" << expected.transpose()
                                       << "]";
}

// BrickModel plus the two things the FSM asks a model for that a brick cannot
// answer: a body-frame foot position, and an assignable model (BrickModel does
// not override ModelInterface::operator=, so UpdateModel would be a silent no-op).
class TestModel : public BrickModel {
 public:
  TestModel() : BrickModel(Eigen::Matrix3d::Identity(), 1.0) {}

  std::array<Eigen::Vector3d, N_LEGS> body_frame_positions_{};

  Eigen::Vector3d CalcFootPositionInBodyFrame(unsigned int foot_idx,
                                              const Eigen::Vector3d &joint_positions) const override {
    (void)joint_positions;  // the brick has no joints; GetJointPositions() is a dummy
    return body_frame_positions_[foot_idx];
  }

  ModelInterface &operator=(const ModelInterface &other) override {
    body_frame_positions_ = dynamic_cast<const TestModel &>(other).body_frame_positions_;
    return *this;
  }
};

// One control cycle's worth of pipeline inputs plus the stage under test, wired
// the way MITController::ControlLoopCallback wires them.
struct Harness {
  BrickState state;
  GaitSequence gait_sequence{};
  WrenchSequence wrench_sequence{};
  FeetTargets slc_targets{};
  std::array<double, N_LEGS> swing_progress{};
  std::array<LegState, N_LEGS> swing_states{};

  std::unique_ptr<DefaultContactLogic> logic;

  // Reconciled outputs of the last Cycle().
  ContactLogicInterface::FootContacts contacts{};
  ContactLogicInterface::Wrenches wrenches{};
  FeetTargets targets{};
  ContactLogicInterface::ContactEvents events{};
  std::array<LegContactState, N_LEGS> states{};

  Harness(bool early, bool late, bool lost, bool reschedule) {
    // Standing, all four feet sensing and scheduled in contact.
    state.position_ = Eigen::Vector3d::Zero();
    state.orientation_ = Eigen::Quaterniond::Identity();
    state.feet_contacts_.fill(true);
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      state.virt_feet_positions_[leg] = Eigen::Vector3d(0.1 * leg, 0.0, 0.0);
    }
    for (auto &step : gait_sequence.contact_sequence) {
      step.fill(true);
    }
    for (auto &step : gait_sequence.foot_position_sequence) {
      for (unsigned int leg = 0; leg < N_LEGS; leg++) {
        step[leg] = Eigen::Vector3d(1.0 + leg, 2.0, 3.0);
      }
    }
    for (auto &orientation : gait_sequence.reference_trajectory_orientation) {
      orientation = Eigen::Quaterniond::Identity();
    }
    for (auto &step : gait_sequence.swing_time_sequence) {
      step.fill(0.0);
    }
    for (auto &step : wrench_sequence.forces) {
      for (unsigned int leg = 0; leg < N_LEGS; leg++) {
        step[leg] = Eigen::Vector3d(0.0, 0.0, 100.0);
      }
    }
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      slc_targets.positions[leg] = Eigen::Vector3d(9.0 + leg, 9.0, 9.0);
      slc_targets.velocities[leg] = Eigen::Vector3d(1.0, 1.0, 1.0);
      slc_targets.accelerations[leg] = Eigen::Vector3d(2.0, 2.0, 2.0);
    }
    swing_states.fill(LegState::STANCE);

    auto model = std::make_unique<TestModel>();
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      model->body_frame_positions_[leg] = Eigen::Vector3d(0.0, 0.0, -0.3 - 0.01 * leg);
    }
    logic = std::make_unique<DefaultContactLogic>(
        early, late, lost, reschedule, std::move(model), std::make_unique<BrickState>(state));
  }

  // Exactly the host's control-loop sequence: feed the stage, seed the in/out
  // triple from the plan / MPC / SLC, reconcile once, read back.
  void Cycle() {
    logic->UpdateState(state);
    logic->UpdateGaitSequence(gait_sequence);
    logic->UpdateWrenchSequence(wrench_sequence);
    logic->UpdateSwingLegState(slc_targets, swing_progress, swing_states);

    contacts = gait_sequence.contact_sequence[0];
    wrenches = wrench_sequence.forces[0];
    targets = slc_targets;
    logic->Reconcile(contacts, wrenches, targets);

    logic->GetContactEvents(events);
    logic->GetLegContactStates(states);
  }

  // Schedules `leg` to swing over the given steps. The FSM only reads step 0, but
  // the early-contact wrench substitution indexes further ahead.
  void ScheduleSwing(unsigned int leg, std::size_t first_step, std::size_t last_step) {
    for (std::size_t step = first_step; step <= last_step; step++) {
      gait_sequence.contact_sequence[step][leg] = false;
    }
  }

  // Runs the leg into a settled SWING: the FSM starts every leg in STANCE, so the
  // first cycle of a scheduled swing only performs the STANCE -> SWING transition.
  // The early/late checks live in the SWING case and are reached from the *next*
  // cycle on — which is exactly how they behave on the robot.
  void EnterSwing(unsigned int leg, std::size_t last_swing_step) {
    ScheduleSwing(leg, 0, last_swing_step);
    state.feet_contacts_[leg] = false;
    swing_states[leg] = LegState::SWINGING;
    Cycle();
  }
};

Harness AllDetectionsOn() { return Harness(true, true, true, false); }
Harness AllDetectionsOff() { return Harness(false, false, false, false); }

void ExpectNoEvents(const ContactLogicInterface::ContactEvents &events, unsigned int leg) {
  EXPECT_FALSE(events.early_contact_detected[leg]);
  EXPECT_FALSE(events.late_contact_detected[leg]);
  EXPECT_FALSE(events.lost_contact_detected[leg]);
  EXPECT_FALSE(events.contact_regained[leg]);
}

// -----------------------------------------------------------------------------
// The two undisturbed cases: stance and swing with a running SLC.
// -----------------------------------------------------------------------------

// Stance overrides the SLC target with the planned foothold and holds it still,
// and forwards the MPC's first-step wrench untouched.
TEST(DefaultContactLogic, StancePassesThroughThePlan) {
  Harness h = AllDetectionsOff();
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::STANCE);
  EXPECT_TRUE(h.contacts[kLeg]);
  EXPECT_TRUE(Equals(h.wrenches[kLeg], h.wrench_sequence.forces[0][kLeg]));
  EXPECT_TRUE(Equals(h.targets.positions[kLeg], h.gait_sequence.foot_position_sequence[0][kLeg]));
  EXPECT_TRUE(h.targets.velocities[kLeg].isZero());
  EXPECT_TRUE(h.targets.accelerations[kLeg].isZero());
  ExpectNoEvents(h.events, kLeg);
  EXPECT_FALSE(h.events.swing_scheduled_before_slc_started[kLeg]);
}

// Swing zeroes the wrench and leaves the SLC's trajectory alone.
TEST(DefaultContactLogic, SwingZeroesWrenchAndKeepsSlcTargets) {
  Harness h = AllDetectionsOff();
  h.ScheduleSwing(kLeg, 0, 4);
  h.state.feet_contacts_[kLeg] = false;
  h.swing_states[kLeg] = LegState::SWINGING;
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::SWING);
  EXPECT_FALSE(h.contacts[kLeg]);
  EXPECT_TRUE(h.wrenches[kLeg].isZero());
  EXPECT_TRUE(Equals(h.targets.positions[kLeg], h.slc_targets.positions[kLeg]));
  EXPECT_TRUE(Equals(h.targets.velocities[kLeg], h.slc_targets.velocities[kLeg]));
  EXPECT_TRUE(Equals(h.targets.accelerations[kLeg], h.slc_targets.accelerations[kLeg]));
  ExpectNoEvents(h.events, kLeg);
  EXPECT_FALSE(h.events.swing_scheduled_before_slc_started[kLeg]);
}

// The control loop reaching a swing the SLC has not started falls back to the
// last stance target, and reports it so the host can log it. Both SLC states that
// mean "not swinging yet" take the branch.
TEST(DefaultContactLogic, SwingBeforeSlcStartedHoldsLastStanceTarget) {
  for (const LegState slc_state : {LegState::NOT_STARTED, LegState::STANCE}) {
    Harness h = AllDetectionsOff();
    h.Cycle();  // one stance cycle, so last_feet_pos_targets_ is the planned foothold
    const Eigen::Vector3d last_stance_target = h.gait_sequence.foot_position_sequence[0][kLeg];

    h.ScheduleSwing(kLeg, 0, 4);
    h.state.feet_contacts_[kLeg] = false;
    h.swing_states[kLeg] = slc_state;
    h.Cycle();

    EXPECT_EQ(h.states[kLeg], LegContactState::SWING);
    EXPECT_TRUE(h.events.swing_scheduled_before_slc_started[kLeg]) << "slc state " << slc_state;
    EXPECT_TRUE(Equals(h.targets.positions[kLeg], last_stance_target));
    EXPECT_TRUE(h.targets.velocities[kLeg].isZero());
    EXPECT_TRUE(h.targets.accelerations[kLeg].isZero());
  }
}

// -----------------------------------------------------------------------------
// Early contact.
// -----------------------------------------------------------------------------

// Past half the swing, a sensed contact is accepted: the leg is forced back into
// stance, held where it touched down, and loaded with the wrench planned for its
// next stance step, rotated from the plan's reference orientation into the
// current one.
TEST(DefaultContactLogic, EarlyContactHoldsTouchdownAndLoadsNextStanceWrench) {
  Harness h = AllDetectionsOn();
  h.gait_sequence.swing_time_sequence[0][kLeg] = 2.0 * MPC_DT;  // -> next_stance_indx == 3
  h.wrench_sequence.forces[3][kLeg] = Eigen::Vector3d(7.0, 8.0, 9.0);
  // A non-trivial rotation on both sides, so the transform cannot pass by accident.
  const Eigen::Quaterniond body(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond plan(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  h.state.orientation_ = body;
  h.gait_sequence.reference_trajectory_orientation[3] = plan;
  h.EnterSwing(kLeg, 2);  // back in contact from step 3

  h.state.feet_contacts_[kLeg] = true;
  h.swing_progress[kLeg] = 0.6;
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::EARLY_CONTACT);
  EXPECT_TRUE(h.events.early_contact_detected[kLeg]);
  EXPECT_TRUE(h.contacts[kLeg]) << "an early contact must be loaded as a stance by the WBC";
  EXPECT_TRUE(Equals(h.targets.positions[kLeg], h.state.virt_feet_positions_[kLeg]));
  EXPECT_TRUE(h.targets.velocities[kLeg].isZero());
  EXPECT_TRUE(h.targets.accelerations[kLeg].isZero());
  const Eigen::Vector3d expected = body * plan.inverse() * Eigen::Vector3d(7.0, 8.0, 9.0);
  EXPECT_TRUE(Approx(h.wrenches[kLeg], expected));
}

// Below half the swing the same sensed contact is ignored — that threshold is the
// whole defence against a foot brushing an obstacle mid-flight.
TEST(DefaultContactLogic, EarlyContactIgnoredBeforeHalfTheSwing) {
  Harness h = AllDetectionsOn();
  h.EnterSwing(kLeg, 4);

  h.state.feet_contacts_[kLeg] = true;
  h.swing_progress[kLeg] = 0.5;  // the condition is strictly greater
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::SWING);
  EXPECT_FALSE(h.events.early_contact_detected[kLeg]);
  EXPECT_FALSE(h.contacts[kLeg]);
  EXPECT_TRUE(h.wrenches[kLeg].isZero());
}

// The leg leaves EARLY_CONTACT only when the plan catches up with it.
TEST(DefaultContactLogic, EarlyContactWaitsForThePlanToScheduleStance) {
  Harness h = AllDetectionsOn();
  h.gait_sequence.swing_time_sequence[0][kLeg] = 2.0 * MPC_DT;
  h.EnterSwing(kLeg, 2);

  h.state.feet_contacts_[kLeg] = true;
  h.swing_progress[kLeg] = 0.9;
  h.Cycle();
  ASSERT_EQ(h.states[kLeg], LegContactState::EARLY_CONTACT);

  h.Cycle();  // plan still says swing
  EXPECT_EQ(h.states[kLeg], LegContactState::EARLY_CONTACT);
  EXPECT_FALSE(h.events.early_contact_detected[kLeg]) << "the event marks the transition cycle only";

  h.gait_sequence.contact_sequence[0][kLeg] = true;  // plan schedules stance
  h.Cycle();
  EXPECT_EQ(h.states[kLeg], LegContactState::STANCE);
  EXPECT_TRUE(Equals(h.targets.positions[kLeg], h.gait_sequence.foot_position_sequence[0][kLeg]));
  EXPECT_TRUE(Equals(h.wrenches[kLeg], h.wrench_sequence.forces[0][kLeg]));
}

// -----------------------------------------------------------------------------
// Late and lost contact.
// -----------------------------------------------------------------------------

// A scheduled stance that never touches down is unloaded and pinned to the body,
// so the leg keeps its geometry while the body moves under it.
TEST(DefaultContactLogic, LateContactUnloadsAndHoldsInBodyFrame) {
  Harness h = AllDetectionsOn();
  h.EnterSwing(kLeg, 0);
  ASSERT_EQ(h.states[kLeg], LegContactState::SWING);

  h.gait_sequence.contact_sequence[0][kLeg] = true;  // plan wants stance, foot still in the air
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::LATE_CONTACT);
  EXPECT_TRUE(h.events.late_contact_detected[kLeg]);
  EXPECT_FALSE(h.contacts[kLeg]) << "a foot that is not on the ground must not be loaded";
  EXPECT_TRUE(h.wrenches[kLeg].isZero());
  EXPECT_TRUE(h.targets.velocities[kLeg].isZero());
  EXPECT_TRUE(h.targets.accelerations[kLeg].isZero());

  // The hold is recorded in the body frame and republished in world every cycle:
  // moving and turning the body must move the target with it.
  h.state.position_ = Eigen::Vector3d(1.0, 2.0, 0.5);
  h.state.orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  h.Cycle();
  const Eigen::Vector3d expected = Eigen::Translation3d(h.state.position_) * h.state.orientation_
                                   * Eigen::Vector3d(0.0, 0.0, -0.3 - 0.01 * kLeg);
  EXPECT_EQ(h.states[kLeg], LegContactState::LATE_CONTACT);
  EXPECT_TRUE(Approx(h.targets.positions[kLeg], expected));
}

// Losing a stance contact is the same policy, entered from the other side, and is
// the one transition the host logs as a warning.
TEST(DefaultContactLogic, LostContactHoldsLikeLateContact) {
  Harness h = AllDetectionsOn();
  h.Cycle();
  ASSERT_EQ(h.states[kLeg], LegContactState::STANCE);

  h.state.feet_contacts_[kLeg] = false;  // still scheduled stance
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::LOST_CONTACT);
  EXPECT_TRUE(h.events.lost_contact_detected[kLeg]);
  EXPECT_FALSE(h.contacts[kLeg]);
  EXPECT_TRUE(h.wrenches[kLeg].isZero());
  const Eigen::Vector3d expected = Eigen::Translation3d(h.state.position_) * h.state.orientation_
                                   * Eigen::Vector3d(0.0, 0.0, -0.3 - 0.01 * kLeg);
  EXPECT_TRUE(Approx(h.targets.positions[kLeg], expected));
}

// Regaining contact hands the leg back to whichever phase the plan is in.
TEST(DefaultContactLogic, RegainedContactFollowsThePlan) {
  for (const bool plan_says_stance : {true, false}) {
    Harness h = AllDetectionsOn();
    h.Cycle();
    h.state.feet_contacts_[kLeg] = false;
    h.Cycle();
    ASSERT_EQ(h.states[kLeg], LegContactState::LOST_CONTACT);

    h.state.feet_contacts_[kLeg] = true;
    h.gait_sequence.contact_sequence[0][kLeg] = plan_says_stance;
    h.Cycle();

    EXPECT_EQ(h.states[kLeg], plan_says_stance ? LegContactState::STANCE : LegContactState::SWING);
    EXPECT_TRUE(h.events.contact_regained[kLeg]);
  }
}

// Without the reschedule feature a late-contact leg that never touches down stays
// held even once the plan has moved on to swing; with it, the plan wins.
TEST(DefaultContactLogic, LateContactRescheduleSwingPhase) {
  for (const bool reschedule : {false, true}) {
    Harness h = Harness(true, true, true, reschedule);
    h.EnterSwing(kLeg, 0);
    h.gait_sequence.contact_sequence[0][kLeg] = true;
    h.Cycle();
    ASSERT_EQ(h.states[kLeg], LegContactState::LATE_CONTACT);

    h.gait_sequence.contact_sequence[0][kLeg] = false;  // plan schedules swing, still no contact
    h.Cycle();

    EXPECT_EQ(h.states[kLeg], reschedule ? LegContactState::SWING : LegContactState::LATE_CONTACT)
        << "reschedule " << reschedule;
  }
}

// -----------------------------------------------------------------------------
// The toggles: each one off means its transition simply never happens.
// -----------------------------------------------------------------------------

TEST(DefaultContactLogic, EarlyContactDetectionOffKeepsTheLegSwinging) {
  Harness h = Harness(false, true, true, false);
  h.EnterSwing(kLeg, 4);

  h.state.feet_contacts_[kLeg] = true;
  h.swing_progress[kLeg] = 0.9;
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::SWING);
  EXPECT_FALSE(h.events.early_contact_detected[kLeg]);
  EXPECT_FALSE(h.contacts[kLeg]) << "the plan still says swing, and nothing overrode it";
}

TEST(DefaultContactLogic, LateContactDetectionOffTakesThePlannedStance) {
  Harness h = Harness(true, false, true, false);
  h.EnterSwing(kLeg, 0);
  h.gait_sequence.contact_sequence[0][kLeg] = true;
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::STANCE);
  EXPECT_FALSE(h.events.late_contact_detected[kLeg]);
  EXPECT_TRUE(h.contacts[kLeg]);
}

TEST(DefaultContactLogic, LostContactDetectionOffKeepsLoadingTheFoot) {
  Harness h = Harness(true, true, false, false);
  h.Cycle();
  h.state.feet_contacts_[kLeg] = false;
  h.Cycle();

  EXPECT_EQ(h.states[kLeg], LegContactState::STANCE);
  EXPECT_FALSE(h.events.lost_contact_detected[kLeg]);
  EXPECT_TRUE(h.contacts[kLeg]);
  EXPECT_TRUE(Equals(h.wrenches[kLeg], h.wrench_sequence.forces[0][kLeg]));
}

// -----------------------------------------------------------------------------
// Cross-cutting: per-leg independence, model updates, parameters.
// -----------------------------------------------------------------------------

// The FSM is per leg; one leg misbehaving must not disturb the other three.
TEST(DefaultContactLogic, LegsAreIndependent) {
  Harness h = AllDetectionsOn();
  h.Cycle();
  h.state.feet_contacts_[2] = false;
  h.Cycle();

  EXPECT_EQ(h.states[2], LegContactState::LOST_CONTACT);
  for (unsigned int leg : {0u, 1u, 3u}) {
    EXPECT_EQ(h.states[leg], LegContactState::STANCE) << "leg " << leg;
    EXPECT_TRUE(h.contacts[leg]);
    EXPECT_TRUE(Equals(h.targets.positions[leg], h.gait_sequence.foot_position_sequence[0][leg]));
    ExpectNoEvents(h.events, leg);
  }
}

// Events describe the cycle that just ran, not the state the leg is in: a leg
// sitting in LOST_CONTACT reports nothing until something changes.
TEST(DefaultContactLogic, EventsFireOnlyOnTheTransitionCycle) {
  Harness h = AllDetectionsOn();
  h.Cycle();
  h.state.feet_contacts_[kLeg] = false;
  h.Cycle();
  ASSERT_TRUE(h.events.lost_contact_detected[kLeg]);

  h.Cycle();
  EXPECT_EQ(h.states[kLeg], LegContactState::LOST_CONTACT);
  ExpectNoEvents(h.events, kLeg);
}

// The model the hold positions are computed from is swappable at runtime — that
// is what the host's model-adaptation broadcast needs.
TEST(DefaultContactLogic, UpdateModelChangesTheHoldKinematics) {
  Harness h = AllDetectionsOn();
  TestModel updated;
  updated.body_frame_positions_.fill(Eigen::Vector3d(0.0, 0.0, -0.9));
  h.logic->UpdateModel(updated);

  h.Cycle();
  h.state.feet_contacts_[kLeg] = false;
  h.Cycle();

  ASSERT_EQ(h.states[kLeg], LegContactState::LOST_CONTACT);
  const Eigen::Vector3d expected = Eigen::Translation3d(h.state.position_) * h.state.orientation_
                                   * Eigen::Vector3d(0.0, 0.0, -0.9);
  EXPECT_TRUE(Approx(h.targets.positions[kLeg], expected));
}

TEST(DefaultContactLogic, SetParameterTogglesDetectionAtRuntime) {
  Harness h = AllDetectionsOn();
  h.Cycle();

  EXPECT_TRUE(h.logic->SetParameter(contact_logic_params::kLostContactDetection, rclcpp::ParameterValue(false)));
  h.state.feet_contacts_[kLeg] = false;
  h.Cycle();
  EXPECT_EQ(h.states[kLeg], LegContactState::STANCE) << "lost contact detection was switched off";

  EXPECT_TRUE(h.logic->SetParameter(contact_logic_params::kLostContactDetection, rclcpp::ParameterValue(true)));
  h.Cycle();
  EXPECT_EQ(h.states[kLeg], LegContactState::LOST_CONTACT);
}

TEST(DefaultContactLogic, SetParameterAcceptsEveryRatifiedKey) {
  Harness h = AllDetectionsOn();
  for (const char *key : {contact_logic_params::kEarlyContactDetection,
                          contact_logic_params::kLateContactDetection,
                          contact_logic_params::kLostContactDetection,
                          contact_logic_params::kLateContactRescheduleSwingPhase}) {
    EXPECT_TRUE(h.logic->SetParameter(key, rclcpp::ParameterValue(true))) << key;
  }
}

// An unrecognised key is reported, not swallowed: the host turns the false into
// the "not yet suported" warning it prints for every other stage.
TEST(DefaultContactLogic, SetParameterRejectsUnknownKeys) {
  Harness h = AllDetectionsOn();
  EXPECT_FALSE(h.logic->SetParameter("early_contact_detection", rclcpp::ParameterValue(true)))
      << "the flat legacy spelling is the host's bridge to translate, not this stage's key";
  EXPECT_FALSE(h.logic->SetParameter("contact_logic.type", rclcpp::ParameterValue(std::string("x"))));
  EXPECT_FALSE(h.logic->SetParameter("slc_swing_height", rclcpp::ParameterValue(0.1)));
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
