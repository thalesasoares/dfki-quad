#include "mit_controller/default_contact_logic.hpp"

#include <Eigen/Geometry>
#include <cassert>
#include <utility>

#include "mit_controller/pipeline_constants.hpp"

DefaultContactLogic::DefaultContactLogic(bool early_contact_detection,
                                         bool late_contact_detection,
                                         bool lost_contact_detection,
                                         bool late_contact_reschedule_swing_phase,
                                         std::unique_ptr<ModelInterface> quad_model,
                                         std::unique_ptr<StateInterface> quad_state)
    : early_contact_detection_(early_contact_detection),
      late_contact_detection_(late_contact_detection),
      lost_contact_detection_(lost_contact_detection),
      late_contact_reschedule_swing_phase_(late_contact_reschedule_swing_phase),
      gait_sequence_(),
      wrench_sequence_(),
      swing_progress_(),
      swing_states_(),
      early_contact_hold_position_(),
      slip_hold_in_body_(),
      last_feet_pos_targets_(),
      last_events_(),
      quad_model_(std::move(quad_model)),
      quad_state_(std::move(quad_state)) {
  // Start with the current stance, exactly as the host did before this stage
  // existed.
  feet_status_.fill(LegContactState::STANCE);
  // The three hold-position arrays are value-initialised above rather than left
  // to the first write. As host members they were raw `Eigen::Vector3d` arrays,
  // and the SWING branch of the apply loop can read `last_feet_pos_targets_` on
  // the very first cycle — before any STANCE cycle wrote it — if the plan already
  // schedules a swing while the SLC has not started. Zero is a defined value
  // instead of whatever was on the stack; on the stock bring-up path (all four
  // feet scheduled in stance) neither version is ever read.
}

void DefaultContactLogic::UpdateState(const StateInterface &quad_state) { *quad_state_ = quad_state; }

void DefaultContactLogic::UpdateGaitSequence(const GaitSequence &gait_sequence) { gait_sequence_ = gait_sequence; }

void DefaultContactLogic::UpdateWrenchSequence(const WrenchSequence &wrench_sequence) {
  wrench_sequence_ = wrench_sequence;
}

void DefaultContactLogic::UpdateSwingLegState(
    const FeetTargets &feet_targets,
    const std::array<double, N_LEGS> &swing_progress,
    const std::array<SwingLegControllerInterface::LegState, N_LEGS> &swing_states) {
  (void)feet_targets;  // see the member comment: Reconcile receives these as its in/out argument
  swing_progress_ = swing_progress;
  swing_states_ = swing_states;
}

void DefaultContactLogic::UpdateModel(const ModelInterface &quad_model) { *quad_model_ = quad_model; }

void DefaultContactLogic::Reconcile(FootContacts &contacts, Wrenches &wrenches, FeetTargets &feet_targets) {
  // `contacts` is the gait sequencer's contact_sequence[0] as the host seeded it,
  // and is what the two loops below call `gait`: read as the *plan*, written where
  // the FSM overrides it.
  last_events_ = ContactEvents{};

  // Update leg status
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    switch (feet_status_[leg_idx]) {
      case LegContactState::LOST_CONTACT:
        [[fallthrough]];
      case LegContactState::LATE_CONTACT:
        // Leave it in slipped and override anything until it gets contact again
        // or do the late contact movement until it gets contact again and ignore any planned stance
        if (quad_state_->GetFeetContacts()[leg_idx] and contacts[leg_idx]) {
          // Regaining contact and schedule stance phase
          feet_status_[leg_idx] = LegContactState::STANCE;
          last_events_.contact_regained[leg_idx] = true;
        } else if (quad_state_->GetFeetContacts()[leg_idx] and !contacts[leg_idx]) {
          // Regaining contact and scheduled flight phase
          feet_status_[leg_idx] = LegContactState::SWING;
          last_events_.contact_regained[leg_idx] = true;
        } else if (late_contact_reschedule_swing_phase_ and !contacts[leg_idx])
        // Special feature to reschedule swing phase even if late contact (sometimes
        // legs are not properly touching the ground)
        {
          feet_status_[leg_idx] = LegContactState::SWING;
          // Reported as a regain because `contact_regained` is defined as the
          // transition out of LATE/LOST_CONTACT (contact_logic_interface.hpp).
          // This is the one path that takes it without a sensed contact.
          last_events_.contact_regained[leg_idx] = true;
        }
        // Otherwise stay in that phase
        break;
      case LegContactState::EARLY_CONTACT:
        // Wait for scheduling to react to early contact
        if (contacts[leg_idx]) {
          feet_status_[leg_idx] = LegContactState::STANCE;
        }
        break;
      case LegContactState::SWING:
        // Check for early or late contact:
        if (early_contact_detection_ and !contacts[leg_idx] and quad_state_->GetFeetContacts()[leg_idx]
            and swing_progress_[leg_idx] > 0.5) {  // Early contact
          feet_status_[leg_idx] = LegContactState::EARLY_CONTACT;
          early_contact_hold_position_[leg_idx] = quad_model_->CalcFootPositionInWorld(leg_idx, *quad_state_);
          last_events_.early_contact_detected[leg_idx] = true;
        } else if (late_contact_detection_ and contacts[leg_idx] and !quad_state_->GetFeetContacts()[leg_idx]) {
          feet_status_[leg_idx] = LegContactState::LATE_CONTACT;
          slip_hold_in_body_[leg_idx] = quad_model_->CalcFootPositionInBodyFrame(
              leg_idx, Eigen::Map<const Eigen::Vector3d>(quad_state_->GetJointPositions()[leg_idx].data()));
          last_events_.late_contact_detected[leg_idx] = true;
        } else if (contacts[leg_idx]) {
          feet_status_[leg_idx] = LegContactState::STANCE;  // rare case but this is the ideal one
        }
        break;
      case LegContactState::STANCE:
        // Check if flight phase scheduled or slip detected
        if (!contacts[leg_idx]) {
          feet_status_[leg_idx] = LegContactState::SWING;  // Swing scheduled so going to swing phase
        } else if (lost_contact_detection_ and contacts[leg_idx] and !quad_state_->GetFeetContacts()[leg_idx]) {
          feet_status_[leg_idx] = LegContactState::LOST_CONTACT;  // Slip detected going to slip
          slip_hold_in_body_[leg_idx] = quad_model_->CalcFootPositionInBodyFrame(
              leg_idx, Eigen::Map<const Eigen::Vector3d>(quad_state_->GetJointPositions()[leg_idx].data()));
          last_events_.lost_contact_detected[leg_idx] = true;
        }
        break;
    }
  }

  // Apply leg commands
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    switch (feet_status_[leg_idx]) {
      case LegContactState::STANCE: {
        last_feet_pos_targets_[leg_idx] = gait_sequence_.foot_position_sequence[0][leg_idx];
        feet_targets.positions[leg_idx] = gait_sequence_.foot_position_sequence[0][leg_idx];
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
      } break;
      case LegContactState::SWING: {
        wrenches[leg_idx].setZero();
        // feet targets stay the same
        if ((swing_states_[leg_idx] == SwingLegControllerInterface::STANCE)
            or (swing_states_[leg_idx] == SwingLegControllerInterface::NOT_STARTED)) {
          // take last feet targets for now
          feet_targets.velocities[leg_idx].setZero();
          feet_targets.accelerations[leg_idx].setZero();
          feet_targets.positions[leg_idx] = last_feet_pos_targets_[leg_idx];
          last_events_.swing_scheduled_before_slc_started[leg_idx] = true;
        }
      } break;
      case LegContactState::EARLY_CONTACT: {
        contacts[leg_idx] = true;
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
        feet_targets.positions[leg_idx] = early_contact_hold_position_[leg_idx];
        // Next stance phase targets have to be transformed to current pose
        int next_stance_indx = int(gait_sequence_.swing_time_sequence[0][leg_idx] / MPC_DT) + 1;
        assert(gait_sequence_.contact_sequence[next_stance_indx][leg_idx] == true);
        wrenches[leg_idx] = quad_state_->GetOrientationInWorld()
                            * gait_sequence_.reference_trajectory_orientation[next_stance_indx].inverse()
                            * wrench_sequence_.forces[next_stance_indx][leg_idx];
      } break;
      case LegContactState::LOST_CONTACT: {
        // Keep foot on lost position
        // same behaviour as LATE_CONTACT for now
        [[fallthrough]];
      }
      case LegContactState::LATE_CONTACT: {
        contacts[leg_idx] = false;
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
        // The host used to transform this with its *live* `quad_state_` member —
        // an unsynchronised read from the control loop — while everything around
        // it used the cycle's snapshot. The stage has only the snapshot, which is
        // both the intended datum and one race less.
        feet_targets.positions[leg_idx] = Eigen::Translation3d(quad_state_->GetPositionInWorld())
                                          * quad_state_->GetOrientationInWorld()
                                          * slip_hold_in_body_[leg_idx];  // Transformed to world
        wrenches[leg_idx].setZero();
      } break;
    }
  }
}

void DefaultContactLogic::GetLegContactStates(std::array<LegContactState, N_LEGS> &states) const {
  states = feet_status_;
}

void DefaultContactLogic::GetContactEvents(ContactEvents &events) const { events = last_events_; }

bool DefaultContactLogic::SetParameter(const std::string &name, const rclcpp::ParameterValue &value) {
  if (name == contact_logic_params::kEarlyContactDetection) {
    early_contact_detection_ = value.get<bool>();
  } else if (name == contact_logic_params::kLateContactDetection) {
    late_contact_detection_ = value.get<bool>();
  } else if (name == contact_logic_params::kLostContactDetection) {
    lost_contact_detection_ = value.get<bool>();
  } else if (name == contact_logic_params::kLateContactRescheduleSwingPhase) {
    late_contact_reschedule_swing_phase_ = value.get<bool>();
  } else {
    return false;
  }
  return true;
}
