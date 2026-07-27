#pragma once

#include <rclcpp/parameter_value.hpp>

#include <Eigen/Core>
#include <array>
#include <string>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "feet_targets.hpp"
#include "gait_sequence.hpp"
#include "mit_controller/pipeline_constants.hpp"
#include "swing_leg_controller_interface.hpp"
#include "wrench_sequence.hpp"

/**
 * Contract for the contact-reconciliation stage of the control pipeline.
 *
 * This is the sixth pipeline stage in everything but name (see
 * doc/modularity/stage_contracts.md §4.6 and §7). It reconciles the *planned*
 * contact schedule from the gait sequencer with the *sensed* foot contacts from
 * the state, running the per-leg early/late/lost-contact FSM and overriding the
 * WBC inputs — contact flags, wrenches and foot targets — accordingly.
 *
 * This header specifies the API (issue #4, M1.4). The implementation was
 * extracted out of `MITController::ControlLoopCallback` into the stock
 * `DefaultContactLogic` plugin in issue #12 (M3.1), which is also where the host
 * started loading and calling the stage like the other five.
 *
 * ## Placement in the pipeline
 * The host runs this stage inside the 500 Hz control loop, after it has copied
 * the latest `GaitSequence`, `WrenchSequence`, `MPCPrediction` and the swing
 * leg controller outputs, and before it feeds the reconciled results to the
 * `WBCInterface`. See the call-order and threading notes on `Reconcile`.
 */
/**
 * The runtime parameter keys of the contact stage.
 *
 * These are the four detection toggles the FSM reads. They are named here, on
 * the shared contract, rather than spelled as literals on either side: the host
 * declares them and routes `SetParameter` by them (mit_controller_node.cpp), the
 * stock plugin reads them in `Init` and recognises them in `SetParameter`
 * (src/plugins/contact_logic_plugins.cpp), and start-up configuration and runtime
 * reconfiguration are one vocabulary (plugin_lifecycle.md §3). Promoting them
 * from the prose of `SetParameter` below to constants in M3.1 (issue #12) is what
 * keeps the two sides from drifting.
 *
 * The host also still accepts the pre-M3.1 flat spelling (`early_contact_detection`
 * and friends) and bridges it onto these keys; that bridge is host-only and goes
 * away with #23 (M5.4). See `stage_selection.hpp`.
 */
namespace contact_logic_params {
inline constexpr char kEarlyContactDetection[] = "contact_logic.early_contact_detection";
inline constexpr char kLateContactDetection[] = "contact_logic.late_contact_detection";
inline constexpr char kLostContactDetection[] = "contact_logic.lost_contact_detection";
inline constexpr char kLateContactRescheduleSwingPhase[] = "contact_logic.late_contact_reschedule_swing_phase";
}  // namespace contact_logic_params

class ContactLogicInterface {
 protected:
  ContactLogicInterface() = default;  // protected, as there cant be any Object from an Interface

 public:
  /**
   * Per-leg reconciliation status. Replaces the private `MITController::LegStatus`
   * the extraction (issue #12, M3.1) removed from the host.
   *   - STANCE:        scheduled and sensed in contact.
   *   - SWING:         scheduled and sensed off the ground.
   *   - EARLY_CONTACT: sensed contact while still scheduled to swing; the foot is
   *                    held and loaded with the next stance wrench until the plan
   *                    schedules it back to stance.
   *   - LATE_CONTACT:  scheduled to stand but not yet sensing contact; held at the
   *                    last body-frame position, unloaded, until contact returns.
   *   - LOST_CONTACT:  lost a stance contact (slip); currently handled like
   *                    LATE_CONTACT.
   */
  enum class LegContactState { SWING, STANCE, EARLY_CONTACT, LATE_CONTACT, LOST_CONTACT };

  // These mirror `WBCInterface::FootContact` / `::Wrenches`, and the host passes
  // them straight through from this stage into the WBC. They stay restated rather
  // than reused so that no stage contract has to include a sibling stage's header;
  // `test_stage_contracts.cpp` `static_assert`s the two spellings are one type, so
  // the layouts cannot drift apart. (Before #13 they *could not* be reused:
  // `WBCInterface` was a class template, so naming its typedefs meant picking an
  // instantiation — gap G8.)
  using FootContacts = std::array<bool, N_LEGS>;
  using Wrenches = std::array<Eigen::Vector3d, N_LEGS>;

  /**
   * Per-cycle transition events, so the host can keep ROS logging and the
   * `ControllerInfo` heartbeat counters (e.g. `num_early_contacts`) without the
   * stage depending on a logger or the heartbeat message. Each array is indexed
   * by leg; an entry is true only on the cycle the transition happens.
   */
  struct ContactEvents {
    std::array<bool, N_LEGS> early_contact_detected;  ///< SWING -> EARLY_CONTACT this cycle
    std::array<bool, N_LEGS> late_contact_detected;   ///< SWING -> LATE_CONTACT this cycle
    std::array<bool, N_LEGS> lost_contact_detected;   ///< STANCE -> LOST_CONTACT this cycle
    std::array<bool, N_LEGS> contact_regained;        ///< LATE/LOST_CONTACT -> STANCE/SWING this cycle
    std::array<bool, N_LEGS> swing_scheduled_before_slc_started;  ///< control loop reached SWING while
                                                                  ///< the SLC is still STANCE/NOT_STARTED
  };

  /**
   * Provides the newest state, carrying the sensed foot contacts
   * (`GetFeetContacts`), joint positions and world pose the FSM triggers and
   * hold-position computations read. Called every control cycle (500 Hz), first
   * of the Update* methods, under the host's `wbc_lock_`.
   *
   * @param quad_state the new state
   */
  virtual void UpdateState(const StateInterface& quad_state) = 0;
  /**
   * Provides the newest gait sequence — the *planned* contact schedule. The
   * reconciliation reads the first step (`contact_sequence[0]`,
   * `foot_position_sequence[0]`) plus the future indices the early-contact wrench
   * substitution needs (`swing_time_sequence[0]`, and `contact_sequence`,
   * `reference_trajectory_orientation` at the next scheduled stance). The whole
   * sequence is passed for that reason. Called every control cycle (500 Hz).
   *
   * @param gait_sequence the planned contact schedule and foot placements
   */
  virtual void UpdateGaitSequence(const GaitSequence& gait_sequence) = 0;
  /**
   * Provides the newest MPC wrench sequence. The reconciliation forwards
   * `forces[0]` in stance and substitutes the next-stance wrench (rotated into
   * the current orientation) while a foot is in EARLY_CONTACT. The whole sequence
   * is passed so the implementation can index the future stance step. Called
   * every control cycle (500 Hz).
   *
   * @param wrench_sequence the MPC ground-reaction-force plan
   */
  virtual void UpdateWrenchSequence(const WrenchSequence& wrench_sequence) = 0;
  /**
   * Provides the swing leg controller outputs the FSM gates on: the current foot
   * targets, the per-leg swing progress (early contact is only accepted past
   * `progress > 0.5`) and the per-leg `SwingLegControllerInterface::LegState`
   * (used to fall back to the last target when the control loop reaches a swing
   * phase the SLC has not started). Called every control cycle (500 Hz).
   *
   * @param feet_targets the swing foot targets from the SLC this cycle
   * @param swing_progress per-leg swing progress in [0, 1]
   * @param swing_states per-leg SLC leg state
   */
  virtual void UpdateSwingLegState(
      const FeetTargets& feet_targets,
      const std::array<double, N_LEGS>& swing_progress,
      const std::array<SwingLegControllerInterface::LegState, N_LEGS>& swing_states) = 0;
  /**
   * Provides an updated model, used for the foot-position kinematics the hold
   * positions depend on (`CalcFootPositionInWorld`, `CalcFootPositionInBodyFrame`).
   * Only called when the model adaptation changed the model (event-driven).
   *
   * @param quad_model the new model
   */
  virtual void UpdateModel(const ModelInterface& quad_model) = 0;

  /**
   * Advances the per-leg contact FSM exactly one step and writes the reconciled
   * WBC inputs. Called exactly **once** per control cycle (500 Hz), after all the
   * Update* methods, under the host's `wbc_lock_`.
   *
   * This method mutates internal state (the per-leg status and the hold
   * positions), so unlike the `Get*` accessors it must not be called more than
   * once per cycle. It is the analogue of
   * `ModelAdaptationInterface::DoModelAdaptation`, the pipeline's only other
   * mutating per-cycle stage method.
   *
   * On return: `contacts` holds the reconciled stance flags (a foot forced to
   * stance by early contact reads true; a lost/late foot reads false); `wrenches`
   * holds the reconciled ground-reaction forces (zeroed in swing/late/lost,
   * next-stance wrench in early contact); `feet_targets` holds the reconciled
   * foot targets (hold positions with zero velocity/acceleration where the FSM
   * overrides the plan). All three are in/out: the host seeds them with
   * `wrench_sequence.forces[0]`, the SLC `feet_targets` and
   * `gait_sequence.contact_sequence[0]`, exactly as the pre-extraction inline
   * code did.
   *
   * @param contacts     in/out: per-leg stance flags, reconciled in place
   * @param wrenches     in/out: per-leg ground-reaction forces, reconciled in place
   * @param feet_targets in/out: per-leg foot targets, reconciled in place
   */
  virtual void Reconcile(FootContacts& contacts, Wrenches& wrenches, FeetTargets& feet_targets) = 0;

  /**
   * Returns the per-leg contact FSM state after the last `Reconcile`, for
   * diagnostics. Const and side-effect free; may be called more than once per
   * cycle.
   *
   * @param states out: per-leg contact state
   */
  virtual void GetLegContactStates(std::array<LegContactState, N_LEGS>& states) const = 0;
  /**
   * Returns the transition events of the last `Reconcile`, so the host can log
   * them and update the heartbeat counters. Const and side-effect free; may be
   * called more than once per cycle.
   *
   * @param events out: the per-leg transition events of the last cycle
   */
  virtual void GetContactEvents(ContactEvents& events) const = 0;

  /**
   * Applies a runtime parameter to this stage. Same contract as the other five
   * stages: called from the host parameter-event callback with the stage's mutex
   * held, never from the control loops; returns false for unrecognised keys (the
   * host logs a warning). The keys are the four detection toggles the FSM reads,
   * named in `contact_logic_params` above; the host wired them up in M3.1
   * (issue #12).
   *
   * @param name the full ROS parameter name
   * @param value the new parameter value
   * @return true if the key was recognised and applied, false otherwise
   */
  virtual bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) = 0;

  virtual ~ContactLogicInterface() = default;
};
