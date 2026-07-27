#pragma once

#include <rclcpp/parameter_value.hpp>

#include <Eigen/Core>
#include <array>
#include <memory>
#include <string>

#include "mit_controller/contact_logic_interface.hpp"

/**
 * The stock contact reconciliation stage (issue #12, M3.1).
 *
 * This is the per-leg early/late/lost-contact FSM that used to run inline in
 * `MITController::ControlLoopCallback`, moved behind `ContactLogicInterface`
 * unchanged: same states, same transition conditions in the same order, same
 * overrides of the WBC inputs. What the host kept is everything that is not the
 * FSM — the ROS logging and the heartbeat counter, which it now drives off
 * `GetContactEvents` instead of off the transitions themselves.
 *
 * Like the other stock stages this class is plain, host-free algorithm code; the
 * pluginlib packaging is the thin adapter in `src/plugins/contact_logic_plugins.cpp`
 * (plugin_lifecycle.md §4, Option B). It is deliberately *not* part of the
 * exported header surface — only the interface is, exactly as for
 * `SwingLegController` / `swing_leg_controller_interface.hpp`.
 *
 * Threading: none of its own. The host calls it from the control loop under
 * `wbc_lock_`, and routes `SetParameter` under the same lock
 * (contact_logic_interface.hpp), so this class takes no locks and does no
 * allocation on the per-cycle path.
 */
class DefaultContactLogic : public ContactLogicInterface {
 public:
  /**
   * @param early_contact_detection            accept a sensed contact during a scheduled swing
   * @param late_contact_detection             react to a missing contact at a scheduled stance
   * @param lost_contact_detection             react to a stance contact disappearing (slip)
   * @param late_contact_reschedule_swing_phase let a late-contact leg follow the plan back into
   *                                            swing even if it never regained contact
   * @param quad_model the model clone this stage owns (foot kinematics for the hold positions)
   * @param quad_state the state clone this stage owns (sensed contacts, joint positions, pose)
   */
  DefaultContactLogic(bool early_contact_detection,
                      bool late_contact_detection,
                      bool lost_contact_detection,
                      bool late_contact_reschedule_swing_phase,
                      std::unique_ptr<ModelInterface> quad_model,
                      std::unique_ptr<StateInterface> quad_state);

  void UpdateState(const StateInterface &quad_state) override;
  void UpdateGaitSequence(const GaitSequence &gait_sequence) override;
  void UpdateWrenchSequence(const WrenchSequence &wrench_sequence) override;
  void UpdateSwingLegState(const FeetTargets &feet_targets,
                           const std::array<double, N_LEGS> &swing_progress,
                           const std::array<SwingLegControllerInterface::LegState, N_LEGS> &swing_states) override;
  void UpdateModel(const ModelInterface &quad_model) override;

  void Reconcile(FootContacts &contacts, Wrenches &wrenches, FeetTargets &feet_targets) override;

  void GetLegContactStates(std::array<LegContactState, N_LEGS> &states) const override;
  void GetContactEvents(ContactEvents &events) const override;

  bool SetParameter(const std::string &name, const rclcpp::ParameterValue &value) override;

 private:
  // Detection toggles, `contact_logic.*` (contact_logic_interface.hpp). Set from
  // Init and re-settable at runtime through SetParameter, which is how they
  // behaved as host members.
  bool early_contact_detection_;
  bool late_contact_detection_;
  bool lost_contact_detection_;
  bool late_contact_reschedule_swing_phase_;

  // The pipeline inputs, from the Update* methods. Plain value members: every one
  // of these types is fixed-size, so a cycle's worth of updates is a handful of
  // memcpys and never touches the allocator.
  //
  // The `FeetTargets` of UpdateSwingLegState are deliberately not kept: the host
  // seeds `Reconcile`'s in/out `feet_targets` with exactly those values
  // (contact_logic_interface.hpp), so a second copy would be dead state.
  GaitSequence gait_sequence_;
  WrenchSequence wrench_sequence_;
  std::array<double, N_LEGS> swing_progress_;
  std::array<SwingLegControllerInterface::LegState, N_LEGS> swing_states_;

  // FSM state, carried across cycles. `feet_status_` starts in STANCE, as the
  // host's `feet_status_.fill(STANCE)` did.
  std::array<LegContactState, N_LEGS> feet_status_;
  std::array<Eigen::Vector3d, N_LEGS> early_contact_hold_position_;
  std::array<Eigen::Vector3d, N_LEGS> slip_hold_in_body_;
  std::array<Eigen::Vector3d, N_LEGS> last_feet_pos_targets_;

  // The transitions of the last Reconcile, for the host's logging and heartbeat.
  ContactEvents last_events_;

  std::unique_ptr<ModelInterface> quad_model_;
  std::unique_ptr<StateInterface> quad_state_;
};
