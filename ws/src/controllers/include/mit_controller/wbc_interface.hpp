#pragma once

#include <rclcpp/parameter_value.hpp>

#include <string>

#include "common/model_interface.hpp"
#include "feet_targets.hpp"
#include "gait_sequence.hpp"
#include "joint_commands.hpp"
#include "mpc_prediction.hpp"
#include "wrench_sequence.hpp"

struct WBCReturn {
  bool success;
  double qp_update_time;
  double qp_solve_time;
};

/**
 * Which family of commands a whole-body controller produces (issue #13, M3.2).
 *
 * Until M3.2 this was a *compile-time* property: `WBCInterface` was a class
 * template keyed on the command struct, so a build produced exactly one
 * instantiation and choosing ARC-OPT over inverse dynamics meant rebuilding
 * (gap G8, doc/modularity/stage_contracts.md §5). It is now a runtime property
 * of the loaded plugin: the host reads it once at bring-up, checks it against
 * `leg_control_mode`, and then calls the matching getter every cycle.
 *
 * The pairing with `MITController::LEGControlMode` is the host's to enforce —
 * see `WBCCommandModeForLegControlMode` in `stage_selection.hpp`.
 */
enum class WBCCommandMode {
  kJoint,     //!< joint torque/velocity/position commands (`JointTorqueVelocityPositionCommands`)
  kCartesian  //!< Cartesian foot position/velocity/force commands (`CartesianCommands`)
};

class WBCInterface {
 protected:
  WBCInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  typedef std::array<Eigen::Vector3d, ModelInterface::N_LEGS> Wrenches;
  typedef std::array<bool, ModelInterface::N_LEGS> FootContact;

  /**
   * Provides the newest state. Called every control cycle (500 Hz), first of all Update* methods.
   *
   * @param quad_state the new state
   */
  virtual void UpdateState(const StateInterface &quad_state) = 0;
  /**
   * Provides an updated model. Only called when the model adaptation changed the model.
   *
   * @param quad_model the new model
   */
  virtual void UpdateModel(const ModelInterface &quad_model) = 0;
  /**
   * Provides the swing foot targets from the swing leg controller. Called every control cycle.
   *
   * @param feet_targets position, velocity and acceleration target per leg
   */
  virtual void UpdateFeetTarget(const FeetTargets &feet_targets) = 0;
  /**
   * Provides the ground reaction forces from the MPC, after the host applied the contact logic.
   * Called every control cycle.
   *
   * @param wrenches force per leg
   */
  virtual void UpdateWrenches(const Wrenches &wrenches) = 0;
  /**
   * Provides the current contact state per leg, as determined by the host contact logic.
   * Called every control cycle.
   *
   * @param foot_contact true if the leg is in stance
   */
  virtual void UpdateFootContact(const FootContact &foot_contact) = 0;
  /**
   * Provides the body target. Note that the host passes the MPC prediction one step ahead here,
   * not the raw gait sequence target. Called every control cycle.
   *
   * @param orientation target orientation in world frame
   * @param position target position in world frame
   * @param lin_vel target linear velocity in world frame
   * @param ang_vel target angular velocity in world frame
   */
  virtual void UpdateTarget(const Eigen::Quaterniond &orientation,
                            const Eigen::Vector3d &position,
                            const Eigen::Vector3d &lin_vel,
                            const Eigen::Vector3d &ang_vel) = 0;
  /**
   * Which command family this controller produces. Fixed for the lifetime of the instance:
   * the host reads it once at bring-up to pick the getter below and to reject a
   * `leg_control_mode` it cannot serve. Never called from a control loop.
   *
   * @return the mode whose getter actually solves; the other one is a stub
   */
  virtual WBCCommandMode SupportedCommandMode() const = 0;
  /**
   * Solves for the joint commands. Called every control cycle (500 Hz), after all Update* methods,
   * on implementations reporting `WBCCommandMode::kJoint`.
   * Must leave joint_command in a usable state even when the solve did not converge.
   *
   * An implementation of the *other* mode must return `{false, 0.0, 0.0}` and leave the out
   * parameter untouched. The host never takes that path — it validates the mode at bring-up — so
   * the stub is defence in depth, not a code path.
   *
   * @param joint_command out: the commands to send to the leg driver
   * @return success flag and solver timings
   */
  virtual WBCReturn GetJointCommand(JointTorqueVelocityPositionCommands &joint_command) = 0;
  /**
   * Solves for the Cartesian foot commands. The `WBCCommandMode::kCartesian` counterpart of
   * `GetJointCommand`, with the same contract, including the stub requirement for the other mode.
   *
   * @param cartesian_command out: the commands to send to the leg driver
   * @return success flag and solver timings
   */
  virtual WBCReturn GetCartesianCommand(CartesianCommands &cartesian_command) = 0;
  /**
   * Applies a runtime parameter to this stage.
   * Called from the host parameter-event callback with the stage's mutex (wbc_lock_) held, never
   * from the control loops. An implementation that recognises no runtime parameters returns false.
   *
   * @param name the full ROS parameter name (e.g. "wbc.inverse_dynamics.target_velocity_blend")
   * @param value the new parameter value
   * @return true if the key was recognised and applied, false otherwise (the host logs a warning)
   */
  virtual bool SetParameter(const std::string &name, const rclcpp::ParameterValue &value) = 0;

  virtual ~WBCInterface() = default;
};
