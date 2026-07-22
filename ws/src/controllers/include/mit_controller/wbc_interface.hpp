#pragma once

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

template <class JointCommandType>
class WBCInterface {
 protected:
  WBCInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  typedef JointCommandType JOINT_COMMAND_TYPE;
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
   * Solves for the joint commands. Called every control cycle (500 Hz), after all Update* methods.
   * Must leave joint_command in a usable state even when the solve did not converge.
   *
   * @param joint_command out: the commands to send to the leg driver
   * @return success flag and solver timings
   */
  virtual WBCReturn GetJointCommand(JointCommandType &joint_command) = 0;

  virtual ~WBCInterface() = default;
};
