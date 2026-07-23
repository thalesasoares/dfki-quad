#pragma once

#include <rclcpp/parameter_value.hpp>

#include <string>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "feet_targets.hpp"
#include "gait_sequence.hpp"

class SwingLegControllerInterface {
 public:
  enum LegState {
    /*
     * GS has scheduled this leg for standing
     */
    STANCE,
    /*
     * GS has scheduled the leg for swinging, but SLC has not yet moved this foot
     */
    NOT_STARTED,
    /**
     * SLC is currently swinging this leg
     */
    SWINGING,
    /**
     * SLC has reached the end of the slc for this feet
     */
    REACHED
  };

 protected:
  SwingLegControllerInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  /**
   * Provides the newest gait sequence. Only called when the gait sequencer produced a new one,
   * i.e. at most at MPC rate (100 Hz), not at SLC rate.
   *
   * @param gs the new gait sequence
   */
  virtual void UpdateGaitSequence(const GaitSequence &gs) = 0;
  /**
   * Provides the newest state. Called every SLC cycle (500 Hz) and additionally from the MPC
   * loop (100 Hz).
   *
   * @param state the new state
   */
  virtual void UpdateState(const StateInterface &state) = 0;
  /**
   * Provides an updated model. Only called when the model adaptation changed the model.
   *
   * @param model the new model
   */
  virtual void UpdateModel(const ModelInterface &model) = 0;
  /**
   * Returns the swing foot targets. Called every SLC cycle (500 Hz), after UpdateState.
   *
   * @param feet_targets out: position, velocity and acceleration target per leg
   */
  virtual void GetFeetTargets(FeetTargets &feet_targets) = 0;
  /**
   * Returns the swing progress per leg. Called every SLC cycle (500 Hz), after GetFeetTargets.
   *
   * @param progress out: swing progress in [0, 1] per leg
   * @param swing_states out: LegState per leg
   */
  virtual void GetProgress(std::array<double, N_LEGS> &progress, std::array<LegState, N_LEGS> &swing_states) = 0;
  /**
   * Returns the current swing trajectory endpoints, for visualisation only. Called from the
   * control loop, so possibly concurrently with the other methods (see doc/modularity).
   *
   * @param start_pos out: swing start position per leg
   * @param end_pos out: swing end position per leg
   */
  virtual void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS> &start_pos,
                               std::array<Eigen::Vector3d, N_LEGS> &end_pos) = 0;
  /**
   * Applies a runtime parameter to this stage.
   * Called from the host parameter-event callback with the stage's mutex (slc_lock_) held, never
   * from the control loops. An implementation that recognises no runtime parameters returns false.
   *
   * @param name the full ROS parameter name (e.g. "slc_swing_height")
   * @param value the new parameter value
   * @return true if the key was recognised and applied, false otherwise (the host logs a warning)
   */
  virtual bool SetParameter(const std::string &name, const rclcpp::ParameterValue &value) = 0;

  virtual ~SwingLegControllerInterface() = default;
};