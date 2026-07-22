#pragma once

#include "common/model_interface.hpp"
#include "common/quaternion_operations.hpp"
#include "common/state_interface.hpp"
#include "gait_sequence.hpp"
#include "gait_sequencer_types.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "potato_sim/potato_model.hpp"
#include "rclcpp/time.hpp"
#include "target.hpp"

class GaitSequencerInterface {
 protected:
  GaitSequencerInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  /**
   * Returns the gait plan for the next GAIT_SEQUENCE_SIZE steps of MPC_DT.
   * Called every MPC cycle (100 Hz), after UpdateTarget and UpdateState.
   *
   * @param gait_sequence out: contact schedule, foot placements and reference trajectory
   */
  virtual void GetGaitSequence(GaitSequence& gait_sequence) = 0;
  /**
   * Provides the newest state. Called every MPC cycle (100 Hz).
   * May be called before the first state was received, so a default constructed state
   * has to be tolerated.
   *
   * @param quad_state the new state
   */
  virtual void UpdateState(const StateInterface& quad_state) = 0;
  /**
   * Provides an updated model. Only called when the model adaptation changed the model.
   *
   * @param quad_model the new model
   */
  virtual void UpdateModel(const ModelInterface& quad_model) = 0;

  /**
   * Provides the newest operator target. Called once at construction and then every MPC cycle,
   * before UpdateState.
   *
   * @param new_target the new target
   */
  virtual void UpdateTarget(const Target& new_target) = 0;
  /**
   * Returns the internal gait state, for diagnostics only.
   * Called from the MPC loop after the gait sequencer lock was released (see doc/modularity).
   *
   * @param state out: the gait state message
   */
  virtual void GetGaitState(interfaces::msg::GaitState& state) = 0;
  /** Identifies which gait sequencer implementation this is. */
  virtual GS_Type GetType() const = 0;

  virtual ~GaitSequencerInterface() = default;
};
