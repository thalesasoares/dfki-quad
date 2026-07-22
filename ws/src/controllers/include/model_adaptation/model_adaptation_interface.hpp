#pragma once
#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "gait_sequence.hpp"

class ModelAdaptationInterface {
 protected:
  ModelAdaptationInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  static const int NUM_PARAMS = 3;
  /**
   * Updates the current state. Called every model adaptation cycle (100 Hz).
   *
   * @param state the new state
   */
  virtual void UpdateState(const StateInterface& state) = 0;
  /**
   * Provides the newest gait sequence, used to know which legs are in contact.
   * Called every model adaptation cycle, after UpdateState.
   *
   * @param gs the new gait sequence
   */
  virtual void UpdateGaitSequence(const GaitSequence& gs) = 0;
  /**
   * Updates the model according to state data. This is the only stage method that mutates shared
   * pipeline data: on true the host broadcasts the new model to all other stages.
   * Called every model adaptation cycle, after UpdateGaitSequence.
   *
   * @param model the model to update
   * @return if the model was updated
   */
  virtual bool DoModelAdaptation(ModelInterface& model) = 0;
  /** Estimated parameter vector, for diagnostics. Must be side effect free. */
  virtual Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const = 0;
  /** Estimation covariance, for diagnostics. Must be side effect free. */
  virtual Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const = 0;
  /** Change applied by the last update, for diagnostics. Must be side effect free. */
  virtual Eigen::Vector<double, NUM_PARAMS> GetDelta() const = 0;
  /** Measured total force and torque, for diagnostics. Must be side effect free. */
  virtual Eigen::Vector<double, 6> GetTotalForceTorque() const = 0;
  /** Singular values of the estimation problem, for diagnostics. Must be side effect free. */
  virtual Eigen::Vector<double, NUM_PARAMS> GetSV() const = 0;

  virtual ~ModelAdaptationInterface() = default;
};