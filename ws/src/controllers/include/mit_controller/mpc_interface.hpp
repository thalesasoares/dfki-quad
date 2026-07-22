#pragma once

#include <common/model_interface.hpp>

#include "gait_sequence.hpp"
#include "mpc_prediction.hpp"
#include "potato_sim/potato_model.hpp"
#include "wrench_sequence.hpp"

struct SolverInformation {
  bool success;
  int return_code;
  double total_solver_time;

  double acados_solve_QP_time;
  double acados_condensing_time;
  double acados_interface_time;
  int acados_num_iter;
  int acados_t_computed;

  double qp_objective_value;
  double qp_residuals[4]; //inf norm res: stat, dyn, ineq, comp
};

class MPCInterface {
 protected:
  MPCInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  /**
   * Provides the newest state. Called every MPC cycle (100 Hz).
   * May be called before the first state was received, so a default constructed state
   * has to be tolerated.
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
   * Provides the newest gait plan. Called every MPC cycle, after UpdateState.
   *
   * @param gait_sequence the new gait sequence
   */
  virtual void UpdateGaitSequence(const GaitSequence &gait_sequence) = 0;
  /**
   * Solves for the ground reaction forces over MPC_PREDICTION_HORIZON steps of MPC_DT.
   * Called every MPC cycle (100 Hz), after UpdateState and UpdateGaitSequence.
   * This call is allowed to block; the host warns when it exceeds MPC_CONTROL_DT.
   * All three outputs must be left in a usable state even when the solve did not converge,
   * because the host keeps running the pipeline on them.
   *
   * @param wrench_sequence out: ground reaction force per leg and horizon step
   * @param state_prediction out: predicted body pose and twist along the horizon
   * @param solver_information out: success flag, return code and solver timings
   */
  virtual void GetWrenchSequence(WrenchSequence &wrench_sequence,
                                 MPCPrediction &state_prediction,
                                 SolverInformation &solver_information) = 0;
  virtual ~MPCInterface() = default;
};