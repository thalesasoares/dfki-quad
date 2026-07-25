// Stock MPC plugin (issue #8, M2.3).
//
// Thin adapter (plugin_lifecycle.md §4, Option B) around the existing acados MPC.
// Init is the host's MPC factory block (mit_controller_node.cpp:412-466): the
// state-weight reshaping, the mpc_solver name -> ocp_qp_solver_t mapping and the
// make_unique<MPC>(...) call, with get_parameter(x).as_T() replaced by
// Require<T>/GetOr<T>. The algorithm class is untouched.

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mit_controller/mpc.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

namespace {

// The mpc_solver string -> acados enum mapping (mit_controller_node.cpp:435-452).
// Unknown names fail loudly, as the host did (RCLCPP_ERROR + exit(-1)).
ocp_qp_solver_t ResolveMpcSolver(const std::string& name) {
  if (name == "PARTIAL_CONDENSING_HPIPM") return PARTIAL_CONDENSING_HPIPM;
  if (name == "PARTIAL_CONDENSING_OSQP") return PARTIAL_CONDENSING_OSQP;
  if (name == "FULL_CONDENSING_HPIPM") return FULL_CONDENSING_HPIPM;
  if (name == "FULL_CONDENSING_DAQP") return FULL_CONDENSING_DAQP;
  if (name == "FULL_CONDENSING_QPOASES") return FULL_CONDENSING_QPOASES;
  if (name == "PARTIAL_CONDENSING_QPDUNES") return PARTIAL_CONDENSING_QPDUNES;
  throw StageInitError("unknown mpc solver '" + name + "' for mpc_solver");
}

}  // namespace

/**
 * Stock plugin wrapping MPC (acados). Init mirrors mit_controller_node.cpp:412-466.
 */
class AcadosMpcPlugin final : public StagePlugin<MPCInterface> {
 public:
  void Init(StageInit init) override {
    const auto weights_stand = Require<std::vector<double>>(init, "mpc_state_weights_stand");
    const auto weights_move = Require<std::vector<double>>(init, "mpc_state_weights_move");
    if (weights_stand.size() != static_cast<size_t>(MPC::STATE_SIZE - 1)
        || weights_move.size() != static_cast<size_t>(MPC::STATE_SIZE - 1)) {
      throw StageInitError("mpc_state_weights_stand/move must have length " + std::to_string(MPC::STATE_SIZE - 1));
    }
    const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1> state_weights_stand =
        Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(weights_stand.data());
    const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1> state_weights_move =
        Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(weights_move.data());

    impl_ = std::make_unique<MPC>(
        Require<double>(init, "mpc_alpha"),
        state_weights_stand,
        state_weights_move,
        Require<double>(init, "mpc_mu"),
        Require<double>(init, "mpc_fmin"),
        Require<double>(init, "mpc_fmax"),
        std::move(init.state),
        std::move(init.model),
        ResolveMpcSolver(GetOr<std::string>(init, "mpc_solver", "PARTIAL_CONDENSING_HPIPM")),
        GetOr<int64_t>(init, "mpc_condensed_size", MPC_PREDICTION_HORIZON / 2),
        GetOr<std::string>(init, "mpc_hpipm_mode", "SPEED"),
        GetOr<int64_t>(init, "mpc_warm_start", 1),
        GetOr<double>(init, "mpc_solver_tolerances", -1.0),
        GetOr<std::string>(init, "mpc_osqp_linsys_solver", "qdldl"));
  }

  void UpdateState(const StateInterface& quad_state) override { impl_->UpdateState(quad_state); }
  void UpdateModel(const ModelInterface& quad_model) override { impl_->UpdateModel(quad_model); }
  void UpdateGaitSequence(const GaitSequence& gait_sequence) override { impl_->UpdateGaitSequence(gait_sequence); }
  void GetWrenchSequence(WrenchSequence& wrench_sequence,
                         MPCPrediction& state_prediction,
                         SolverInformation& solver_information) override {
    impl_->GetWrenchSequence(wrench_sequence, state_prediction, solver_information);
  }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<MPC> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::AcadosMpcPlugin, StagePlugin<MPCInterface>)
