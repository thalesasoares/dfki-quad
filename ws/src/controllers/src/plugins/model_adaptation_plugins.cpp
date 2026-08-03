// Stock model-adaptation plugins (issue #8, M2.3).
//
// Thin adapters (plugin_lifecycle.md §4, Option B) around the two existing model
// adaptation implementations. Init mirrors the host's ma_ construction
// (mit_controller_node.cpp:469-501). In the host the branch was chosen by the
// ma_mode parameter; under plugins the `type:` selector picks the plugin, so
// each wrapper is one branch and neither reads ma_mode (that selection now lives
// in the loader, M2.5 owns the key).
//
// Behaviour quirks are preserved exactly (issue #8 criterion 3): rls zeroes the
// convergence threshold it is handed, and kf hard-codes gravity 9.81 and builds
// the noise matrices as identity with the parameter vector on the diagonal.

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "model_adaptation/kf_model_adaptation.hpp"
#include "model_adaptation/least_squares_model_adaptation.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

using MA = ModelAdaptationInterface;

/**
 * Stock plugin wrapping KFModelAdaptation (ma_mode default / 0).
 * Init mirrors the default branch of mit_controller_node.cpp:481-500.
 */
class KfAdaptationPlugin final : public StagePlugin<ModelAdaptationInterface> {
 public:
  void Init(StageInit init) override {
    const auto conv_thresh_vec = GetOr<std::vector<double>>(init, "ma_convergence_threshold", {0.695, 0.12, 0.11});
    const auto process_noise_vec = GetOr<std::vector<double>>(init, "ma_process_noise", {0.005, 0.0005, 0.0005});
    const auto measurement_noise_vec = GetOr<std::vector<double>>(
        init, "ma_measurement_noise", {1000.0, 1000.0, 10000.0, 10000.0, 10000.0, 1000.0});

    const Eigen::Vector<double, MA::NUM_PARAMS> conv_thresh =
        Eigen::Map<const Eigen::Vector<double, MA::NUM_PARAMS>>(conv_thresh_vec.data());

    Eigen::Matrix<double, MA::NUM_PARAMS, MA::NUM_PARAMS> process_noise;
    process_noise.setIdentity();
    process_noise.diagonal() = Eigen::Map<const Eigen::Vector<double, MA::NUM_PARAMS>>(process_noise_vec.data());

    Eigen::Matrix<double, 6, 6> measurement_noise;
    measurement_noise.setIdentity();
    measurement_noise.diagonal() = Eigen::Map<const Eigen::Vector<double, 6>>(measurement_noise_vec.data());

    impl_ = std::make_unique<KFModelAdaptation>(std::move(init.model),
                                                std::move(init.state),
                                                process_noise,
                                                measurement_noise,
                                                9.81,
                                                conv_thresh);
  }

  void UpdateState(const StateInterface& state) override { impl_->UpdateState(state); }
  void UpdateGaitSequence(const GaitSequence& gs) override { impl_->UpdateGaitSequence(gs); }
  bool DoModelAdaptation(ModelInterface& model) override { return impl_->DoModelAdaptation(model); }
  Eigen::Vector<double, MA::NUM_PARAMS> GetParameterVector() const override { return impl_->GetParameterVector(); }
  Eigen::Matrix<double, MA::NUM_PARAMS, MA::NUM_PARAMS> GetParameterCovariance() const override {
    return impl_->GetParameterCovariance();
  }
  Eigen::Vector<double, MA::NUM_PARAMS> GetDelta() const override { return impl_->GetDelta(); }
  Eigen::Vector<double, 6> GetTotalForceTorque() const override { return impl_->GetTotalForceTorque(); }
  Eigen::Vector<double, MA::NUM_PARAMS> GetSV() const override { return impl_->GetSV(); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<KFModelAdaptation> impl_;
};

/**
 * Stock plugin wrapping LeastSquaresModelAdaptation (ma_mode 1).
 * Init mirrors the case-1 branch of mit_controller_node.cpp:473-480. The
 * convergence threshold is deliberately zeroed (the host passed
 * conv_thresh.setZero(), "Estimation covariance is not thresholdable here").
 */
class RlsAdaptationPlugin final : public StagePlugin<ModelAdaptationInterface> {
 public:
  void Init(StageInit init) override {
    const Eigen::Vector<double, MA::NUM_PARAMS> conv_thresh = Eigen::Vector<double, MA::NUM_PARAMS>::Zero();
    impl_ = std::make_unique<LeastSquaresModelAdaptation>(std::move(init.model),
                                                          std::move(init.state),
                                                          conv_thresh,
                                                          GetOr<double>(init, "ma_forgetting_factor", 0.5));
  }

  void UpdateState(const StateInterface& state) override { impl_->UpdateState(state); }
  void UpdateGaitSequence(const GaitSequence& gs) override { impl_->UpdateGaitSequence(gs); }
  bool DoModelAdaptation(ModelInterface& model) override { return impl_->DoModelAdaptation(model); }
  Eigen::Vector<double, MA::NUM_PARAMS> GetParameterVector() const override { return impl_->GetParameterVector(); }
  Eigen::Matrix<double, MA::NUM_PARAMS, MA::NUM_PARAMS> GetParameterCovariance() const override {
    return impl_->GetParameterCovariance();
  }
  Eigen::Vector<double, MA::NUM_PARAMS> GetDelta() const override { return impl_->GetDelta(); }
  Eigen::Vector<double, 6> GetTotalForceTorque() const override { return impl_->GetTotalForceTorque(); }
  Eigen::Vector<double, MA::NUM_PARAMS> GetSV() const override { return impl_->GetSV(); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<LeastSquaresModelAdaptation> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::KfAdaptationPlugin, StagePlugin<ModelAdaptationInterface>)
PLUGINLIB_EXPORT_CLASS(stock_plugins::RlsAdaptationPlugin, StagePlugin<ModelAdaptationInterface>)
