#pragma once

#include <algorithm>  // for some array functions
#include <memory>     // for the smart pointers

#include "common/filters.hpp"
#include "model_adaptation_interface.hpp"

class LeastSquaresModelAdaptation : public ModelAdaptationInterface {
 public:
  LeastSquaresModelAdaptation(std::unique_ptr<ModelInterface> quad_model,
                              std::unique_ptr<StateInterface> initial_state,
                              const Eigen::Ref<const Eigen::Vector<double, NUM_PARAMS>> CONV_THRESH,
                              double lambda);
  void UpdateState(const StateInterface& state) override;
  void UpdateGaitSequence(const GaitSequence& gs) override;
  bool DoModelAdaptation(ModelInterface& model) override;
  Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const override;
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const override;
  Eigen::Vector<double, NUM_PARAMS> GetDelta() const override;
  Eigen::Vector<double, 6> GetTotalForceTorque() const override;
  Eigen::Vector<double, NUM_PARAMS> GetSV() const override;
  // No runtime-tunable parameters.
  bool SetParameter(const std::string& /*name*/, const rclcpp::ParameterValue& /*value*/) override {
    return false;
  }

 private:
  void CalcMeasurementVector(const std::array<Eigen::Vector3d, ModelInterface::N_LEGS>& contact_forces,
                             const std::array<Eigen::Vector3d, ModelInterface::N_LEGS>& foot_positions,
                             const std::array<bool, ModelInterface::N_LEGS>& contact_state,
                             const std::array<bool, ModelInterface::N_LEGS>& planned_contact,
                             Eigen::Ref<Eigen::Vector<double, 6>> measurement_vector) const;

  std::unique_ptr<StateInterface> state_;
  GaitSequence gait_sequence_;
  Eigen::Vector<double, NUM_PARAMS> parameter_vector_;
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> parameter_covariance_;
  Eigen::Vector<double, NUM_PARAMS> innovation_;
  Eigen::Vector<double, 6> measurement_vector_;
  Eigen::Vector<double, NUM_PARAMS> conv_thresh_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> contact_forces_;
  double lambda_;
  const double g_ = 9.81;  // gravity TODO: make model parameter
  double leg_total_mass_;
  Eigen::Vector3d leg_total_mcom_;
};
