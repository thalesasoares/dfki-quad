#pragma once

#include <memory>  // for the smart pointers
#include <queue>   // for persistency of excitation check

#include "common/filters.hpp"  //maybe not necessary?
#include "common/quaternion_operations.hpp"
#include "model_adaptation_interface.hpp"

class KFModelAdaptation : public ModelAdaptationInterface {
 public:
  KFModelAdaptation(std::unique_ptr<ModelInterface> quad_model,
                    std::unique_ptr<StateInterface> initial_state,
                    const Eigen::Ref<const Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS>> process_noise,
                    const Eigen::Ref<const Eigen::Matrix<double, 6, 6>> measurement_noise,
                    const double GRAVITY_CONSTANT,
                    const Eigen::Ref<const Eigen::Vector<double, NUM_PARAMS>> CONV_THRESH);
  void UpdateState(const StateInterface& state) override;
  void UpdateGaitSequence(const GaitSequence& gs) override;
  bool DoModelAdaptation(ModelInterface& model) override;
  Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const override;
  Eigen::Vector<double, NUM_PARAMS> GetDelta() const override;
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const override;
  Eigen::Vector<double, 6> GetTotalForceTorque() const override;
  Eigen::Vector<double, NUM_PARAMS> GetSV() const override;
  // No runtime-tunable parameters.
  bool SetParameter(const std::string& /*name*/, const rclcpp::ParameterValue& /*value*/) override {
    return false;
  }

 private:
  std::unique_ptr<StateInterface> state_;
  GaitSequence gait_sequence_;
  Eigen::Vector<double, NUM_PARAMS> parameter_vector_, conv_thresh_, sv_;
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> parameter_covariance_;
  Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> process_noise_;
  Eigen::Vector<double, 6> measurement_vector_;
  Eigen::Matrix<double, 6, 6> measurement_noise_;
  double g_;
  double leg_total_mass_;
  Eigen::Vector3d leg_total_mcom_;
  Eigen::Matrix3d leg_total_inertia_;

  void DoPredictionStep(const Eigen::Ref<const Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS>> process_noise,
                        Eigen::Ref<Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS>> parameter_covariance,
                        bool full_id) const;
  void DoUpdateStep(const Eigen::Matrix<double, 6, NUM_PARAMS>& regressor_matrix,
                    const Eigen::Ref<const Eigen::Matrix<double, 6, 6>> measurement_noise,
                    const Eigen::Ref<const Eigen::Vector<double, 6>> measurement_vector,
                    Eigen::Ref<Eigen::Vector<double, NUM_PARAMS>> parameter_vector,
                    Eigen::Ref<Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS>> parameter_covariance) const;
  void CalcRegressorMatrix(const Eigen::Ref<const Eigen::Vector3d> orientation,
                           const Eigen::Ref<const Eigen::Vector3d> linear_acceleration,
                           const Eigen::Ref<const Eigen::Vector3d> angular_acceleration,
                           const std::array<Eigen::Vector3d, 4>& contact_forces,
                           const double GRAVITY_CONSTANT,
                           Eigen::Ref<Eigen::Matrix<double, 6, 10>> y) const;
  void CalcMeasurementVector(const std::array<Eigen::Vector3d, 4>& contact_forces,
                             const std::array<Eigen::Vector3d, 4>& foot_positions,
                             const std::array<bool, 4>& contact_state,
                             const std::array<bool, 4>& planned_contact,
                             Eigen::Ref<Eigen::Vector<double, 6>> measurement_vector) const;
};
