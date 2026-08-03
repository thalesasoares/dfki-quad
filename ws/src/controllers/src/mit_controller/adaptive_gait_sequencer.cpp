#include "adaptive_gait_sequencer.hpp"

#include <math.h>

#include <algorithm>
#include <array>
#include <optional>

#include "common/sequence_containers.hpp"
#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "mpc_trajectory_planner.hpp"
#include "raibert_foot_step_planner.hpp"
#include "target.hpp"

AdaptiveGaitSequencer::AdaptiveGaitSequencer(const AdaptiveGait& gait,
                                             double k,
                                             const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                                             std::unique_ptr<StateInterface> quad_state,
                                             std::unique_ptr<ModelInterface> quad_model,
                                             unsigned int raibert_filtersize,
                                             bool raibert_z_on_plane,
                                             bool fix_standing_position,
                                             double fix_position_distance_threshold,
                                             double fix_position_angular_threshold,
                                             double fix_position_velocity_threshold,
                                             bool early_contact_detection)
    : target_({}),
      quad_state_(std::move(quad_state)),
      quad_model_(std::move(quad_model)),
      gait_(gait),
      foot_step_planner_(shoulder_positions, *quad_state_, *quad_model_, raibert_filtersize, raibert_z_on_plane, k),
      trajectory_planner_(MPC_DT,
                          *quad_state_,
                          *quad_model_,
                          fix_standing_position,
                          fix_position_distance_threshold,
                          fix_position_angular_threshold,
                          fix_position_velocity_threshold),
      early_contact_detection_(early_contact_detection) {}

void AdaptiveGaitSequencer::GetGaitSequence(GaitSequence& gait_sequence) {
  // update MPC trajectory
  trajectory_planner_.plan_trajectory(gait_sequence, target_);
  // update contact stuff
  if (early_contact_detection_) {
    gait_.update_sequence(gait_sequence,
                          MPC_CONTROL_DT,
                          quad_state_->GetFeetContacts(),
                          foot_step_planner_);  // TODO: calculate real dt instead
  } else {
    gait_.update_sequence(
        gait_sequence, MPC_CONTROL_DT, std::nullopt, foot_step_planner_);  // TODO: calculate real dt instead
  }
  // update footstep positions
  foot_step_planner_.get_foot_position_sequence(gait_sequence, gait_);
  gait_sequence.time_stamp = quad_state_->GetTime();
}

void AdaptiveGaitSequencer::UpdateState(const StateInterface& quad_state) {
  // update model
  *quad_state_ = quad_state;  // The others update automatically as they hold a reference
}

void AdaptiveGaitSequencer::UpdateTarget(const Target& target) { target_ = target; }

void AdaptiveGaitSequencer::GetGaitState(interfaces::msg::GaitState& state) {
  gait_.get_current_period(state.period);
  gait_.get_current_contact(state.contact);
  gait_.get_current_duty_factor(state.duty_factor);
  gait_.get_current_phase(state.phase);
  gait_.get_current_phase_offset(state.phase_offset);
  state.gait_sequencer = (uint8_t)GetType();
}

void AdaptiveGaitSequencer::UpdateModel(const ModelInterface& model) { *quad_model_ = model; }

AdaptiveGait& AdaptiveGaitSequencer::Gait() { return gait_; }

bool AdaptiveGaitSequencer::SetParameter(const std::string& name, const rclcpp::ParameterValue& value) {
  const auto expect_double = value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
  const auto expect_bool = value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL;
  const auto expect_int = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER;
  const auto expect_double_array = value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;

  if (name == "adaptive_gait_sequencer.gait.phase_offset") {
    if (!expect_double_array || value.get<std::vector<double>>().size() != N_LEGS) return false;
    gait_.set_offset(to_array<N_LEGS>(value.get<std::vector<double>>()));
  } else if (name == "adaptive_gait_sequencer.gait.swing_time") {
    if (!expect_double) return false;
    gait_.set_swing_time(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.filter_size") {
    if (!expect_int) return false;
    gait_.set_filter_size(static_cast<unsigned int>(value.get<int64_t>()));
  } else if (name == "adaptive_gait_sequencer.gait.zero_velocity_threshold") {
    if (!expect_double) return false;
    gait_.set_zero_velocity_threshold(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.switch_offsets") {
    if (!expect_bool) return false;
    gait_.set_offset_switch(value.get<bool>());
  } else if (name == "adaptive_gait_sequencer.gait.gait_change_froude") {
    if (!expect_double_array || value.get<std::vector<double>>().size() != 2) return false;
    gait_.set_gait_change_froude(to_array<2>(value.get<std::vector<double>>()));
  } else if (name == "adaptive_gait_sequencer.gait.standing_foot_position_threshold") {
    if (!expect_double) return false;
    gait_.set_standing_foot_position_threshold(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.min_v_cmd_factor") {
    if (!expect_double) return false;
    gait_.set_min_v_cmd_factor(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.max_correction_cycles") {
    if (!expect_double) return false;
    gait_.set_max_correction_cycles(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.correct_all") {
    if (!expect_bool) return false;
    gait_.set_correct_all(value.get<bool>());
  } else if (name == "adaptive_gait_sequencer.gait.correction_period") {
    if (!expect_double) return false;
    gait_.set_correction_period(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.disturbance_correction") {
    if (!expect_double) return false;
    gait_.set_disturbance_correction(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.min_v") {
    if (!expect_double) return false;
    gait_.set_min_v(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.offset_delay") {
    if (!expect_double) return false;
    gait_.set_offset_delay(value.get<double>());
  } else if (name == "adaptive_gait_sequencer.gait.max_stride_length") {
    if (!expect_double) return false;
    gait_.set_max_stride_length(value.get<double>());
  } else {
    return false;
  }
  return true;
}
GS_Type AdaptiveGaitSequencer::GetType() const { return ADAPTIVE; }
