// Stock gait-sequencer plugins (issue #8, M2.3).
//
// Thin adapters (plugin_lifecycle.md §4, Option B) around the existing
// SimpleGaitSequencer / AdaptiveGaitSequencer. Each holds a unique_ptr to the
// untouched algorithm class, builds it inside Init from StageInit, and forwards
// the frozen GaitSequencerInterface methods to it. No algorithm code changes, so
// "no intentional behavior change" (issue #8 acceptance criterion 3) is
// auditable by reading this file against GetGaitSequencerFromParams in
// mit_controller_node.cpp — the Init bodies are that factory's two branches with
// get_parameter(x).as_T() replaced by Require<T>/GetOr<T>.
//
// bio_gait is deliberately absent: issue #8 defers it to M4.1 (#16).

#include <pluginlib/class_list_macros.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/sequence_containers.hpp"
#include "mit_controller/adaptive_gait_sequencer.hpp"
#include "mit_controller/gait.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/simple_gait_sequencer.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "plugins/plugin_param_utils.hpp"

namespace stock_plugins {

namespace {

// The four shoulder positions, laid out flat as [x,y,z]*4 in gs_shoulder_positions,
// exactly as the host read them (mit_controller_node.cpp:641-648). The caller
// keeps `flat` alive across the copy into the returned value array.
std::array<const Eigen::Vector3d, N_LEGS> ShoulderPositions(const std::vector<double>& flat) {
  return {Eigen::Map<const Eigen::Vector3d>(flat.data()),
          Eigen::Map<const Eigen::Vector3d>(flat.data() + 3),
          Eigen::Map<const Eigen::Vector3d>(flat.data() + 6),
          Eigen::Map<const Eigen::Vector3d>(flat.data() + 9)};
}

// The simple gait sequencer's gait string -> Gait mapping (mit_controller_node.cpp:599-636).
// Unknown names fail loudly, as the host did (RCLCPP_ERROR + return nullptr).
Gait BuildSimpleGait(const StageInit& init) {
  const std::string gait_str = GetOr<std::string>(init, "simple_gait_sequencer.gait", "STAND");
  if (gait_str == "Manual" || gait_str == "MANUAL") {
    const auto df = GetOr<std::vector<double>>(init, "simple_gait_sequencer.manual_gait.duty_factor",
                                               {0.6, 0.6, 0.6, 0.6});
    const auto po = GetOr<std::vector<double>>(init, "simple_gait_sequencer.manual_gait.phase_offset",
                                               {0.0, 0.5, 0.5, 0.0});
    if (df.size() != N_LEGS || po.size() != N_LEGS) {
      throw StageInitError("simple_gait_sequencer.manual_gait duty_factor/phase_offset must have length "
                           + std::to_string(N_LEGS));
    }
    return Gait(GetOr<double>(init, "simple_gait_sequencer.manual_gait.period", 0.5),
                to_array<N_LEGS>(df),
                to_array<N_LEGS>(po),
                MPC_DT);
  }
  if (gait_str == "STAND") return GaitDatabase::getGait(GaitDatabase::STAND, MPC_DT);
  if (gait_str == "STATIC_WALK") return GaitDatabase::getGait(GaitDatabase::STATIC_WALK, MPC_DT);
  if (gait_str == "WALKING_TROT") return GaitDatabase::getGait(GaitDatabase::WALKING_TROT, MPC_DT);
  if (gait_str == "TROT") return GaitDatabase::getGait(GaitDatabase::TROT, MPC_DT);
  if (gait_str == "FLYING_TROT") return GaitDatabase::getGait(GaitDatabase::FLYING_TROT, MPC_DT);
  if (gait_str == "PACE") return GaitDatabase::getGait(GaitDatabase::PACE, MPC_DT);
  if (gait_str == "BOUND") return GaitDatabase::getGait(GaitDatabase::BOUND, MPC_DT);
  if (gait_str == "ROTARY_GALLOP") return GaitDatabase::getGait(GaitDatabase::ROTARY_GALLOP, MPC_DT);
  if (gait_str == "TRAVERSE_GALLOP") return GaitDatabase::getGait(GaitDatabase::TRAVERSE_GALLOP, MPC_DT);
  if (gait_str == "PRONK") return GaitDatabase::getGait(GaitDatabase::PRONK, MPC_DT);
  throw StageInitError("unknown gait type '" + gait_str + "' for simple_gait_sequencer.gait");
}

}  // namespace

/**
 * Stock plugin wrapping SimpleGaitSequencer. Init is the "Simple" branch of
 * GetGaitSequencerFromParams (mit_controller_node.cpp:597-657).
 */
class SimpleGaitSequencerPlugin final : public StagePlugin<GaitSequencerInterface> {
 public:
  void Init(StageInit init) override {
    const auto shoulder_flat = Require<std::vector<double>>(init, "gs_shoulder_positions");
    if (shoulder_flat.size() != static_cast<size_t>(N_LEGS * 3)) {
      throw StageInitError("gs_shoulder_positions must have length " + std::to_string(N_LEGS * 3));
    }
    const Gait gait = BuildSimpleGait(init);
    impl_ = std::make_unique<SimpleGaitSequencer>(gait,
                                                  GetOr<double>(init, "raibert.k", 0.03),
                                                  ShoulderPositions(shoulder_flat),
                                                  std::move(init.state),
                                                  std::move(init.model),
                                                  GetOr<int64_t>(init, "raibert.filtersize", 20),
                                                  GetOr<bool>(init, "raibert.z_on_plane", false),
                                                  Require<bool>(init, "fix_standing_position"),
                                                  GetOr<double>(init, "fix_position_distance_threshold", 0.1),
                                                  GetOr<double>(init, "fix_position_angular_threshold", 0.26),
                                                  GetOr<double>(init, "fix_position_velocity_threshold", 0.1),
                                                  Require<bool>(init, "early_contact_detection"));
  }

  void GetGaitSequence(GaitSequence& gait_sequence) override { impl_->GetGaitSequence(gait_sequence); }
  void UpdateState(const StateInterface& quad_state) override { impl_->UpdateState(quad_state); }
  void UpdateModel(const ModelInterface& quad_model) override { impl_->UpdateModel(quad_model); }
  void UpdateTarget(const Target& new_target) override { impl_->UpdateTarget(new_target); }
  void GetGaitState(interfaces::msg::GaitState& state) override { impl_->GetGaitState(state); }
  GS_Type GetType() const override { return impl_->GetType(); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<SimpleGaitSequencer> impl_;
};

/**
 * Stock plugin wrapping AdaptiveGaitSequencer. Init is the "Adaptive" branch of
 * GetGaitSequencerFromParams (mit_controller_node.cpp:659-707).
 */
class AdaptiveGaitSequencerPlugin final : public StagePlugin<GaitSequencerInterface> {
 public:
  void Init(StageInit init) override {
    const auto shoulder_flat = Require<std::vector<double>>(init, "gs_shoulder_positions");
    if (shoulder_flat.size() != static_cast<size_t>(N_LEGS * 3)) {
      throw StageInitError("gs_shoulder_positions must have length " + std::to_string(N_LEGS * 3));
    }
    const auto po = GetOr<std::vector<double>>(init, "adaptive_gait_sequencer.gait.phase_offset",
                                               {0.0, 0.5, 0.5, 0.0});
    if (po.size() != N_LEGS) {
      throw StageInitError("adaptive_gait_sequencer.gait.phase_offset must have length " + std::to_string(N_LEGS));
    }
    AdaptiveGait gait(
        to_array<N_LEGS>(po),
        MPC_DT,
        GetOr<double>(init, "adaptive_gait_sequencer.gait.swing_time", 0.2),
        GetOr<int64_t>(init, "adaptive_gait_sequencer.gait.filter_size", 10),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.zero_velocity_threshold", 0.03),
        GetOr<bool>(init, "adaptive_gait_sequencer.gait.switch_offsets", true),
        to_array<2>(GetOr<std::vector<double>>(init, "adaptive_gait_sequencer.gait.gait_change_froude",
                                               {0.02, 0.006})),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.standing_foot_position_threshold", 0.08),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.min_v_cmd_factor", 0.5),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.max_correction_cycles", 2.0),
        GetOr<bool>(init, "adaptive_gait_sequencer.gait.correct_all", false),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.correction_period", 0.6),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.disturbance_correction", 0.0),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.min_v", 0.0),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.offset_delay", 0.0),
        GetOr<double>(init, "adaptive_gait_sequencer.gait.max_stride_length", 0.0));
    impl_ = std::make_unique<AdaptiveGaitSequencer>(gait,
                                                    GetOr<double>(init, "raibert.k", 0.03),
                                                    ShoulderPositions(shoulder_flat),
                                                    std::move(init.state),
                                                    std::move(init.model),
                                                    GetOr<int64_t>(init, "raibert.filtersize", 20),
                                                    GetOr<bool>(init, "raibert.z_on_plane", false),
                                                    Require<bool>(init, "fix_standing_position"),
                                                    GetOr<double>(init, "fix_position_distance_threshold", 0.1),
                                                    GetOr<double>(init, "fix_position_angular_threshold", 0.26),
                                                    GetOr<double>(init, "fix_position_velocity_threshold", 0.1),
                                                    Require<bool>(init, "early_contact_detection"));
  }

  void GetGaitSequence(GaitSequence& gait_sequence) override { impl_->GetGaitSequence(gait_sequence); }
  void UpdateState(const StateInterface& quad_state) override { impl_->UpdateState(quad_state); }
  void UpdateModel(const ModelInterface& quad_model) override { impl_->UpdateModel(quad_model); }
  void UpdateTarget(const Target& new_target) override { impl_->UpdateTarget(new_target); }
  void GetGaitState(interfaces::msg::GaitState& state) override { impl_->GetGaitState(state); }
  GS_Type GetType() const override { return impl_->GetType(); }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return impl_->SetParameter(name, value);
  }

 private:
  std::unique_ptr<AdaptiveGaitSequencer> impl_;
};

}  // namespace stock_plugins

PLUGINLIB_EXPORT_CLASS(stock_plugins::SimpleGaitSequencerPlugin, StagePlugin<GaitSequencerInterface>)
PLUGINLIB_EXPORT_CLASS(stock_plugins::AdaptiveGaitSequencerPlugin, StagePlugin<GaitSequencerInterface>)
