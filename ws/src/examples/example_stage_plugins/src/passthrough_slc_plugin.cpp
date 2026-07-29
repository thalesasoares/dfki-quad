// Example out-of-package swing-leg-controller stage plugin (issue #17, M4.2).
//
// WHAT THIS IS
//
// A demonstration stage, not production locomotion. It fills the swing-leg
// controller slot of the control pipeline with identity behaviour: every foot is
// commanded to stay exactly where it already is, so the robot stands and never
// steps. Optionally it logs the scheduled leg states at a throttled rate, which
// makes it useful as a diagnostic probe — drop it into `slc.type` to see what the
// gait sequencer is actually asking the swing stage to do, with no swing motion
// confusing the picture.
//
// WHY IT EXISTS
//
// M4's exit criterion is that a *third party* can add a control stage without
// editing the host. `bio_gait` (#16) proved half of that — a new stage reachable
// by one `type:` value and no host edit — but it shipped inside `controllers`,
// next to the code it extends. This plugin proves the other half: it lives in a
// separate ament package, compiles against nothing but the installed export
// surface of `controllers`, and is found by the host's ordinary discovery path.
// Nothing in `controllers` knows this file exists.
//
// Read alongside this package's README.md, which is the human-facing version of
// the same story, and doc/modularity/plugin_lifecycle.md, which is the contract
// the class below implements.

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>

#include <array>
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"

namespace example_stage_plugins {
namespace {

// The one parameter this stage reads, spelled once. Start-up configuration and
// runtime reconfiguration share a single key vocabulary
// (plugin_lifecycle.md §3), so this constant serves both `Init` and
// `SetParameter` — they cannot drift apart.
constexpr char kLogPeriodKey[] = "example_passthrough_slc.log_period";

// Logging off. Chosen as the default so that selecting this stage costs nothing
// it does not have to: a demo plugin that spams the console by default is a demo
// plugin people turn off rather than read.
constexpr double kLogPeriodDisabled = 0.0;

// Plugins are constructed by pluginlib, not by the node, so there is no node
// handle and no `this->get_logger()`. A free-standing named logger is the
// supported way for a stage to log; the name matches the plugin id, so the
// console line points straight at the `slc.type` value that produced it.
rclcpp::Logger PluginLogger() { return rclcpp::get_logger("example_passthrough_slc"); }

}  // namespace

/**
 * Identity / passthrough swing leg controller with optional throttled logging.
 *
 * Behaviour, per the frozen `SwingLegControllerInterface` contract:
 *
 *   * every foot target is the foot's *current* world position, with zero
 *     velocity and zero acceleration — the "passthrough" in the name;
 *   * a leg the gait sequencer schedules for stance reports `STANCE` at progress
 *     0; a leg scheduled for swing reports `REACHED` at progress 1.0, i.e. a
 *     swing that is instantly complete at the position it started from. That
 *     mapping is what keeps the downstream contact logic benign: a swing leg that
 *     is already `REACHED` at its current position produces no motion command and
 *     no unexpected contact transition;
 *   * the visualisation trajectory is the degenerate one, start == end == the
 *     held position.
 *
 * The net effect on the robot is a stand. Commanding a walk while this stage is
 * loaded is a documented no-op, not a bug (README.md §1).
 */
class PassthroughSlcPlugin final : public StagePlugin<SwingLegControllerInterface> {
 public:
  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------

  /**
   * Phase two of construction (plugin_lifecycle.md §2). Takes the model clone,
   * reads the one optional parameter, and seeds the held foot positions from the
   * state clone so that a `GetFeetTargets` arriving before the first
   * `UpdateState` still returns something physically meaningful rather than the
   * origin.
   */
  void Init(StageInit init) override {
    if (init.model == nullptr) {
      throw StageInitError("example_passthrough_slc requires a model, but StageInit carried none");
    }
    model_ = std::move(init.model);

    log_period_ = ReadLogPeriod(init);

    // Seed from the state clone the host handed us. The host calls Init only
    // after the first /quad_state has arrived, so this is a valid snapshot
    // (stage_plugin.hpp, StageInit) — which is precisely why the clone is worth
    // reading here even though the stage keeps no state member afterwards.
    if (init.state != nullptr) {
      HoldFeetAt(*init.state);
    }

    // The one unthrottled line this stage ever logs: which stage got loaded and
    // what it will do to the robot. Selecting a demo controller by accident
    // should be obvious in the bring-up log, not discovered on the floor.
    RCLCPP_INFO(PluginLogger(),
                "example_passthrough_slc loaded: feet will HOLD their current position (no swing motion). "
                "This is a demonstration/diagnostic stage, not production locomotion. Logging %s.",
                LoggingEnabled() ? ("every " + std::to_string(log_period_) + " s").c_str() : "disabled");
  }

  // ---------------------------------------------------------------------------
  // Per-cycle interface. See the performance note at the bottom of this file.
  // ---------------------------------------------------------------------------

  /** 500 Hz. Re-reads where the feet are; that position *is* the target. */
  void UpdateState(const StateInterface& state) override { HoldFeetAt(state); }

  /** Only when model adaptation changed the model. */
  void UpdateModel(const ModelInterface& model) override { *model_ = model; }

  /**
   * At most 100 Hz. Latches the scheduled stance/swing flags for the current
   * step, and — because this is the slowest of the three per-cycle entry points —
   * carries the throttled log.
   */
  void UpdateGaitSequence(const GaitSequence& gs) override {
    for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
      // Index 0 of the contact sequence is the step being executed now; the rest
      // of the horizon is for stages that plan ahead, which this one does not.
      scheduled_stance_[leg] = gs.contact_sequence[0][leg];
    }
    LogIfDue();
  }

  /** 500 Hz. The identity: hold position, no velocity, no acceleration. */
  void GetFeetTargets(FeetTargets& feet_targets) override {
    for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
      feet_targets.positions[leg] = held_positions_[leg];
      feet_targets.velocities[leg].setZero();
      feet_targets.accelerations[leg].setZero();
    }
  }

  /** 500 Hz, after GetFeetTargets. Scheduled stance → STANCE; swing → REACHED. */
  void GetProgress(std::array<double, N_LEGS>& progress, std::array<LegState, N_LEGS>& swing_states) override {
    for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
      const bool stance = scheduled_stance_[leg];
      swing_states[leg] = stance ? STANCE : REACHED;
      progress[leg] = stance ? 0.0 : 1.0;
    }
  }

  /** Visualisation only. A held foot has a zero-length trajectory. */
  void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS>& start_pos,
                       std::array<Eigen::Vector3d, N_LEGS>& end_pos) override {
    // Called from the control loop, so possibly concurrently with the methods
    // above. No lock is taken here for the same reason the stock `bezier_swing`
    // takes none: the frozen interface documents this getter as viewport data,
    // and the stock stage this one stands in for has always had the same
    // property. A plugin that needed stronger guarantees would be free to lock —
    // the contract permits it — but it must not be *more* synchronised than the
    // stage it replaces without saying so, because that cost lands in the loop.
    for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
      start_pos[leg] = held_positions_[leg];
      end_pos[leg] = held_positions_[leg];
    }
  }

  // ---------------------------------------------------------------------------
  // Runtime reconfiguration
  // ---------------------------------------------------------------------------

  /**
   * Called from the host's parameter-event callback with the stage's lock held,
   * never from a control loop (plugin_lifecycle.md §3), so a plain member write
   * is sufficient here.
   *
   * Returning `false` for an unrecognised key is load-bearing: the host uses the
   * return value to warn that a parameter change did not apply. Silently
   * accepting an unknown key would make a typo look like a working setting.
   */
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    if (name != kLogPeriodKey) {
      return false;
    }
    double period = kLogPeriodDisabled;
    try {
      period = value.get<double>();
    } catch (const std::exception& error) {
      RCLCPP_WARN(PluginLogger(), "ignoring '%s': expected a double (%s)", kLogPeriodKey, error.what());
      return false;
    }
    if (period < 0.0) {
      RCLCPP_WARN(PluginLogger(), "ignoring '%s': must be >= 0 (0 disables logging), got %f", kLogPeriodKey, period);
      return false;
    }
    log_period_ = period;
    // Make the next line due immediately, so a human who just turned logging on
    // sees output now rather than one period from now.
    last_log_ = SteadyClock::time_point{};
    return true;
  }

 private:
  using SteadyClock = std::chrono::steady_clock;

  /**
   * The optional-key read (plugin_lifecycle.md §3: the default lives in the
   * stage, not the host). `controllers` has helpers for this shape
   * (src/plugins/plugin_param_utils.hpp) but they are deliberately internal to
   * that package and not installed, so an out-of-package plugin writes the four
   * lines itself. The error contract is what actually matters and is reproduced
   * faithfully: a malformed value is a `StageInitError` that *names the key*.
   */
  static double ReadLogPeriod(const StageInit& init) {
    const auto it = init.params.find(kLogPeriodKey);
    if (it == init.params.end()) {
      return kLogPeriodDisabled;
    }
    double period = kLogPeriodDisabled;
    try {
      period = it->second.get<double>();
    } catch (const std::exception& error) {
      throw StageInitError(std::string("optional stage parameter '") + kLogPeriodKey
                           + "' has the wrong type (expected a double): " + error.what());
    }
    if (period < 0.0) {
      throw StageInitError(std::string("stage parameter '") + kLogPeriodKey
                           + "' must be >= 0 (0 disables logging), got " + std::to_string(period));
    }
    return period;
  }

  /** Forward kinematics for all four feet — the whole of this stage's per-cycle work. */
  void HoldFeetAt(const StateInterface& state) {
    for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
      held_positions_[leg] = model_->CalcFootPositionInWorld(leg, state);
    }
  }

  bool LoggingEnabled() const { return log_period_ > 0.0; }

  /**
   * The throttle. When logging is disabled — the default — this is a single
   * double comparison and an immediate return: no clock read, no formatting, no
   * I/O. When it is enabled, the clock read is the only added cost on cycles
   * where the period has not elapsed. See the performance note below.
   */
  void LogIfDue() {
    if (!LoggingEnabled()) {
      return;
    }
    const SteadyClock::time_point now = SteadyClock::now();
    const auto period = std::chrono::duration<double>(log_period_);
    if (last_log_ != SteadyClock::time_point{} && now - last_log_ < period) {
      return;
    }
    last_log_ = now;
    RCLCPP_INFO(PluginLogger(),
                "scheduled [%s %s %s %s] holding at z = [%.3f %.3f %.3f %.3f]",
                scheduled_stance_[0] ? "stance" : "swing", scheduled_stance_[1] ? "stance" : "swing",
                scheduled_stance_[2] ? "stance" : "swing", scheduled_stance_[3] ? "stance" : "swing",
                held_positions_[0].z(), held_positions_[1].z(), held_positions_[2].z(), held_positions_[3].z());
  }

  std::unique_ptr<ModelInterface> model_;

  // Every per-cycle member is a fixed-size array sized at compile time, so no
  // control-loop call allocates.
  std::array<Eigen::Vector3d, N_LEGS> held_positions_{};
  std::array<bool, N_LEGS> scheduled_stance_{{true, true, true, true}};

  double log_period_ = kLogPeriodDisabled;
  SteadyClock::time_point last_log_{};
};

}  // namespace example_stage_plugins

// The base-class-type string must match `stage_plugin_bases::kSwingLegController`
// and the `base_class_type` attribute in plugins/example_slc_plugins.xml
// character for character — that string *is* the schema
// (doc/modularity/plugin_discovery.md §3).
PLUGINLIB_EXPORT_CLASS(example_stage_plugins::PassthroughSlcPlugin, StagePlugin<SwingLegControllerInterface>)

// ---------------------------------------------------------------------------
// Performance note (the discipline every stage plugin owes the pipeline)
// ---------------------------------------------------------------------------
//
// This stage sits in the 500 Hz swing loop, so "it is only a demo" is not an
// excuse for it to be expensive; a demo that visibly perturbs loop timing teaches
// the wrong lesson.
//
//   * Per 500 Hz cycle: four `CalcFootPositionInWorld` calls in `UpdateState`,
//     plus fixed-size copies in `GetFeetTargets` / `GetProgress`. That is strictly
//     less work than the stock `bezier_swing`, which does the same forward
//     kinematics and then evaluates a Bezier spline per leg.
//   * Per 100 Hz cycle: four boolean reads, plus the throttle check.
//   * No heap allocation on any per-cycle path — every member is a fixed-size
//     array. No I/O except when the log period has actually elapsed.
//   * Logging defaults to off, so the default configuration's added cost over an
//     empty stage is one predictable branch per gait-sequence update.
//   * Built with the node's own Release flags (see CMakeLists.txt), so none of
//     the above is silently deoptimised relative to the stock path.
