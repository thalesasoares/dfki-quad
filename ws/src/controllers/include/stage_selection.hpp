#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/wbc_interface.hpp"

/**
 * Which stage implementation the host loads, and how that is spelled in the
 * parameters (issue #9, M2.4).
 *
 * The host no longer constructs any concrete algorithm; it resolves one string
 * per stage through `StageLoader` (`mit_controller/stage_loader.hpp`). This
 * header owns the two things that decision needs and that a `StageLoader` must
 * *not* know: the parameter names, and the stock pluginlib class names.
 *
 * ## The legacy bridge
 *
 * M2.4 landed before the YAML schema work of #10 (M2.5), so the shipped configs
 * carried no `*.type` keys. To keep "launch still works, unchanged" true across
 * that refactor, the host *declares* each `*.type` parameter with a default
 * derived from the parameter that used to pick the implementation inside the
 * deleted host factories:
 *
 * | Key | Legacy source |
 * |---|---|
 * | `gs.type` | `gait_sequencer` ("Simple" / "Adaptive") |
 * | `mpc.type` | — (one stock MPC) |
 * | `slc.type` | — (one stock SLC) |
 * | `wbc.type` | `USE_WBC`, i.e. the `ROBOT_MODEL` build flavour (a default only, since #13) |
 * | `model_adaptation.type` | `ma_mode` (0 = Kalman filter, 1 = recursive least squares) |
 * | `contact_logic.type` | — (the FSM was inline host code until #12) |
 *
 * An explicitly set `*.type` always wins — the derivation only supplies the
 * *default* of the declared parameter. M2.5 wrote the keys into
 * `mit_controller_{sim,real}_go2.yaml` and moved the last runtime caller
 * (`scripts/joy_to_target.py`) onto `gs.type`, so on the stock Go2 path the
 * derivation is now dead weight rather than a behaviour. It survives because the
 * ULab configs still select through the declared defaults and out-of-tree
 * configs may still spell the legacy keys; #23 (M5.4) deletes it together with
 * them.
 *
 * This is deliberately **not** a stage factory: it maps strings to strings and
 * never names a C++ type, so the host stays free of the concrete-algorithm
 * `if/else` that issue #9 removes. Unknown legacy values are passed through
 * verbatim rather than corrected or defaulted, so a typo surfaces as
 * `StageLoader`'s fail-fast error listing the declared plugins
 * (`doc/modularity/stage_loading.md` §1) instead of silently walking with the
 * wrong controller.
 *
 * Host-only: not part of the exported plugin surface, and not installed.
 */
namespace stage_selection {

/** Stage selection parameter names — the vocabulary of stage_loading.md §4. */
inline constexpr char kGaitSequencerTypeKey[] = "gs.type";
inline constexpr char kMPCTypeKey[] = "mpc.type";
inline constexpr char kSwingLegControllerTypeKey[] = "slc.type";
inline constexpr char kWBCTypeKey[] = "wbc.type";
inline constexpr char kModelAdaptationTypeKey[] = "model_adaptation.type";
inline constexpr char kContactLogicTypeKey[] = "contact_logic.type";

/** The legacy parameters the defaults above are derived from. */
inline constexpr char kLegacyGaitSequencerKey[] = "gait_sequencer";
inline constexpr char kLegacyModelAdaptationModeKey[] = "ma_mode";

/**
 * The pre-M3.1 flat spelling of the contact stage's four detection toggles.
 *
 * These are not *selection* keys — they configure the stage rather than choose
 * it — but they are legacy keys bridged onto a ratified spelling, which is the
 * one thing this header exists to keep in one place. #23 (M5.4) deletes them
 * together with the two selectors above.
 */
inline constexpr char kLegacyEarlyContactDetectionKey[] = "early_contact_detection";
inline constexpr char kLegacyLateContactDetectionKey[] = "late_contact_detection";
inline constexpr char kLegacyLostContactDetectionKey[] = "lost_contact_detection";
inline constexpr char kLegacyLateContactRescheduleSwingPhaseKey[] = "late_contact_reschedule_swing_phase";

/** Stock plugin class names, as declared in the `plugins/` XMLs (M2.3, issue #8). */
inline constexpr char kSimpleGaitPlugin[] = "simple_gait";
inline constexpr char kAdaptiveGaitPlugin[] = "adaptive_gait";
// Added by M4.1 (#16), and deliberately absent from GaitSequencerTypeFromLegacy
// below: the pre-plugin factory had no "Bio" branch, so there is no legacy
// spelling to bridge and none is invented for a shim #23 (M5.4) deletes.
inline constexpr char kBioGaitPlugin[] = "bio_gait";
inline constexpr char kAcadosMPCPlugin[] = "acados_mpc";
inline constexpr char kBezierSwingPlugin[] = "bezier_swing";
inline constexpr char kWbcArcOptPlugin[] = "wbc_arc_opt";
inline constexpr char kInverseDynamicsPlugin[] = "inverse_dynamics";
inline constexpr char kKfAdaptationPlugin[] = "kf_adaptation";
inline constexpr char kRlsAdaptationPlugin[] = "rls_adaptation";
inline constexpr char kDefaultContactLogicPlugin[] = "default_contact_logic";

/**
 * `gait_sequencer` → gait sequencer plugin class.
 *
 * "Simple" / "Adaptive" are the two values the deleted
 * `GetGaitSequencerFromParams` accepted. Anything else — including a class name
 * a user already wrote in the new spelling — is returned unchanged, so the
 * loader decides whether it exists.
 */
inline std::string GaitSequencerTypeFromLegacy(const std::string& gait_sequencer) {
  if (gait_sequencer == "Simple") {
    return kSimpleGaitPlugin;
  }
  if (gait_sequencer == "Adaptive") {
    return kAdaptiveGaitPlugin;
  }
  return gait_sequencer;
}

/**
 * `ma_mode` → model adaptation plugin class. 1 selected the recursive least
 * squares estimator; every other value fell through to the Kalman filter in the
 * deleted `switch`, and that "default:" behaviour is preserved here.
 */
inline std::string ModelAdaptationTypeFromLegacy(int64_t ma_mode) {
  return ma_mode == 1 ? kRlsAdaptationPlugin : kKfAdaptationPlugin;
}

/**
 * Flat contact toggle → the ratified `contact_logic.*` key it is bridged onto,
 * or `nullptr` when `name` is not one of the four.
 *
 * Unlike the two mappings above this one bridges *keys*, not values: the host
 * declares each `contact_logic.*` toggle with the flat key's value as its
 * default, and re-routes a runtime change of a flat key onto its nested twin.
 * Both directions go through this function, so #23 (M5.4) removes the bridge by
 * deleting one function and its callers.
 */
inline const char* ContactLogicKeyFromLegacy(const std::string& name) {
  if (name == kLegacyEarlyContactDetectionKey) {
    return contact_logic_params::kEarlyContactDetection;
  }
  if (name == kLegacyLateContactDetectionKey) {
    return contact_logic_params::kLateContactDetection;
  }
  if (name == kLegacyLostContactDetectionKey) {
    return contact_logic_params::kLostContactDetection;
  }
  if (name == kLegacyLateContactRescheduleSwingPhaseKey) {
    return contact_logic_params::kLateContactRescheduleSwingPhase;
  }
  return nullptr;
}

/**
 * `USE_WBC` → the *default* `wbc.type`, nothing more.
 *
 * Before #13 (M3.2) this derivation was load-bearing in a second, hidden way:
 * `WBCInterface` was a class template, so the build flavour also fixed which WBC
 * instantiation — and therefore which plugin base — the host could load at all
 * (gap G8). Selecting the other WBC meant a rebuild. That is gone: both stock
 * WBCs now register under one base and report their command family at runtime,
 * so **an explicit `wbc.type` always wins, on any build**.
 *
 * What survives here is only the default of the declared parameter, for configs
 * that set no `wbc.type` at all — which, in this repository, means the ULab ones
 * (the Go2 configs pinned the key in M2.5). That is the documented transitional
 * limitation for ULab: a ULab launch with no `wbc.type` still gets
 * `inverse_dynamics` from its build flavour rather than from its config. #23
 * (M5.4) deletes this function together with the other legacy derivations, at
 * which point every config must name its stage explicitly.
 */
inline constexpr const char* WBCTypeForBuild(bool use_wbc) {
  return use_wbc ? kWbcArcOptPlugin : kInverseDynamicsPlugin;
}

/**
 * `leg_control_mode` → the WBC command family that mode requires (issue #13, M3.2).
 *
 * The two are independent parameters that must agree: `leg_control_mode` decides
 * which command topic the host publishes on (`leg_joint_cmd` vs `leg_cmd`), and
 * `wbc.type` decides which controller fills it. Pairing them was previously a
 * build-flavour invariant nobody could violate without recompiling; now that the
 * WBC is a launch choice, the host has to check it — once, at bring-up, against
 * the loaded plugin's `SupportedCommandMode()`.
 *
 * Takes the raw parameter value rather than `MITController::LEGControlMode` so
 * the rule can be tested without constructing a node. The enum lives in
 * `mit_controller_node.hpp` and its values are pinned against the cases below by
 * `static_assert`s at the host's validation site.
 *
 * @param leg_control_mode the `leg_control_mode` parameter value
 * @return the required command family, or `std::nullopt` if the value names no
 *         control mode at all — a config error the host reports rather than
 *         casting into an out-of-range enum
 */
inline std::optional<WBCCommandMode> WBCCommandModeForLegControlMode(int64_t leg_control_mode) {
  switch (leg_control_mode) {
    case 0:  // MITController::JOINT_CONTROL
      [[fallthrough]];
    case 1:  // MITController::JOINT_TORQUE_CONTROL
      return WBCCommandMode::kJoint;
    case 2:  // MITController::CARTESIAN_STIFFNESS_CONTROL
      [[fallthrough]];
    case 3:  // MITController::CARTESIAN_JOINT_CONTROL
      return WBCCommandMode::kCartesian;
    default:
      return std::nullopt;
  }
}

/** The `wbc.type` values shipping with each command family, for error messages. */
inline constexpr const char* WBCPluginsForCommandMode(WBCCommandMode mode) {
  return mode == WBCCommandMode::kJoint ? kWbcArcOptPlugin : kInverseDynamicsPlugin;
}

}  // namespace stage_selection
