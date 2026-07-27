#pragma once

#include <cstdint>
#include <string>

#include "mit_controller/contact_logic_interface.hpp"

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
 * | `wbc.type` | `USE_WBC`, i.e. the `ROBOT_MODEL` build flavour |
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
 * `USE_WBC` → WBC plugin class. The command type of the WBC is still fixed at
 * compile time (`WBCInterface` is a template, gap G8 / issue #13), so the
 * default must follow the build flavour: ARC-OPT with joint commands for Go2,
 * inverse dynamics with cartesian commands for ULab.
 */
inline constexpr const char* WBCTypeForBuild(bool use_wbc) {
  return use_wbc ? kWbcArcOptPlugin : kInverseDynamicsPlugin;
}

}  // namespace stage_selection
