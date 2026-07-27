#pragma once

#include <cstdint>
#include <string>

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
 * M2.4 lands before the YAML schema work of #10 (M2.5), so the shipped
 * `mit_controller_{sim,real}_go2.yaml` do not carry `*.type` keys yet. To keep
 * "launch still works, unchanged" true across this refactor, the host *declares*
 * each `*.type` parameter with a default derived from the parameter that used to
 * pick the implementation inside the deleted host factories:
 *
 * | Key | Legacy source |
 * |---|---|
 * | `gs.type` | `gait_sequencer` ("Simple" / "Adaptive") |
 * | `mpc.type` | — (one stock MPC) |
 * | `slc.type` | — (one stock SLC) |
 * | `wbc.type` | `USE_WBC`, i.e. the `ROBOT_MODEL` build flavour |
 * | `model_adaptation.type` | `ma_mode` (0 = Kalman filter, 1 = recursive least squares) |
 *
 * An explicitly set `*.type` always wins — the derivation only supplies the
 * *default* of the declared parameter. Once M2.5 writes the keys into the YAMLs
 * the derivation becomes dead weight rather than a behaviour, and #23 (M5.4)
 * deletes it together with the legacy keys.
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

/** The legacy parameters the defaults above are derived from. */
inline constexpr char kLegacyGaitSequencerKey[] = "gait_sequencer";
inline constexpr char kLegacyModelAdaptationModeKey[] = "ma_mode";

/** Stock plugin class names, as declared in the `plugins/` XMLs (M2.3, issue #8). */
inline constexpr char kSimpleGaitPlugin[] = "simple_gait";
inline constexpr char kAdaptiveGaitPlugin[] = "adaptive_gait";
inline constexpr char kAcadosMPCPlugin[] = "acados_mpc";
inline constexpr char kBezierSwingPlugin[] = "bezier_swing";
inline constexpr char kWbcArcOptPlugin[] = "wbc_arc_opt";
inline constexpr char kInverseDynamicsPlugin[] = "inverse_dynamics";
inline constexpr char kKfAdaptationPlugin[] = "kf_adaptation";
inline constexpr char kRlsAdaptationPlugin[] = "rls_adaptation";

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
 * `USE_WBC` → WBC plugin class. The command type of the WBC is still fixed at
 * compile time (`WBCInterface` is a template, gap G8 / issue #13), so the
 * default must follow the build flavour: ARC-OPT with joint commands for Go2,
 * inverse dynamics with cartesian commands for ULab.
 */
inline constexpr const char* WBCTypeForBuild(bool use_wbc) {
  return use_wbc ? kWbcArcOptPlugin : kInverseDynamicsPlugin;
}

}  // namespace stage_selection
