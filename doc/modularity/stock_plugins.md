# Stock Stage Plugins

**Status:** implemented in M2.3 (issue #8). ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`plugin_lifecycle.md`](plugin_lifecycle.md) — the wrapper contract (§4) ·
[`plugin_discovery.md`](plugin_discovery.md) — the description XML schema ·
[`stage_loading.md`](stage_loading.md) — the loader that instantiates these ·
[`stage_contracts.md`](stage_contracts.md) — the frozen stage APIs the wrappers forward

M2.3 exports every current control algorithm as a `pluginlib` stage plugin, with **no intentional
behaviour change** (issue #8 acceptance criterion 3). Each plugin is a thin adapter
([`plugin_lifecycle.md`](plugin_lifecycle.md) §4, Option B): a `final` class deriving from
`StagePlugin<Interface>` that holds a `unique_ptr` to the untouched algorithm class, builds it inside
`Init` from the `StageInit` context, and forwards the frozen interface methods. The algorithm sources
(`src/mit_controller/*.cpp`, `src/model_adaptation/*.cpp`) are unchanged; the wrappers live in
`src/plugins/`.

## 1. The stock plugins

| Stock id (`name=`) | Wrapper class | Wraps | Library | Base (`stage_plugin_bases::`) |
|---|---|---|---|---|
| `simple_gait` | `SimpleGaitSequencerPlugin` | `SimpleGaitSequencer` | `libgait_sequencer_plugins` | `kGaitSequencer` |
| `adaptive_gait` | `AdaptiveGaitSequencerPlugin` | `AdaptiveGaitSequencer` | `libgait_sequencer_plugins` | `kGaitSequencer` |
| `acados_mpc` | `AcadosMpcPlugin` | `MPC` | `libmpc_plugins` | `kMPC` |
| `bezier_swing` | `BezierSwingPlugin` | `SwingLegController` | `libslc_plugins` | `kSwingLegController` |
| `wbc_arc_opt` | `WbcArcOptPlugin` | `WBCArcOPT` | `libwbc_plugins` | `kWBC` |
| `inverse_dynamics` | `InverseDynamicsPlugin` | `InverseDynamics` | `libwbc_plugins` | `kWBCCartesian` |
| `kf_adaptation` | `KfAdaptationPlugin` | `KFModelAdaptation` | `libmodel_adaptation_plugins` | `kModelAdaptation` |
| `rls_adaptation` | `RlsAdaptationPlugin` | `LeastSquaresModelAdaptation` | `libmodel_adaptation_plugins` | `kModelAdaptation` |

`bio_gait` (`BioInspiredGait`) is deliberately **not** exported here — issue #8 defers it to M4.1
(#16). The `name=` values are the suggested stock ids; the selection-key vocabulary belongs to M2.5
(#10), so these are naming-agnostic.

Two `WBCInterface` instantiations exist because the interface is still a template (gap G8): the Go2
joint-command path (`kWBC`) and the ULab/ikin Cartesian path (`kWBCCartesian`). Both classes live in
the one `libwbc_plugins` and are declared under their respective base strings in `wbc_plugins.xml`.
#13 (M3.2) collapses them into one when `WBCInterface` is de-templated.

## 2. Where each `Init` came from

Every `Init` body is a verbatim relocation of the host factory code, with `get_parameter(x).as_T()`
replaced by `Require<T>` (required key) / `GetOr<T>` (optional key, default in the stage). The
branch the host used to pick with a parameter is now the plugin selection:

| Plugin | Host source (pre-M2.3 `mit_controller_node.cpp`) | Selector that became the plugin choice |
|---|---|---|
| `simple_gait` / `adaptive_gait` | `GetGaitSequencerFromParams`, the `"Simple"` / `"Adaptive"` branches | `gait_sequencer` |
| `acados_mpc` | the state-weight reshape + solver-name map + `make_unique<MPC>` block | — |
| `bezier_swing` | the `make_unique<SwingLegController>` block | — |
| `wbc_arc_opt` / `inverse_dynamics` | the `create_wbc` lambda's two branches | `USE_WBC` (compile-time) |
| `kf_adaptation` / `rls_adaptation` | the `ma_mode` `switch`, default / case 1 | `ma_mode` |

## 3. Parameters per stage

Keys are read from `StageInit::params` (the host's full parameter map). **Required** keys throw
`StageInitError` naming the key if absent; **optional** keys fall back to the listed default *in the
stage*. These defaults mirror the host's pre-M2.3 `declare_parameter` defaults exactly — this table is
the audit surface for that, and the seed for M2.5 (#10, YAML schema) and M5.2 (#21, parameter
reference).

### `simple_gait` / `adaptive_gait`

| Key | Req? | Default | Notes |
|---|---|---|---|
| `gs_shoulder_positions` | required | — | length `N_LEGS*3` (12) |
| `fix_standing_position` | required | — | bool |
| `early_contact_detection` | required | — | bool |
| `raibert.k` | optional | `0.03` | |
| `raibert.filtersize` | optional | `20` | |
| `raibert.z_on_plane` | optional | `false` | |
| `fix_position_distance_threshold` | optional | `0.1` | |
| `fix_position_angular_threshold` | optional | `0.26` | |
| `fix_position_velocity_threshold` | optional | `0.1` | |
| `simple_gait_sequencer.gait` | optional | `"STAND"` | `simple_gait` only; a gait-database name or `Manual`; unknown → `StageInitError` |
| `simple_gait_sequencer.manual_gait.period` | optional | `0.5` | `simple_gait`, `Manual` only |
| `simple_gait_sequencer.manual_gait.duty_factor` | optional | `{0.6,0.6,0.6,0.6}` | `simple_gait`, `Manual` only |
| `simple_gait_sequencer.manual_gait.phase_offset` | optional | `{0.0,0.5,0.5,0.0}` | `simple_gait`, `Manual` only |
| `adaptive_gait_sequencer.gait.*` | optional | see `mit_controller_node.cpp` | `adaptive_gait` only; `phase_offset` default `{0.0,0.5,0.5,0.0}`, `gait_change_froude` `{0.02,0.006}`, plus the scalar `swing_time`/`filter_size`/… defaults |

### `acados_mpc`

| Key | Req? | Default |
|---|---|---|
| `mpc_alpha`, `mpc_mu`, `mpc_fmin`, `mpc_fmax` | required | — |
| `mpc_state_weights_stand`, `mpc_state_weights_move` | required | — (length `MPC::STATE_SIZE-1` = 12) |
| `mpc_solver` | optional | `"PARTIAL_CONDENSING_HPIPM"` (unknown → `StageInitError`) |
| `mpc_condensed_size` | optional | `MPC_PREDICTION_HORIZON/2` |
| `mpc_hpipm_mode` | optional | `"SPEED"` |
| `mpc_warm_start` | optional | `1` |
| `mpc_solver_tolerances` | optional | `-1.0` |
| `mpc_osqp_linsys_solver` | optional | `"qdldl"` |

### `bezier_swing`

| Key | Req? | Default |
|---|---|---|
| `slc_swing_height` | required | — |
| `maximum_swing_leg_progress_to_update_target` | required | — |
| `slc_world_blend` | optional | `1.0` |

### `wbc_arc_opt`

| Key | Req? | Default |
|---|---|---|
| `wbc.arc_opt.model_urdf` | required | — |
| `wbc.arc_opt.feet_names` | required | — (length `N_LEGS`) |
| `wbc.arc_opt.joint_names` | required | — (length `NUM_JOINTS`) |
| `wbc.arc_opt.mu` | required | — |
| `wbc.arc_opt.com_pose_weight` | required | — (6) |
| `wbc.arc_opt.foot_pose_weight`, `foot_force_weight` | required | — (3 each) |
| `wbc.arc_opt.com_pose_Kp`, `com_pose_Kd` | required | — (6 each) |
| `wbc.arc_opt.feet_pose_Kp`, `feet_pose_Kd` | required | — (3 each) |
| `wbc.arc_opt.com_pose_saturation` | optional | `{max}*6` |
| `wbc.arc_opt.feet_pose_saturation` | optional | `{max}*3` |
| `wbc.arc_opt.solver` | optional | `"EiquadprogSolver"` |
| `wbc.arc_opt.scene` | optional | `"AccelerationSceneReducedTSID"` |
| `wbc_solver_tolerances` | optional | `-1.0` |

### `inverse_dynamics`

| Key | Req? | Default |
|---|---|---|
| `wbc.inverse_dynamics.foot_position_based_on_target_height` | required | — |
| `wbc.inverse_dynamics.foot_position_based_on_target_orientation` | required | — |
| `wbc.inverse_dynamics.transformation_filter_size` | optional | `20` |
| `wbc.inverse_dynamics.target_velocity_blend` | optional | `0.0` |

### `kf_adaptation`

| Key | Req? | Default |
|---|---|---|
| `ma_convergence_threshold` | optional | `{0.695, 0.12, 0.11}` |
| `ma_process_noise` | optional | `{0.005, 0.0005, 0.0005}` |
| `ma_measurement_noise` | optional | `{1000, 1000, 10000, 10000, 10000, 1000}` |

Gravity is hard-coded `9.81`, and the process/measurement noise matrices are identity with the
parameter vector on the diagonal — preserved exactly from the host.

### `rls_adaptation`

| Key | Req? | Default |
|---|---|---|
| `ma_forgetting_factor` | optional | `0.5` |

The convergence threshold is passed as a zero vector, exactly as the host did
(`conv_thresh.setZero()`, "Estimation covariance is not thresholdable here").

## 4. Build and packaging

Each library is `add_library(<family>_plugins SHARED …)` in `CMakeLists.txt`, compiling its wrapper
plus the algorithm sources it wraps, installed to `lib/`. Notes that matter for correctness and
performance:

- **`common` is include-only, never linked** — linking it would drag the non-PIC `libfmt.a` closure
  into a shared object ([`stage_loading.md`](stage_loading.md) §6). The algorithm code uses only
  `common`'s abstract interfaces and header-only helpers, so it references no compiled `common`
  symbol. `libwbc_plugins` links `fmt::fmt-header-only` for the same reason.
- **Same optimisation as the node** — the libraries inherit the directory-scope `-Ofast` (x86) /
  `-O3` (aarch64), so control-loop code is optimised identically. A `-fPIC`-only Eigen
  `-Wmaybe-uninitialized` false positive (which the non-PIC node never hits) is demoted to non-fatal
  with `-Wno-error=maybe-uninitialized`; every other warning stays a hard error.
- **The algorithm sources live only here** — M2.3 compiled them both into these libraries and into
  `mitcontrollernode`; M2.4 (#9) made the host a thin loader and dropped them from the node, so these
  libraries are now the only place the algorithms are built
  ([`pipeline_host.md`](pipeline_host.md) §1). `bio_gait_sequencer.cpp` is compiled here too — it has
  no wrapper until #16 (M4.1), but the target that used to compile it is skipped in the
  `WITHOUT_DRAKE` lane.

## 5. What guards it

`test/test_stock_plugins.cpp` loads every stock plugin through production ament discovery. The light
stages (`simple_gait`, `adaptive_gait`, `bezier_swing`, `inverse_dynamics`, `kf_adaptation`,
`rls_adaptation`) are fully `Init`ed and driven one interface cycle. `acados_mpc` and `wbc_arc_opt`
are load-only here — a full `Init` sets up the acados / ARC-OPT solver (and `wbc_arc_opt` needs a real
URDF), which belongs to the M2.6 (#11) sim regression, not a fast unit test; their libraries dlopening
and constructing is still the runtime proof that the link story resolves, and their wrapper-specific
logic (the MPC solver-name mapping) is covered by the fail-fast tests. The suite also pins the
fail-fast errors (missing required key, unknown gait/solver name, unknown selection).
`test/test_plugin_discovery.cpp` and
`test/test_stage_loader.cpp` assert each stage base now declares exactly these stock ids (the flip
from M2.1/M2.2's "empty"). Behavioural equivalence to the pre-plugin path is guaranteed by
construction — the wrappers only forward and the algorithm code is untouched — and is confirmed
end-to-end by the M2.6 (#11) Go2 sim regression.
