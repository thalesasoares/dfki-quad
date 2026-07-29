# Stock Stage Plugins

**Status:** implemented in M2.3 (issue #8); `default_contact_logic` added in M3.1 (issue #12);
`bio_gait` added in M4.1 (issue #16). ·
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
| `bio_gait` | `BioGaitSequencerPlugin` | `BioGaitSequencer` | `libgait_sequencer_plugins` | `kGaitSequencer` |
| `acados_mpc` | `AcadosMpcPlugin` | `MPC` | `libmpc_plugins` | `kMPC` |
| `bezier_swing` | `BezierSwingPlugin` | `SwingLegController` | `libslc_plugins` | `kSwingLegController` |
| `wbc_arc_opt` | `WbcArcOptPlugin` | `WBCArcOPT` | `libwbc_plugins` | `kWBC` |
| `inverse_dynamics` | `InverseDynamicsPlugin` | `InverseDynamics` | `libwbc_plugins` | `kWBC` |
| `kf_adaptation` | `KfAdaptationPlugin` | `KFModelAdaptation` | `libmodel_adaptation_plugins` | `kModelAdaptation` |
| `rls_adaptation` | `RlsAdaptationPlugin` | `LeastSquaresModelAdaptation` | `libmodel_adaptation_plugins` | `kModelAdaptation` |
| `default_contact_logic` | `DefaultContactLogicPlugin` | `DefaultContactLogic` | `libcontact_logic_plugins` | `kContactLogic` |

`default_contact_logic` joined the table in M3.1 (#12) and is the one entry that wraps code written
*for* the plugin boundary rather than code that predates it: `DefaultContactLogic` is the contact
reconciliation FSM lifted out of `MITController::ControlLoopCallback`, so its `Init` has no host
factory to mirror — see §2 — and its "no behaviour change" claim rests on
`test/test_default_contact_logic.cpp` rather than on the wrapper being a pure forwarder.

`bio_gait` joined in M4.1 (#16), and is the other entry with no host factory behind it — for the
opposite reason to `default_contact_logic`. `BioGaitSequencer` predates the plugin boundary and is
unchanged by #16, but the pre-plugin `GetGaitSequencerFromParams` had no `"Bio"` branch: the class
was compiled and never constructed, so no configuration could reach it. Exporting it is therefore
the first time this repository *adds* a stage implementation through the plugin path alone — one XML
entry, one wrapper, one `gs.type` value, and no host edit — which is what M4 exists to prove.
Correspondingly it has no "no behaviour change" claim to make (there was no reachable behaviour);
what it has is `test/test_stock_plugins.cpp` and the Go2 sim smoke recorded on #16.

The `name=` values are the suggested stock ids; the selection-key vocabulary belongs to M2.5
(#10), so these are naming-agnostic.

**This table is the *stock* set, and it stays that way.** From M4.2 (#17) the workspace also contains
plugins that are selectable at `<stage>.type` but are deliberately **not** stock: `example_passthrough_slc`,
in `ws/src/examples/example_stage_plugins`. The distinction is not bookkeeping. Everything in the
table above ships inside `controllers`, is a production control implementation, and is a defensible
choice for a robot that has to walk. An example plugin is demonstration or diagnostic code — the
passthrough SLC makes the robot *stand* — and lives in its own package precisely so that "what can I
select?" and "what should I select?" do not collapse into one list. Example plugins are documented by
their own package's README ([example_stage_plugins](../../ws/src/examples/example_stage_plugins/README.md)),
and for the same reason they are not added to the `type:` comment lists in the Go2 config YAMLs.

Both WBCs declare the one base `kWBC` (`StagePlugin<WBCInterface>`) in `wbc_plugins.xml` and live in
the one `libwbc_plugins`. Until M3.2 they could not: the interface was a class template (gap G8), so
the joint-command path (`kWBC`) and the ULab/ikin Cartesian path (`kWBCCartesian`) were two unrelated
bases and the build flavour decided which one the host could load. #13 (M3.2) de-templated
`WBCInterface`, moving the command family onto `SupportedCommandMode()`; the wrappers forward both
getters to their implementation, which solves in its own family and stubs the other.

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
| `default_contact_logic` | — see below | — |
| `bio_gait` | — see below | — |

`default_contact_logic` is the exception to the paragraph above, because M3.1 (#12) is an extraction,
not a repackaging: there was no host *factory* to relocate, only host *code*. The two switch
statements of `ControlLoopCallback` became `DefaultContactLogic::Reconcile`, and the four detection
toggles the host read into its own members became the stage's `Init` parameters. Its `Init` is
therefore four `Require<bool>` calls and nothing else.

`bio_gait` (M4.1, #16) is the second exception, and the only `Init` in the table that is genuinely
new code: `GetGaitSequencerFromParams` had no `"Bio"` branch to relocate. It is nevertheless written
as `SimpleGaitSequencerPlugin::Init` minus the `Gait` construction, because the keys it reads are the
*shared* gait-sequencer ones (§3) and those must keep one spelling and one required-vs-defaulted
split across all three wrappers — a `bio_gait` that quietly defaulted `fix_standing_position` while
its neighbours required it would be a trap. The one thing it does not do is read a gait: the
sequencer selects from `BioGaitDatabase` by Froude number internally, so it adds no
`bio_gait_sequencer.*` namespace. The `gs_shoulder_positions` read + length check that all three now
share moved into a `RequireShoulderPositions` helper when this third caller arrived.

## 3. Parameters per stage

Keys are read from `StageInit::params` (the host's full parameter map). **Required** keys throw
`StageInitError` naming the key if absent; **optional** keys fall back to the listed default *in the
stage*. These defaults mirror the host's pre-M2.3 `declare_parameter` defaults exactly — this table is
the audit surface for that, and the seed for M2.5 (#10, YAML schema) and M5.2 (#21, parameter
reference).

### `simple_gait` / `adaptive_gait` / `bio_gait`

The first nine keys are shared by all three; the `*_gait_sequencer.*` blocks below them belong to one
sequencer each. `bio_gait` (M4.1, #16) reads **only** the shared nine — it has no block of its own,
because it picks its gait internally by Froude number.

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

### `default_contact_logic`

| Key | Req? | Default | Notes |
|---|---|---|---|
| `contact_logic.early_contact_detection` | required | — | bool; accept a sensed contact past half a scheduled swing |
| `contact_logic.late_contact_detection` | required | — | bool; react to a scheduled stance that has not touched down |
| `contact_logic.lost_contact_detection` | required | — | bool; react to a stance contact disappearing (slip) |
| `contact_logic.late_contact_reschedule_swing_phase` | required | — | bool; let a late-contact leg follow the plan back into swing without regaining contact |

All four are required rather than optional-with-a-default: the host declares every one of them, so an
absent key means an unconfigured caller, not a value worth guessing. They are also the stage's
`SetParameter` keys — one vocabulary for start-up and runtime
([`plugin_lifecycle.md`](plugin_lifecycle.md) §3) — and are named as constants in
`contact_logic_params` (`mit_controller/contact_logic_interface.hpp`) so the host and the stage cannot
drift apart on the spelling.

The host declares each with the pre-M3.1 **flat** key (`early_contact_detection`, …) as its default,
so configs written before M3.1 keep configuring the same policy untouched, and re-routes a runtime
change of a flat key onto its nested twin. That bridge is host-only
(`stage_selection::ContactLogicKeyFromLegacy`) and goes away with #23 (M5.4).

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
  symbol. `libwbc_plugins` links `fmt::fmt-header-only` for the same reason. M4.2 (#17) found the
  out-of-package form of this rule: a plugin in another package must consume **`controllers`** as an
  include path too, because `ament_target_dependencies(<target> controllers)` puts the whole
  re-exported `common` → `quad_model` → `drake` closure on the link line and fails to configure. Same
  underlying fact in both cases — a stage plugin references no compiled symbol of either package, only
  pure-virtual interfaces and plain data — see [`plugin_discovery.md`](plugin_discovery.md) §3a.
- **Same optimisation as the node** — the libraries inherit the directory-scope `-Ofast` (x86) /
  `-O3` (aarch64), so control-loop code is optimised identically. A `-fPIC`-only Eigen
  `-Wmaybe-uninitialized` false positive (which the non-PIC node never hits) is demoted to non-fatal
  with `-Wno-error=maybe-uninitialized`; every other warning stays a hard error.
- **The algorithm sources live only here** — M2.3 compiled them both into these libraries and into
  `mitcontrollernode`; M2.4 (#9) made the host a thin loader and dropped them from the node, so these
  libraries are now the only place the algorithms are built
  ([`pipeline_host.md`](pipeline_host.md) §1). `bio_gait_sequencer.cpp` was already compiled here
  ahead of having a wrapper, so that M2.4 dropping it from the node would not leave it uncompiled in
  the `WITHOUT_DRAKE` lane; since #16 (M4.1) it is here for the ordinary reason — it is the algorithm
  behind an exported plugin. **This is why #16 changed no build files:** the translation unit, its
  flags and the link line were already in place, so exporting `bio_gait` cost one wrapper class in an
  existing TU and one XML entry.

## 5. What guards it

`test/test_stock_plugins.cpp` loads every stock plugin through production ament discovery. The light
stages (`simple_gait`, `adaptive_gait`, `bio_gait`, `bezier_swing`, `inverse_dynamics`,
`kf_adaptation`, `rls_adaptation`) are fully `Init`ed and driven one interface cycle. `acados_mpc` and `wbc_arc_opt`
are load-only here — a full `Init` sets up the acados / ARC-OPT solver (and `wbc_arc_opt` needs a real
URDF), which belongs to the M2.6 (#11) sim regression, not a fast unit test; their libraries dlopening
and constructing is still the runtime proof that the link story resolves, and their wrapper-specific
logic (the MPC solver-name mapping) is covered by the fail-fast tests. The suite also pins the
fail-fast errors (missing required key, unknown gait/solver name, unknown selection). `bio_gait` adds
one assertion the others do not need: `SetParameter` returns `false` for any key, pinning "no
runtime-tunable parameters" as a tested contract rather than a header comment — the host relies on
that return value to know a runtime parameter change did not apply.
`default_contact_logic` is light too: it needs no solver and no URDF, so the suite `Init`s it and runs
a full `Reconcile` cycle through the frozen interface, and pins its missing-required-key error. Its
*behaviour* is covered separately and far more thoroughly by `test/test_default_contact_logic.cpp`
(19 cases over every branch of the FSM), because unlike the M2.3 wrappers it is not forwarding to
code that already ran in production behind the same interface.
`test/test_plugin_discovery.cpp` and
`test/test_stage_loader.cpp` assert each stage base now declares exactly these stock ids (the flip
from M2.1/M2.2's "empty" — including the contact base, which stayed empty until M3.1). Behavioural equivalence to the pre-plugin path is guaranteed by
construction — the wrappers only forward and the algorithm code is untouched — and is confirmed
end-to-end by the M2.6 (#11) Go2 sim regression.
