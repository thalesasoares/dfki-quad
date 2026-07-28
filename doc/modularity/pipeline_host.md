# The Pipeline Host

**Status:** implemented in M2.4 (issue #9). ·
**Applies to:** `ws/src/controllers` — `mit_controller_node` ·
**Companion documents:** [`stage_contracts.md`](stage_contracts.md) — the frozen stage APIs the host
calls · [`plugin_lifecycle.md`](plugin_lifecycle.md) — how a stage comes to life ·
[`stage_loading.md`](stage_loading.md) — the loader the host owns ·
[`stock_plugins.md`](stock_plugins.md) — the implementations it loads by default

M1 froze the stage interfaces and removed the host's casts into concrete stages. M2.1–M2.3 built the
plugin layer and moved every current algorithm behind it. M2.4 is the consequence: `MITController`
**constructs no algorithm at all**. It owns the ROS interface, the loops, the locks and the lifetime
of six plugins, and it talks to them exclusively through the frozen interfaces.

This is what makes the milestone's exit criterion — "stock Go2 sim path runs via plugins only" —
a property of the build rather than a promise: the algorithm translation units are no longer
compiled into `mitcontrollernode` (§7), so there is nothing concrete left for the host to
instantiate even by accident.

## 1. What the host owns, and what it does not

| Owns | Does not own |
|---|---|
| Subscriptions (`quad_state`, `quad_control_target`), publishers (`leg_cmd` / `leg_joint_cmd` and every diagnostic topic), the `switch_op_mode` client | Any algorithm: gait sequencing, MPC, swing-leg trajectories, WBC, model adaptation |
| The three loop timers and their rates, four mutually exclusive callback groups, the 4-thread `MultiThreadedExecutor` | The parameters those algorithms read — it forwards them (§3) but interprets none |
| The locks that separate the loops, and the double-buffered `GaitSequence` / `WrenchSequence` / `FeetTargets` handoff between them | The choice of *which* implementation runs — that is a parameter (§2) |
| The stage loaders and stage instances (§4) | The `type:` vocabulary itself, which M2.5 (#10) owns |
| The swing/stance PD gain switch, the leg command message assembly, the contact logging and the heartbeat counters | Contact reconciliation — the early / late / lost contact FSM, extracted in M3.1 (#12) |

The PD gain switch is still host code on purpose: M2.4 deliberately did not touch it so that the
"thin host" refactor stayed reviewable separately from #13 (M3.2), which turned the command type into
a runtime choice. #13 has since landed and left the gain switch where it is — it is keyed on
`leg_control_mode` and the reconciled contact state, neither of which belongs to the WBC stage.

The contact FSM *was* host code through M2.4, for the same reason; M3.1 (#12) moved it behind
`ContactLogicInterface`, and the host now loads it as the sixth stage. What stayed behind is the
logging and the `num_early_contacts` counter, driven off the stage's `ContactEvents` — a stage should
not need a logger or a message type, and keeping those here is what lets it be plain algorithm code.

## 2. Selecting a stage

Six string parameters, one per stage — the vocabulary proposed in
[`stage_loading.md`](stage_loading.md) §4:

| Stage | Key | Stock default | Base (`stage_plugin_bases::`) |
|---|---|---|---|
| gait sequencer | `gs.type` | `simple_gait` / `adaptive_gait` | `kGaitSequencer` |
| MPC | `mpc.type` | `acados_mpc` | `kMPC` |
| swing leg controller | `slc.type` | `bezier_swing` | `kSwingLegController` |
| WBC | `wbc.type` | `wbc_arc_opt` (joint commands) / `inverse_dynamics` (Cartesian) | `kWBC` |
| model adaptation | `model_adaptation.type` | `kf_adaptation` / `rls_adaptation` | `kModelAdaptation` |
| contact logic | `contact_logic.type` | `default_contact_logic` | `kContactLogic` |

A value that names no declared plugin is a **fatal bring-up error listing what is declared**, never a
fallback ([`stage_loading.md`](stage_loading.md) §1). `main` catches the exception, logs it with
`RCLCPP_FATAL` and exits non-zero, which replaces the mix of "log and `rclcpp::shutdown()`" and "log
and `exit(-1)`" the deleted factories used.

### The legacy bridge (temporary)

M2.4 landed before #10 wrote these keys into the Go2 YAMLs, so each key's **default** is derived from
the parameter that used to select the implementation inside the host factory
(`include/stage_selection.hpp`):

| Key | Derived from |
|---|---|
| `gs.type` | `gait_sequencer`: `Simple` → `simple_gait`, `Adaptive` → `adaptive_gait`, anything else passed through verbatim |
| `model_adaptation.type` | `ma_mode`: `1` → `rls_adaptation`, anything else → `kf_adaptation` (the old `switch`'s `default:`) |
| `wbc.type` | `USE_WBC`, i.e. the `ROBOT_MODEL` build flavour. Since #13 (M3.2) this supplies a *default only* — it no longer restricts which WBC the build can load, and all four shipped configs now pin the key explicitly |
| `mpc.type`, `slc.type` | constant — one stock implementation each |

So the shipped YAMLs and launch files keep working untouched, and an explicitly set `*.type` always
wins. Unrecognised legacy values are **not** corrected: passing them through unchanged is what turns
a typo into the loader's fail-fast error instead of a silent substitution. The derivation is
string → string and never names a C++ type, which is why it is not the concrete-algorithm `if/else`
issue #9 removed. `test/test_stage_selection.cpp` pins both the mapping and the fact that every
default it produces names a declared plugin.

**A runtime set of `gait_sequencer` re-derives `gs.type`** and reloads the sequencer, not only the
declare-time default. That was required while `scripts/joy_to_target.py` switched sequencers through
the legacy key: without it, the reload below would resolve the *startup* value of `gs.type` and
rebuild the sequencer that is already running.

**Since M2.5 (#10) the bridge is inert on the stock Go2 path.** Both Go2 YAMLs now carry all six
`*.type` keys explicitly, and `joy_to_target.py` sets `gs.type`, so nothing in this repository reads
or writes the legacy spelling any more — the derivation and the runtime re-derivation are dead weight
kept for the ULab configs (which still select through the declared defaults) and for out-of-tree
configs. That is exactly the precondition #23 (M5.4) needs to delete both.

## 3. `MakeStageInit`

Every load passes a fresh `StageInit` ([`plugin_lifecycle.md`](plugin_lifecycle.md) §3):

- a `QuadModelPino` clone and a `QuadState` clone the stage takes ownership of — the same
  constructor injection the concrete stages always had;
- the node's **entire** parameter set, flattened to `name → rclcpp::ParameterValue`.

Handing over the whole map is what keeps the host out of the business of knowing which parameter
belongs to which algorithm: a stage reads the keys it documents and ignores the rest, and defaults
for absent keys live in the stage. The host still *declares* all parameters, because rclcpp only
surfaces YAML overrides for declared names and the declared types are what reject a `200` written
where a `200.0` is meant; slimming that surface belongs with #10/#21.

The map is built at bring-up and on reconfiguration only — never from a loop.

## 4. Loader lifetime and declaration order

```cpp
StageLoader<GaitSequencerInterface> gs_loader_{stage_plugin_bases::kGaitSequencer};  // declared first
...
StageLoader<GaitSequencerInterface>::PluginPtr gs_;                                  // destroyed first
```

Destroying a `pluginlib::ClassLoader` unloads the library, and a stage instance that outlives its
loader is a dangling vtable. Members are destroyed in reverse declaration order, so **all six
loaders are declared before all six stage pointers** and the ordering is correct by construction;
`StageLoader` is additionally non-movable so it cannot be pulled out from under its instances
([`plugin_lifecycle.md`](plugin_lifecycle.md) §5).

The stage members are `StageLoader<I>::PluginPtr` (pluginlib's `UniquePtr`), not
`std::unique_ptr<I>`: the deleter is part of the pointer type, and substituting the default one would
destroy the stage outside pluginlib's bookkeeping. Nothing about the *call sites* changes —
`StagePlugin<I>` is an `I`.

Bring-up order is unchanged from before the refactor: declare parameters → build the ROS interface →
wait for the first `/quad_state` → create the stages → switch the leg driver mode → start the timers.
The stages are still created after the first state message, which is what makes the model and state
clones valid snapshots.

## 5. Reconfiguration

Parameter changes are routed to the owning stage's `SetParameter`, unchanged since M1.2. The one path
that used to call a factory — a gait parameter no stage accepted, which forced a rebuild — is now the
swap of [`plugin_lifecycle.md`](plugin_lifecycle.md) §5:

1. `Load` a fresh instance **off-loop** (resolve, create, `Init` — the expensive part, done without
   holding the stage lock);
2. `UpdateTarget` it, as at bring-up;
3. swap the pointer under `gait_sequencer_lock_`;
4. destroy the old instance after the lock is released.

Failure here is *not* fatal, unlike bring-up: the running stage is kept and the error is logged. A
walking robot must not fall because a parameter update was malformed.

## 6. Performance

The refactor is startup-cost only. Six `dlopen`s, six parameter-map copies and the model/state
clones all happen before any timer exists. The loop bodies are unchanged — same rates
(`MPC_CONTROL_DT` 100 Hz, `SWING_LEG_DT` / `CONTROL_DT` 500 Hz, `MODEL_ADAPTATION_DT` 100 Hz), same
callback groups, same locking, same double buffering, no added allocation.

Per-cycle cost changes by exactly one thing: a call now goes through the stock wrapper's forwarding
method before reaching the algorithm, i.e. one extra non-inlinable virtual call on top of the virtual
call that already existed through the interface pointer. Against loops whose work is a multi-millisecond
QP solve this is unmeasurable. The plugin libraries are compiled with the same directory-scope Release
flags as the node (`-Ofast` on x86, `-O3` on aarch64), so no stage is quietly less optimised than it
was when linked in.

## 7. Guarding

- `test/test_stage_selection.cpp` — the legacy derivation and "every host default is a declared
  plugin".
- `test/test_stock_plugins.cpp` (M2.3) — the defaults actually load, `Init` and fail loudly.
- `test/test_stage_loader.cpp` (M2.2) — the fail-fast contract the host relies on.
- The build itself — the algorithm sources are no longer part of `mitcontrollernode`, so a
  reintroduced `make_unique<Concrete>` in the host fails to link.

`#11` (M2.6) is the human acceptance gate: cold start, stand and trot from the joystick with
plugin-loaded stages.

```bash
colcon build --packages-up-to controllers --cmake-args -DROBOT_NAME=go2
colcon test  --packages-select controllers --ctest-args -R "test_stage_selection|test_stock_plugins"
colcon test-result --verbose
```

## 8. Known gaps left open

[`stage_contracts.md`](stage_contracts.md) §7 assigns two gaps to this issue that M2.4 does **not**
close:

- **G6 — inconsistent lock discipline.** `ModelAdaptationCallback` mutates `quad_model_` unlocked,
  and `gs_->UpdateModel` runs under `mpc_lock_` rather than `gait_sequencer_lock_`.
- **G10 — unsynchronised diagnostic reads.** `gs_->GetGaitState()` and `slc_->GetCurrentTrajs()` run
  after their stage locks have been released.

Both are pre-existing data races, and both are fixed by changing *which lock is held where inside the
control loops*. M2.4's whole reviewability argument — and its no-regression claim — rests on the loop
bodies being untouched, so mixing a lock-discipline change into it would make the diff impossible to
read as "same behaviour, different owner of construction". They also overlap the code #12 (M3.1)
moved and #15 (M3.4) reworks (the model update broadcast), which is where the locking wants to be
decided once. They should be a change of their own, on top of this one. M3.1 did not change the
lock discipline either: the contact stage runs under `wbc_lock_`, where the FSM already ran.

**Update after M3.4 (issue #15).** The broadcast rework has landed, and the place this paragraph
asked for now exists: the model fan-out's lock choices are five `Register` lines in the constructor
([`model_update_broadcast.md`](model_update_broadcast.md) §4), so G6's `gs_->UpdateModel` half is a
one-argument edit — `mpc_lock_` → `gait_sequencer_lock_` — rather than surgery inside
`ModelAdaptationCallback`. M3.4 deliberately did **not** make that edit, for the same reason M2.4
and M3.1 did not: it preserved the existing grouping exactly so the refactor stays reviewable as
behaviour-preserving. Both gaps remain open, and the rest of G6 — the unlocked `quad_model_`
mutation — and all of G10 are still in the loop bodies.

## 9. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #7 | [M2.2] Stage plugin base + loader helper | Provides `StageLoader` — [`stage_loading.md`](stage_loading.md) |
| #8 | [M2.3] Wrap existing stages as stock plugins | Provides the implementations the host defaults to — [`stock_plugins.md`](stock_plugins.md) |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | **This document** |
| #10 | [M2.5] YAML schema for stage selection | Wrote the `*.type` keys into the Go2 YAMLs and moved the joystick onto `gs.type`; no host behaviour change (§2) |
| #11 | [M2.6] Go2 sim regression | Acceptance gate for the pipeline this host builds |
| #12 | [M3.1] Extract contact FSM | **Done.** Moved the contact reconciliation out of `ControlLoopCallback` into the `default_contact_logic` stage (§1) |
| #13 | [M3.2] Runtime WBC / command-type profile | **Done.** Deleted `WBCType` / `WBC_PLUGIN_BASE`, collapsed the two WBC bases (§2), and added the bring-up `wbc.type` / `leg_control_mode` compatibility check |
| #16 | [M4.1] Bio gait sequencer via plugin param | **Done, and the prediction held: this file was not touched.** `bio_gait` is a `gs.type` value like any other — bring-up load and the runtime rebuild-and-swap are class-name-generic, so adding a stage implementation needed no host edit |
| #23 | [M5.4] Deprecate monolithic factory paths | Removes the legacy `*.type` derivation (§2) |
