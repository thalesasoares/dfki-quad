# Stage Plugin Lifecycle

**Status:** specified ahead of M2.2 (issue #7); implemented across M2.1–M2.4 (issues #6–#9). M2.1
landed: `stage_plugin.hpp` and the stage interfaces are exported and the plugin description schema is
defined — see [`plugin_discovery.md`](plugin_discovery.md). M2.2 landed: `StageLoader` drives the
create → `Init` path and the fail-fast errors — see [`stage_loading.md`](stage_loading.md). M2.3
landed: the eight stock wrappers (§4) around GS/MPC/SLC/WBC/MA are built, declared and tested — see
[`stock_plugins.md`](stock_plugins.md). ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`stage_contracts.md`](stage_contracts.md) — the frozen stage APIs ·
[`pipeline_types.md`](pipeline_types.md) — the data they exchange ·
[`plugin_discovery.md`](plugin_discovery.md) — the M2.1 dependency, exported surface and XML schema ·
[`stage_loading.md`](stage_loading.md) — the M2.2 loader that implements this contract

`stage_contracts.md` froze *what* each stage does; this document specifies *how a stage comes to
life* once stages are loaded through `pluginlib` (milestone M2). It exists because the two are in
direct conflict and M2 cannot start until the conflict has a decided resolution:

- `pluginlib::ClassLoader` instantiates a plugin through its **no-argument constructor**.
- Every concrete stage today uses **constructor injection**: `unique_ptr<ModelInterface>` /
  `unique_ptr<StateInterface>` clones plus a long list of typed parameters the host reads from ROS
  (`mit_controller_node.cpp:428-548`), a pattern stage_contracts.md §3 (start-up ordering, point 3)
  codifies.

The resolution is **two-phase initialisation**, layered *on top of* the frozen interfaces so that
none of the five M1 contracts is reopened. The layer is one header,
`mit_controller/stage_plugin.hpp`: a `StageInit` context struct, a `StageInitError` exception, and

```cpp
template <class StageInterface>
class StagePlugin : public StageInterface {
 public:
  virtual void Init(StageInit init) = 0;
};
```

Each stage's pluginlib base class is its instantiation — `StagePlugin<GaitSequencerInterface>`,
`StagePlugin<MPCInterface>`, `StagePlugin<SwingLegControllerInterface>`,
`StagePlugin<ModelAdaptationInterface>`, `StagePlugin<ContactLogicInterface>` (from M3.1), and for
the WBC an instantiation per joint command type (§6). The host keeps owning stages as
`unique_ptr<Interface>` exactly as documented in stage_contracts.md; `Init` is a loader-time
concern only.

## 1. Rules

1. **Changing `StageInit` or the `Init` signature requires updating this file in the same pull
   request** — the same rule stage_contracts.md applies to the five interfaces.
2. A plugin's no-arg constructor must be cheap and infallible. All fallible work — parameter
   parsing, solver setup, model loading — happens in `Init`.
3. `Init` is called **exactly once per instance**, before any method of the stage interface, from
   host initialisation context (never a control loop). An instance is never re-`Init`ed;
   reconfiguration beyond `SetParameter`'s reach means a fresh instance (§5).
4. `Init` reports failure by throwing `StageInitError` with a message that names the offending
   parameter or resource. The host fails the pipeline bring-up loudly; there is no silent fallback
   to a different stage (issue #7's acceptance criterion).
5. Stages still do not touch ROS I/O: no node handle crosses the boundary. Everything a stage needs
   at start-up is in `StageInit`; everything at runtime arrives through the frozen `Update*` /
   `SetParameter` methods.

## 2. The lifecycle

| Phase | Actor | What happens |
|---|---|---|
| create | loader (M2.2) | `pluginlib` resolves the `type:` key and default-constructs the plugin |
| init | host, once | `Init(StageInit)` — the plugin-path equivalent of today's constructor call; throws `StageInitError` on failure |
| run | host loops | the Update*/Get*/SetParameter cycle of stage_contracts.md §4, unchanged |
| destroy | host | deletion through the base pointer (virtual destructors, gap G4); the `ClassLoader` must outlive every instance it created (§5) |

The host calls `Init` at the same point it constructs stages today — **after the first
`/quad_state` has been received** (`mit_controller_node.cpp:419-426`) — so the `model` and `state`
clones in `StageInit` are valid snapshots. The start-up ordering guarantees of stage_contracts.md
§3 (SLC/WBC never called before GS/MPC have produced output; GS/MPC must tolerate a
default-constructed state on early cycles) are unchanged.

## 3. `StageInit`: what a stage receives

| Field | Type | Semantics |
|---|---|---|
| `model` | `unique_ptr<ModelInterface>` | clone, stage takes ownership — same as constructor injection today |
| `state` | `unique_ptr<StateInterface>` | clone, stage takes ownership — valid, first state already received |
| `params` | `map<string, rclcpp::ParameterValue>` | the host node's parameters, flat name → value |

**One key-space, not two.** The `params` keys are the same vocabulary
`SetParameter` handles at runtime (stage_contracts.md §6): `mpc_alpha` in the YAML at start-up and
`ros2 param set … mpc_alpha` at runtime name the same knob. A stage documents its keys once.

**The host passes its full parameter map and stages read what they document.** No prefix filtering:
the existing key names are inconsistent (`mpc_*`, `slc_*`, `wbc.*`, `gait_sequencer`, `raibert.*`,
`fix_*`), and M2.5 — not this contract — owns tightening the naming. A stage must ignore unknown
keys; it must **fail** (`StageInitError`, via `StageInit::Require` or its own validation) on a
missing or mistyped key it cannot start without. Defaults for optional keys live in the stage, not
in the host.

**Type conversion belongs to the stage.** Everything the host factory does today between
`get_parameter` and the constructor call — `GaitDatabase` lookups, the MPC solver-name → enum
mapping, `Eigen::Map` reshaping, length assertions — moves into the wrapper's `Init` (§4). That is
precisely how M2.4's "no concrete algorithm factory if/else left in the host" criterion is met: the
factory bodies don't disappear, they relocate behind the plugin boundary.

**How the keys get declared.** Today the host `declare_parameter`s all 88 keys explicitly. It
cannot pre-declare keys of plugins it has never heard of, so with M2.4 the host node enables
`automatically_declare_parameters_from_overrides(true)`: whatever the selected YAML provides
becomes a parameter, the host harvests all of it into `params`, and validation moves where the
knowledge is — the stage (`Require`). The existing explicit declarations may stay during the
transition; they are compatible.

## 4. Stock plugins wrap, they do not modify (M2.3)

Two ways to make `MPC`, `SimpleGaitSequencer`, etc. satisfy the lifecycle were considered:

- **Option A — retrofit**: add a no-arg constructor and `Init` to each concrete class. Rejected for
  M2: the classes are built around fully-initialised const-style members; retrofitting two-phase
  construction is an invasive refactor of exactly the code M2.3 promises not to change.
- **Option B — thin adapter (chosen)**: one wrapper per stock plugin,
  e.g. `class AcadosMpcPlugin final : public StagePlugin<MPCInterface>`, holding a
  `unique_ptr<MPC>` built inside `Init` and forwarding the interface methods to it. The algorithm
  code is untouched, so "no intentional behavior change" is auditable by reading the wrapper alone.
  Cost: ~7 forwarding methods per stage — accepted.

The wrapper's `Init` body is today's host factory code (`GetGaitSequencerFromParams`, the
`make_unique<MPC>(…)` block, the `create_wbc` lambda), with `get_parameter(x)` mechanically
replaced by `init.Require(x)` / defaulted lookups. Option A remains open per-stage later, without
changing this contract — the lifecycle only sees `StagePlugin<Interface>`.

M2.3 (#8) implemented this for all eight current algorithms — `simple_gait`, `adaptive_gait`,
`acados_mpc`, `bezier_swing`, `wbc_arc_opt`, `inverse_dynamics`, `kf_adaptation`, `rls_adaptation` —
in `src/plugins/`, one library per stage base. The `Init` bodies are the two branches of
`GetGaitSequencerFromParams`, the MPC block, the two branches of `create_wbc`, and the two `ma_mode`
branches respectively; the branch that used to be chosen by a parameter (`gait_sequencer`, `ma_mode`,
`USE_WBC`) is now the plugin selection. Because the wrappers only forward, "no intentional behavior
change" is auditable from the wrapper against the cited host lines. See
[`stock_plugins.md`](stock_plugins.md) for the per-stage parameter tables.

## 5. Threading, reconfiguration, loader lifetime

- `Init` runs with no control loop started (initial bring-up) or off-loop (reconfiguration); it
  never runs under a stage lock and must not assume otherwise-concurrent callbacks are excluded.
- **Reconfiguration = replace, not re-`Init`.** The host's existing pattern for a `SetParameter`
  the stage rejects — rebuild the gait sequencer from scratch and swap it under
  `gait_sequencer_lock_` (`mit_controller_node.cpp:399-408`) — generalises: create a fresh instance
  via the loader, `Init` it off-loop, swap the pointer under the stage's lock, destroy the old
  instance after the swap.
- **The `ClassLoader` outlives its instances.** Destroying a `pluginlib::ClassLoader` unloads the
  library; any surviving instance is then a dangling vtable. The host (M2.4) owns one loader per
  stage base class as a member declared *before* the stage pointers, so destruction order is
  correct by construction. The M2.2 loader helper must preserve this property.

## 6. The WBC instantiations

`WBCInterface` is still a class template (gap G8, issue #13), so there is no single
`StagePlugin<WBCInterface>` — only per-command-type instantiations. For M2 the Go2 product path
registers `StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>` as the WBC plugin base;
the ULab/Cartesian path may stay on the compile-time factory until #13 de-templates the interface,
at which point the WBC plugin XML is reworked once. This keeps M2's exit criterion ("stock **Go2**
sim path runs via plugins only") achievable without pulling #13 forward.

## 7. Guarding

`test/test_stage_contracts.cpp` pins the lifecycle layer the same way it pins the stage contracts:
`static_assert`s that each `StagePlugin<…>` instantiation is an abstract base with a virtual
destructor, a fake plugin driven through create → `Init` → run → destroy, and the
`StageInit::Require` missing-key error path. Any breaking change to the layer fails the ordinary
`colcon build` or `colcon test` — see [`contract_tests.md`](contract_tests.md).

`stage_plugin.hpp` is deliberately self-contained (it includes no stage interface header), so it
can join the exported plugin surface in M2.1 without waiting for the `potato_sim` include cleanup
the interface headers need (pipeline_types.md §6).

## 8. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #6 | [M2.1] pluginlib dependency and plugin description XML | Exports `stage_plugin.hpp`; XML names the `StagePlugin<…>` bases — [`plugin_discovery.md`](plugin_discovery.md) |
| #7 | [M2.2] Stage plugin base + loader helper | **Implements this contract** — `StageLoader` resolves `type:`, default-constructs, calls `Init`, translates `StageInitError` into the fail-fast path; `Create` is the §5 swap path — [`stage_loading.md`](stage_loading.md) |
| #8 | [M2.3] Wrap existing stages as stock plugins | §4 — adapter wrappers whose `Init` bodies are today's host factory code |
| #9 | [M2.4] Thin PipelineHost | §3 declaration strategy, §5 loader lifetime and swap-under-lock |
| #10 | [M2.5] YAML schema for stage selection | Owns the `type:` keys and any parameter renaming; this contract is naming-agnostic |
| #13 | [M3.2] Runtime WBC / command-type profile | §6 — collapses the WBC instantiations into one base |
| #12 | [M3.1] Extract contact FSM | Adds the `StagePlugin<ContactLogicInterface>` base |
