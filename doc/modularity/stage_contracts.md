# Control Pipeline Stage Contracts

**Status:** frozen as of M1.1 (issue #1) · **Applies to:** `ws/src/controllers`

This document is the reference contract for the five control-pipeline stages that
`mit_controller_node` hosts. It records, for each stage, the methods an implementation **must**
provide, the data that flows in and out, the rate the host calls it at, and what the host assumes
about threading and locking.

## 1. Purpose and how to use this document

The controllers package is being refactored from a monolithic node into a pluginlib-based pipeline
host (meta issue #24). Loading a stage as a plugin is only possible if the host talks to it purely
through an abstract interface. Today it does not: the host reaches through the interfaces into
concrete algorithm classes in 18 places. This document freezes what the contracts are now, states
where the host violates them, and proposes the API changes that follow-up issues will implement.

Rules going forward:

- **Changing a pure-virtual signature in any of the five headers requires updating this file in the
  same pull request.** The method tables in §4 are the specification; the headers are the
  implementation of that specification.
- New stage implementations are written against §4, not against an existing concrete class.
- Anything listed in §5 is a known deviation with an owning issue. Do not add new ones.

## 2. The five stages

| Stage | Interface header | Concrete implementations |
|---|---|---|
| Gait sequencing | `mit_controller/gait_sequencer_interface.hpp` | `SimpleGaitSequencer`, `AdaptiveGaitSequencer`, `BioGaitSequencer` |
| Force optimisation | `mit_controller/mpc_interface.hpp` | `MPC` (acados QP) |
| Swing trajectories | `mit_controller/swing_leg_controller_interface.hpp` | `SwingLegController` |
| Whole-body control | `mit_controller/wbc_interface.hpp` | `WBCArcOPT` (Go2), `InverseDynamics` (ULab) |
| Model adaptation | `model_adaptation/model_adaptation_interface.hpp` | `KFModelAdaptation`, `LeastSquaresModelAdaptation` |

```mermaid
flowchart LR
    T["Target<br/>(/quad_control_target)"] --> GS
    S["StateInterface<br/>(/quad_state)"] --> GS & MPC & SLC & WBC & MA
    GS["GaitSequencer<br/>100 Hz"] -->|GaitSequence| MPC["MPC<br/>100 Hz"]
    GS -->|GaitSequence| SLC["SwingLegController<br/>500 Hz"]
    GS -->|GaitSequence| MA
    MPC -->|WrenchSequence<br/>MPCPrediction| WBC["WBC<br/>500 Hz"]
    SLC -->|FeetTargets| WBC
    WBC -->|JointCommandType| OUT["/leg_cmd<br/>/leg_joint_cmd"]
    MA["ModelAdaptation<br/>100 Hz"] -.->|ModelInterface| GS & MPC & SLC & WBC
```

Solid arrows are per-cycle data flow. The dashed arrow is the model-update broadcast, which is
event-driven rather than periodic (§4.5).

## 3. Timing and threading

All periods are `static constexpr` in `mit_controller/mit_controller_params.hpp`. `main()` runs a
`rclcpp::executors::MultiThreadedExecutor` with **4 threads**; each loop below owns a **mutually
exclusive callback group**, so a given stage is never re-entered concurrently, but different stages
do run concurrently on different threads.

| Loop | Callback | Constant | Period | Rate | Callback group | Mutex held around stage calls |
|---|---|---|---|---|---|---|
| MPC | `MPCLoopCallback` | `MPC_CONTROL_DT` | 10 ms | 100 Hz | `mpc_call_back_group_` | `mpc_lock_` + `gait_sequencer_lock_` |
| Swing leg | `SLCLoopCallback` | `SWING_LEG_DT` | 2 ms | 500 Hz | `slc_callback_group_` | `slc_lock_` |
| Control / WBC | `ControlLoopCallback` | `CONTROL_DT` | 2 ms | 500 Hz | `control_loop_call_back_group_` | `wbc_lock_` |
| Model adaptation | `ModelAdaptationCallback` | `MODEL_ADAPTATION_DT` | 10 ms | 100 Hz | `model_adaptation_callback_group_` | none — see G6 |
| Heartbeat | `HartbeatCallback` | — | 500 ms | 2 Hz | default | none |

Other shared constants a stage implementation may rely on: `N_LEGS = 4`,
`N_JOINTS_PER_LEG = 3`, `GAIT_SEQUENCE_SIZE = 100`, `MPC_PREDICTION_HORIZON = 10`,
`MPC_DT = 50 ms`. The gait sequence therefore spans 100 × 50 ms = 5 s of plan, and the MPC horizon
covers 10 × 50 ms = 500 ms.

### Start-up ordering

Only the MPC and model-adaptation timers are created in the constructor. The **SLC and control-loop
timers are created lazily inside the first `MPCLoopCallback`**
(`mit_controller_node.cpp:866-880`). Consequences for any implementation:

1. `SwingLegControllerInterface` and `WBCInterface` are never called before the gait sequencer and
   MPC have each produced one output. They may assume a valid `GaitSequence` /
   `WrenchSequence` exists on the first call.
2. `GaitSequencerInterface` and `MPCInterface` **are** called before any state has been received;
   they must tolerate a default-constructed `QuadState` on their first cycles.
3. Constructors receive `std::unique_ptr<ModelInterface>` and `std::unique_ptr<StateInterface>`
   clones and take ownership of them. Every stage therefore needs a working destructor — see G4.

### General call-order rule

Within one cycle the host always calls every relevant `Update*` method **before** the `Get*` method.
An implementation may cache in `Update*` and compute in `Get*`, or compute eagerly; the contract
only guarantees the ordering. No `Get*` method may assume it is called exactly once per `Update*`
(§4.1 and §4.3 both have `Get*` methods invoked an extra time for diagnostics).

## 4. Stage contracts

### 4.1 `GaitSequencerInterface`

Header: `ws/src/controllers/include/mit_controller/gait_sequencer_interface.hpp`

| Method | Dir | Called from | Rate |
|---|---|---|---|
| `void UpdateTarget(const Target&)` | in | `MITController` ctor (`:555`), `MPCLoopCallback` (`:813`) | 100 Hz |
| `void UpdateState(const StateInterface&)` | in | `MPCLoopCallback` (`:814`) | 100 Hz |
| `void UpdateModel(const ModelInterface&)` | in | `ModelAdaptationCallback` (`:1014`) | event-driven |
| `void GetGaitSequence(GaitSequence&)` | out | `MPCLoopCallback` (`:817`) | 100 Hz |
| `void GetGaitState(interfaces::msg::GaitState&)` | out | `MPCLoopCallback` (`:959`), diagnostics only | 100 Hz |
| `GS_Type GetType() const` | out | — (currently unused by the host) | — |

**Data.** In: `Target` (`mit_controller/target.hpp`) — world/hybrid-frame pose and twist setpoints,
each with an `active` flag. Out: `GaitSequence` (`mit_controller/gait_sequence.hpp`) — a
`GAIT_SEQUENCE_SIZE`-long plan holding `contact_sequence`, `foot_position_sequence`,
`reference_trajectory_*`, `swing_time_sequence`, and `sequence_mode ∈ {KEEP, MOVE}`.
`GS_Type` is `{SIMPLE, ADAPTIVE, BIOINSPIRED}` (`gait_sequencer_types.hpp`).

**Call order.** `UpdateTarget` → `UpdateState` → `GetGaitSequence`. `GetGaitState` is called later in
the same callback for publishing.

**Threading.** `gait_sequencer_lock_` is held across `:813-817`. It is *not* held for the
`GetGaitState` call at `:959` (G10) nor for the `UpdateModel` call at `:1014`, which runs under
`mpc_lock_` instead (G6). Implementations must be safe against `GetGaitSequence` blocking the 100 Hz
loop; no internal blocking beyond the loop period.

**Notes.** `sequence_mode` transitions drive the MPC weight switch in the host (G1). The
`GetGaitState` output is diagnostic (`/gait_state`), gated on `PUBLISH_GAIT_STATE`.

### 4.2 `MPCInterface`

Header: `ws/src/controllers/include/mit_controller/mpc_interface.hpp`

| Method | Dir | Called from | Rate |
|---|---|---|---|
| `void UpdateState(const StateInterface&)` | in | `MPCLoopCallback` (`:815`) | 100 Hz |
| `void UpdateModel(const ModelInterface&)` | in | `ModelAdaptationCallback` (`:1013`) | event-driven |
| `void UpdateGaitSequence(const GaitSequence&)` | in | `MPCLoopCallback` (`:818`) | 100 Hz |
| `void GetWrenchSequence(WrenchSequence&, MPCPrediction&, SolverInformation&)` | out | `MPCLoopCallback` (`:844`) | 100 Hz |

**Data.** Out: `WrenchSequence` — `MPC_PREDICTION_HORIZON × N_LEGS` ground-reaction forces;
`MPCPrediction` — predicted pose/twist over `MPC_PREDICTION_HORIZON + 1` knots plus the raw 13-state
vector; `SolverInformation` (defined in the same header) — `success`, `return_code`,
`total_solver_time` and acados/QP diagnostics.

**Call order.** `UpdateState` and `UpdateGaitSequence` → `GetWrenchSequence`.

**Threading.** `mpc_lock_` held for the whole sequence. **`GetWrenchSequence` is permitted to
block** — the host annotates the call `// This one might block` and warns when
`total_solver_time >= MPC_CONTROL_DT`. It must still return; there is no timeout or cancellation in
the contract. `success == false` is reported by the host as an error but the pipeline continues
using whatever was written into `wrench_sequence`, so an implementation must always leave both
output arguments in a usable state.

**Missing from the contract.** Runtime tuning — `SetStateWeights`, `SetInputWeights`, `SetFmax`,
`SetMu` — is used by the host but lives only on `MPC`. See G1 and §6.

### 4.3 `SwingLegControllerInterface`

Header: `ws/src/controllers/include/mit_controller/swing_leg_controller_interface.hpp`

| Method | Dir | Called from | Rate |
|---|---|---|---|
| `void UpdateGaitSequence(const GaitSequence&)` | in | `SLCLoopCallback` (`:1350`), only when `gs_updated_` | ≤ 100 Hz |
| `void UpdateState(const StateInterface&)` | in | `MPCLoopCallback` (`:816`) and `SLCLoopCallback` (`:1359`) | 100 + 500 Hz |
| `void UpdateModel(const ModelInterface&)` | in | `ModelAdaptationCallback` (`:1022`) | event-driven |
| `void GetFeetTargets(FeetTargets&)` | out | `SLCLoopCallback` (`:1360`) | 500 Hz |
| `void GetProgress(std::array<double, N_LEGS>&, std::array<LegState, N_LEGS>&)` | out | `SLCLoopCallback` (`:1361`) | 500 Hz |
| `void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS>&, std::array<Eigen::Vector3d, N_LEGS>&)` | out | `ControlLoopCallback` (`:1301`), diagnostics only | 500 Hz |

**Data.** Out: `FeetTargets` (`mit_controller/feet_targets.hpp`) — per-leg position, velocity and
acceleration. `LegState` is declared on the interface itself:
`STANCE`, `NOT_STARTED` (scheduled to swing, not yet moving), `SWINGING`, `REACHED`.
`GetCurrentTrajs` returns swing start/end points for the `/swing_leg_trajs` visualisation, gated on
`PUBLISH_SWING_LEG_TRAJECTORIES` (currently `false`).

**Call order.** `UpdateGaitSequence` (conditional) → `UpdateState` → `GetFeetTargets` →
`GetProgress`.

**Threading.** `slc_lock_` is held across `:1345-1368`. Note that `UpdateState` is *also* called
from the 100 Hz MPC loop at `:816` — under `mpc_lock_`, **not** `slc_lock_` (G6) — and
`GetCurrentTrajs` is called from the control loop with no SLC lock at all (G10). An implementation
must not assume single-threaded access today, even though the contract intends it.

**Missing from the contract.** `SetSwingHeight`, `SetWorldBlend`,
`SetMaximumSwingProgressToUpdateTarget` — see G2.

### 4.4 `WBCInterface<JointCommandType>`

Header: `ws/src/controllers/include/mit_controller/wbc_interface.hpp`

| Method | Dir | Called from | Rate |
|---|---|---|---|
| `void UpdateState(const StateInterface&)` | in | `ControlLoopCallback` (`:1069`) | 500 Hz |
| `void UpdateModel(const ModelInterface&)` | in | `ModelAdaptationCallback` (`:1018`) | event-driven |
| `void UpdateTarget(const Quaterniond&, const Vector3d& pos, const Vector3d& lin_vel, const Vector3d& ang_vel)` | in | `ControlLoopCallback` (`:1075`) | 500 Hz |
| `void UpdateFeetTarget(const FeetTargets&)` | in | `ControlLoopCallback` (`:1246`) | 500 Hz |
| `void UpdateFootContact(const FootContact&)` | in | `ControlLoopCallback` (`:1247`) | 500 Hz |
| `void UpdateWrenches(const Wrenches&)` | in | `ControlLoopCallback` (`:1248`) | 500 Hz |
| `WBCReturn GetJointCommand(JointCommandType&)` | out | `ControlLoopCallback` (`:1257` / `:1269`) | 500 Hz |

**Type members.** `JOINT_COMMAND_TYPE`, `Wrenches = std::array<Eigen::Vector3d, N_LEGS>`,
`FootContact = std::array<bool, N_LEGS>`. `WBCReturn` (same header) carries `success`,
`qp_update_time`, `qp_solve_time`.

**Command types** (`mit_controller/joint_commands.hpp`): `CartesianCommands` (per-leg position,
velocity, force → `/leg_cmd`) or `JointTorqueVelocityPositionCommands` (per-joint → `/leg_joint_cmd`).
Which one is used is fixed **at compile time** by `USE_WBC`, itself set from `ROBOT_MODEL`
(`GO2` → `WBCArcOPT` with joint commands, `ULAB` → `InverseDynamics` with Cartesian commands).

**Call order.** `UpdateState` → `UpdateTarget` → `UpdateFeetTarget` → `UpdateFootContact` →
`UpdateWrenches` → `GetJointCommand`. All six inputs are refreshed every cycle before the solve.

**Important:** the target passed to `UpdateTarget` is **`mpc_prediction.orientation[1]` /
`position[1]`** — the MPC's prediction one step ahead — not the raw gait-sequence target. The
commented-out alternative at `:1070` shows the gait-sequence variant that is *not* in use.

**Threading.** `wbc_lock_` is held from `:1064` to `:1279`, i.e. across the contact FSM, all
`Update*` calls and the solve. Unlike the MPC, the host does **not** warn when the WBC overruns its
2 ms budget — the check exists but is commented out at `:1292-1297`.

**Missing from the contract.** The `InverseDynamics` tuning setters (`setFootPositionBasedOnTargetHeight`,
`setFootPositionBasedOnTargetOrientation`, `setTransformationFilterSize`, `setTargetVelocityBlend`)
— see G3. The template parameter itself is a plugin blocker — see G8.

### 4.5 `ModelAdaptationInterface`

Header: `ws/src/controllers/include/model_adaptation/model_adaptation_interface.hpp`

| Method | Dir | Called from | Rate |
|---|---|---|---|
| `void UpdateState(const StateInterface&)` | in | `ModelAdaptationCallback` (`:993`) | 100 Hz |
| `void UpdateGaitSequence(const GaitSequence&)` | in | `ModelAdaptationCallback` (`:997`) | 100 Hz |
| `bool DoModelAdaptation(ModelInterface&)` | in/out | `ModelAdaptationCallback` (`:998`) | 100 Hz |
| `Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const` | out | diagnostics (`:1000`) | 100 Hz |
| `Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const` | out | diagnostics (`:1002`) | 100 Hz |
| `Eigen::Vector<double, NUM_PARAMS> GetDelta() const` | out | diagnostics (`:1004`) | 100 Hz |
| `Eigen::Vector<double, 6> GetTotalForceTorque() const` | out | diagnostics (`:1005`) | 100 Hz |
| `Eigen::Vector<double, NUM_PARAMS> GetSV() const` | out | diagnostics (`:1007`) | 100 Hz |

`NUM_PARAMS = 3` (mass, CoM offset terms). The whole stage is gated on the `use_model_adaptation`
parameter; when it is false none of these are called.

**`DoModelAdaptation` is the only mutating stage method in the pipeline.** It takes the host's
`QuadModelPino` by non-const reference and returns `true` if it changed it. On `true`, the host
broadcasts the new model to every other stage — `mpc_`, `gs_`, `wbc_`, `slc_` — and publishes it on
`/quad_model` (`:1009-1032`). This is the model-update fan-out that issue #15 (M3.4) will factor
into a helper.

**Call order.** `UpdateState` → `UpdateGaitSequence` → `DoModelAdaptation` → the five `Get*`
accessors (called unconditionally, for `/quad_model_debug`, whether or not the model changed).

**Threading.** This stage runs in its own callback group with **no mutex protecting the model it
mutates** — see G6. The `Get*` accessors are `const` and must be side-effect free.

## 5. Gaps between the contracts and the current host

Every item below is a place where `mit_controller_node` does not respect the contract above, or
where the contract is incomplete. Each has an owning follow-up issue.

| # | Gap | Evidence | Impact | Owner |
|---|---|---|---|---|
| G1 | MPC tuning setters not on `MPCInterface` | `reinterpret_cast<MPC*>` at `mit_controller_node.cpp:367, 376, 382, 387, 392, 824, 831`; methods at `mpc.hpp:145-148` | Blocks plugin loading. `:824`/`:831` are on the **hot 100 Hz path** (weight switch on `KEEP`↔`MOVE`), not just the parameter callback | #2 |
| G2 | SLC tuning setters not on the interface | `reinterpret_cast<SwingLegController*>` at `:458, 462, 471` | Blocks plugin loading | #2 |
| G3 | WBC tuning setters not on the interface | `reinterpret_cast<InverseDynamics*>` at `:440, 446, 452, 466` | Blocks plugin loading; also `InverseDynamics`-specific, so meaningless for the Go2 `WBCArcOPT` path | #2 |
| G4 | Missing virtual destructors | `SwingLegControllerInterface`, `WBCInterface<T>`, `ModelAdaptationInterface` had none, yet the host owns all three as `std::unique_ptr<Interface>` (`mit_controller_node.hpp:111-117`) | Deleting through the base pointer was undefined behaviour and leaked the `unique_ptr<ModelInterface>` / `unique_ptr<StateInterface>` each implementation owns | **fixed in M1.1** |
| G5 | `typeid` guards are dead code | `:439, 445, 451` compare `typeid(wbc_.get())` — a *pointer* type — with `typeid(InverseDynamics)` | Never equal, so the three `wbc.inverse_dynamics.*` parameters silently do nothing at runtime | #2 |
| G6 | Inconsistent lock discipline | `ModelAdaptationCallback` mutates `quad_model_` unlocked (`:998`); `gs_->UpdateModel` (`:1014`) and `slc_->UpdateState` (`:816`) run under `mpc_lock_` rather than their own stage lock | The contract cannot state "one mutex per stage" until this is regularised. Model ownership is undefined | #9 |
| G7 | Wrong setter called | `:466-468` — parameter `wbc.inverse_dynamics.transformation_filter_size` calls `setFootPositionBasedOnTargetOrientation(integer_value)` instead of `setTransformationFilterSize` | Pre-existing bug; the filter size cannot be changed at runtime and a bool setter receives an int | #2 |
| G8 | `WBCInterface` is a template | `WBCType` is a `std::conditional<USE_WBC, …>` typedef (`mit_controller_node.hpp:112-114`); `reinterpret_cast` at `:604, 623, 1257, 1269` | A class template cannot be a pluginlib base class. The command type must become a runtime choice | #13 |
| G9 | `AdaptiveGaitSequencer` config escape hatch | `dynamic_cast` + `ad_gs->Gait()` at `:316-317`, reaching ~15 `AdaptiveGait` setters | The one *guarded* cast, so not unsafe — but still concrete-type coupling. Needs a generic per-stage parameter hook | #2 |
| G10 | Unsynchronised diagnostic reads | `gs_->GetGaitState()` at `:959` runs after `gait_sequencer_lock_` is released at `:881`; `slc_->GetCurrentTrajs()` at `:1301` runs after `wbc_lock_` is released and never takes `slc_lock_` | Data race against `UpdateModel` / `SLCLoopCallback`. Low severity (diagnostics only) but it means `Get*` methods cannot be documented as "called under the stage lock" | #9 |

**Cast inventory:** `grep -c reinterpret_cast ws/src/controllers/src/mit_controller_node.cpp` → **18**,
covering G1 (7), G2 (3), G3 (4), G8 (4). Issue #2's acceptance criterion is that this count reaches
zero for stage access.

## 6. Proposed API changes for M1.2 (issue #2)

G1, G2, G3 and G9 are the same problem: the host needs to push runtime configuration into a stage,
and today it does that by knowing the concrete type. Two options were considered.

**Option A — typed setters on each interface.**

```cpp
// on MPCInterface
virtual void SetStateWeights(const Eigen::Ref<const Eigen::VectorXd>& weights) = 0;
virtual void SetInputWeights(double alpha) = 0;
virtual void SetFmax(double fmax) = 0;
virtual void SetMu(double mu) = 0;
```

Direct and type-safe, but it forces every implementation to have a notion of "state weights" and
leaks `MPC::STATE_SIZE` into the abstract contract. It also does nothing for G9, where the
configuration surface is ~15 gait-specific setters.

**Option B — a generic parameter hook on every stage (recommended).**

```cpp
/**
 * Applies a runtime parameter to this stage.
 * @return true if the key was recognised and applied, false otherwise (host logs a warning).
 * Called from the parameter-event callback with the stage's mutex held.
 */
virtual bool SetParameter(const std::string& key, const rclcpp::ParameterValue& value) = 0;
```

One method covers G1, G2, G3 and G9; new stages declare their own keys without touching the
interface; and it survives plugin loading, since the host never needs the concrete type. Costs:
the type check moves to runtime, and `MPC` must map `"mpc_state_weights_stand"` onto its typed
setter internally.

**Recommendation: Option B**, with the existing typed setters kept as public methods on the concrete
classes (so `SetParameter` is a thin dispatch layer and current unit-level usage still compiles).

Two changes are needed regardless of which option is chosen:

1. **The `KEEP`↔`MOVE` weight switch** (`:820-838`) must move out of the host. It is control logic,
   not configuration. Either the MPC observes `sequence_mode` from the `GaitSequence` it already
   receives, or the host calls a single `SetParameter("mpc_mode", …)`.
2. **G8 must be resolved before `WBCInterface` can be a plugin base.** Suggested direction: a
   non-template `WBCInterface` with a `GetJointCommand(JointCommandVariant&)` or separate
   `GetCartesianCommand` / `GetJointCommand` methods plus a `SupportedCommandType()` query. This is
   issue #13's scope; M1.2 should not attempt it.

## 7. Adjacent seams

Not part of the five frozen contracts, but relevant to the modularity work:

- **`GaitInterface`** (`mit_controller/gait_interface.hpp`) — 9 lines, one method
  (`get_t_stance(leg)`), already well-formed. Used *inside* gait sequencer implementations, not by
  the host. No changes needed.
- **`StateInterface` / `ModelInterface`** (`common/`) — the shared data types every stage consumes.
  Issue #3 (M1.3) covers exposing these as a stable package surface.
- **The contact FSM has no interface at all.** `SWING / STANCE / EARLY_CONTACT / LATE_CONTACT /
  LOST_CONTACT` is declared as a private enum on `MITController`
  (`mit_controller_node.hpp:50`) and implemented inline in `ControlLoopCallback`
  (`:1088-1200`), controlling early/late/lost contact handling and swing-phase rescheduling. It is a
  sixth pipeline stage in everything but name. Extracting it is issue #4 (M1.4, define
  `ContactLogicInterface`) and issue #12 (M3.1, extract the implementation).
- **The model-update broadcast** (`:1009-1032`) is duplicated logic across four stages and is
  issue #15 (M3.4).

## 8. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #1 | [M1.1] Audit and freeze stage interface APIs | **This document** |
| #2 | [M1.2] Remove host→concrete casts | Consumes §5 (G1, G2, G3, G5, G7, G9) and §6 |
| #3 | [M1.3] Shared pipeline data types package surface | Consumes §7 |
| #4 | [M1.4] Define `ContactLogicInterface` | Consumes §7 |
| #5 | [M1.5] Contract tests / compile smoke | Asserts §4 method tables |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | Owns G6, G10 |
| #13 | [M3.2] Runtime WBC / command-type profile | Owns G8 |
| #15 | [M3.4] Model update broadcast helper | Consumes §4.5 |
