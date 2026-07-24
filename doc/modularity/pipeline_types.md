# Shared Pipeline Data Types

**Status:** exported as of M1.3 (issue #3) ·
**Applies to:** `ws/src/controllers` ·
**Companion document:** [`stage_contracts.md`](stage_contracts.md)

`stage_contracts.md` specifies the *methods* each control-pipeline stage must provide. This document
specifies the *data* those methods exchange: the five in-process structs that flow between stages,
the constants their sizes are defined in terms of, and the include path an out-of-package stage
plugin compiles against.

## 1. Why these types are a package surface

The pipeline is being refactored so that each stage loads as an in-process `pluginlib` plugin (meta
issue #24, M2). A plugin lives in its own package and its own shared library, so it must be able to
include the pipeline types **without** reaching into the node implementation. Before M1.3 that was
impossible for a mechanical reason: `ws/src/controllers` installed no headers at all, so nothing
downstream could include them from an overlay workspace.

The types are deliberately **plain C++ aggregates, not ROS messages**. The architecture locked in
meta issue #24 rules out 500 Hz ROS hops for the swing-leg and whole-body stages; `FeetTargets` and
`WrenchSequence` cross stage boundaries every 2 ms. The `interfaces/` messages that carry pipeline
data (`interfaces/msg/GaitSequence`, `interfaces/msg/WBCTarget`) are **truncated, diagnostic and
visualisation-only**, and stay that way — see §6.

## 2. The surface

Six headers are installed to `<prefix>/include/mit_controller/`. This is an explicit allowlist in
`ws/src/controllers/CMakeLists.txt`, not `install(DIRECTORY include/)`: everything else under
`include/` is node and algorithm internals and stays private.

| Header | Provides | Produced by | Consumed by |
|---|---|---|---|
| `mit_controller/target.hpp` | `Target` | `/quad_control_target` subscriber (host) | Gait sequencing |
| `mit_controller/gait_sequence.hpp` | `GaitSequence` | Gait sequencing | MPC, swing leg, model adaptation |
| `mit_controller/wrench_sequence.hpp` | `WrenchSequence` | MPC | Whole-body control |
| `mit_controller/mpc_prediction.hpp` | `MPCPrediction` | MPC | Whole-body control |
| `mit_controller/feet_targets.hpp` | `FeetTargets` | Swing leg control | Whole-body control |
| `mit_controller/pipeline_constants.hpp` | `N_LEGS`, `GAIT_SEQUENCE_SIZE`, … | — | every stage |

Include them by their full path, which is stable and identical inside and outside the package:

```cpp
#include "mit_controller/gait_sequence.hpp"
```

### Consuming the surface from another package

```cmake
find_package(controllers REQUIRED)   # the pipeline types
find_package(common REQUIRED)        # StateInterface / ModelInterface
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(drake CONFIG REQUIRED /opt/drake)

add_library(my_stage src/my_stage.cpp)
ament_target_dependencies(my_stage controllers)
```

```xml
<depend>controllers</depend>
```

The `find_package` calls beyond `controllers` are needed because `common` exports library targets
without declaring its own transitive dependencies, so the consumer must resolve them. That is a
pre-existing gap in `common`, not a property of this surface — see §7.

## 3. Constants

`mit_controller/pipeline_constants.hpp` holds every structural constant the types and stage
contracts are written against. All are `static constexpr`, so they cost nothing at runtime.

| Constant | Value | Meaning |
|---|---|---|
| `N_LEGS` | 4 | legs, and the extent of every per-leg array |
| `N_JOINTS_PER_LEG` | 3 | joints per leg |
| `GAIT_SEQUENCE_SIZE` | 100 | knots in a `GaitSequence` |
| `MPC_PREDICTION_HORIZON` | 10 | MPC horizon steps |
| `MPC_DT` | 0.05 s | spacing between gait-sequence / MPC knots |
| `MPC_CONTROL_DT` | 0.01 s | MPC loop period (100 Hz) |
| `SWING_LEG_DT` | 0.002 s | swing-leg loop period (500 Hz) |
| `CONTROL_DT`, `WBC_CYCLE_DT` | 0.002 s | control / WBC loop period (500 Hz) |
| `MODEL_ADAPTATION_DT` | 0.01 s | model adaptation loop period (100 Hz) |
| `MODEL_ADAPTATION_BATCH_SIZE` | 100 | model adaptation batch |
| `FEET_POSITION_SEQUENCE_SIZE` | 500 | `(MPC_DT / MPC_CONTROL_DT) × GAIT_SEQUENCE_SIZE` |

So a gait sequence spans 100 × 50 ms = **5 s** of plan, and the MPC horizon covers
10 × 50 ms = **500 ms**.

**Host-only configuration is not on the surface.** The `PUBLISH_*` diagnostic switches and the
`USE_WBC` selector stay in `mit_controller/mit_controller_params.hpp`, which is *not* installed.
`USE_WBC` is guarded by `#ifdef ROBOT_MODEL`, a compile definition only this package's own targets
set, so exporting it would hand plugins a symbol whose value depends on flags they do not control.
`mit_controller_params.hpp` includes `pipeline_constants.hpp`, so in-package code sees exactly the
same set of names it always did.

## 4. The types

### `Target` — `mit_controller/target.hpp`

Operator setpoint entering the pipeline. Position `x, y, z` and yaw rate `wz` are **world frame**;
`roll`, `pitch`, `wx`, `wy` and the `hybrid_*_dot` velocities are **hybrid frame** (z axis aligned
with world, x axis with body); `yaw` is the same in both. `full_orientation` is a world-frame
quaternion.

Every field has a matching `bool` in the nested `active` struct. A field whose flag is `false` is
**ignored** by the gait sequencer — the flags are how the host expresses "hold whatever you had".
All fields default to zero and all flags to `false`, so a default-constructed `Target` requests
nothing.

### `GaitSequence` — `mit_controller/gait_sequence.hpp`

The `GAIT_SEQUENCE_SIZE`-long plan, all values in **world frame**.

- `time_stamp` (`StateInterface::TimePoint`) — creation time; the time-dependent fields are valid
  relative to it. This is the surface's only dependency on the `common` package.
- Current target: `target_position`, `target_velocity`, `target_orientation`, `target_twist`,
  `target_height`, `current_height`.
- Per-knot sequences: `contact_sequence`, `foot_position_sequence`, `reference_trajectory_position`
  / `_orientation` / `_velocity` / `_twist`, `desired_reference_trajectory_position` /
  `_orientation`, `swing_time_sequence`.
- `gait_swing_time` — per-leg nominal swing duration.
- `sequence_mode` — `GaitSequence::KEEP` (0) or `GaitSequence::MOVE` (1).

`sequence_mode` is **control data, not configuration**. Since issue #2 the MPC reads it inside
`UpdateGaitSequence` and switches its own cost weights; the host no longer pushes a weight switch on
the hot path. A gait sequencer implementation must therefore set it correctly every cycle.

### `WrenchSequence` — `mit_controller/wrench_sequence.hpp`

`forces[MPC_PREDICTION_HORIZON][N_LEGS]` — ground reaction forces per horizon step per leg. Note the
extent is the horizon itself, **not** `horizon + 1` as in `MPCPrediction`.

### `MPCPrediction` — `mit_controller/mpc_prediction.hpp`

Predicted body trajectory over `MPC_PREDICTION_HORIZON + 1` knots: `orientation`, `position`,
`angular_velocity`, `linear_velocity`, plus `raw_data`, the raw 13-state vector per knot.

Index 0 is the current knot. **The whole-body controller tracks index 1** — the one-step-ahead
prediction — not the raw gait-sequence target (see `stage_contracts.md` §4.4).

### `FeetTargets` — `mit_controller/feet_targets.hpp`

`positions`, `velocities`, `accelerations`, each `N_LEGS` long. Written by the swing-leg stage at
500 Hz and consumed by the whole-body stage in the same cycle.

## 5. Rules for changing these types

1. **Any change to a field, a constant value, or an array extent in these six headers requires
   updating this document in the same pull request.** The tables above are the specification.
2. New stage implementations are written against this document, not against a concrete stage class.
3. Adding a header to the exported surface means adding it to three places that must stay in sync:
   the `install(FILES ...)` allowlist in `CMakeLists.txt`, the include list in
   `src/tools/pipeline_types_surface_check.cpp`, and §2 here.
4. Do not add host-only configuration to `pipeline_constants.hpp`. If a value depends on
   `ROBOT_MODEL` or on a `PUBLISH_*` switch, it belongs in `mit_controller_params.hpp`.

### Self-containment is enforced at build time

`src/tools/pipeline_types_surface_check.cpp` is compiled by the `pipeline_types_surface_check`
OBJECT library, whose include path is restricted to `include/` plus the `common` and Eigen
dependencies — it deliberately does **not** get the package's usual `include/mit_controller` and
`src` entries. It therefore sees the headers exactly as an out-of-package plugin does, and the build
fails if a surface header stops being self-contained, starts pulling in node internals, or drifts
out of sync with `pipeline_constants.hpp`. It is built unconditionally, not under `BUILD_TESTING`,
so it cannot silently rot. The stage-shape contract test is a separate, complementary guard, added
in M1.5 (issue #5) — see [`contract_tests.md`](contract_tests.md).

### Copying and alignment

All five types are aggregates of `std::array`s of fixed-size Eigen types and are copied **by value**
across stage boundaries under the stage locks. `GaitSequence` is the large one — roughly 20 KB — and
the host copies it once per 100 Hz cycle, as it did before M1.3. The package builds as C++17, whose
aligned `new` handles Eigen's over-aligned fixed-size types, so no
`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` is required to heap-allocate them in a plugin.

## 6. Non-goals

- **No full-horizon ROS messages.** Issue #3 explicitly excludes them. `interfaces/msg/GaitSequence`
  and friends stay truncated and viz-only; publishing a 5 s plan or 500 Hz foot targets over ROS is
  not a supported path.
- **No new package.** The include root stays `mit_controller/` inside `controllers`, the spelling
  every stage header already uses.
- **No layout or semantic changes.** M1.3 changed include lines, CMake rules and documentation only.
- **Stage interface headers are exported as of M2.1 (issue #6).** `*_interface.hpp` joined the
  surface together with the pluginlib base — see [`plugin_discovery.md`](plugin_discovery.md) §2. The
  dead `potato_sim/potato_model.hpp` includes in `mpc_interface.hpp` / `gait_sequencer_interface.hpp`
  were removed (follow-up #2 below, now resolved); `SolverInformation`, `WBCReturn` and
  `joint_commands.hpp` travel with their interfaces (and `joint_commands.hpp` was retargeted off the
  host-only `mit_controller_params.hpp` onto `pipeline_constants.hpp`). The pluginlib base,
  `mit_controller/stage_plugin.hpp`, was already self-contained.

## 7. Known follow-ups

| # | Item | Owner |
|---|---|---|
| 1 | `common` calls no `ament_export_dependencies`, so consumers must `find_package` its transitive dependencies themselves (§2). Fixing it would shorten the consumer boilerplate. | unfiled |
| 2 | ~~`mpc_interface.hpp` / `gait_sequencer_interface.hpp` depend on `potato_sim/potato_model.hpp`; must be resolved before the interfaces can be exported.~~ **Resolved in M2.1 (#6):** the includes were dead and were removed. | #6 (M2.1) |
| 3 | `common` exports `ROBOT_MODEL=<robot>` as an INTERFACE compile definition on `quad_model_symbolic`, so it leaks into every consumer of `common`. Harmless today, but it means the macro's presence proves nothing about the consumer's own configuration. | unfiled |

## 8. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #1 | [M1.1] Audit and freeze stage interface APIs | `stage_contracts.md`, the method-side companion |
| #2 | [M1.2] Remove host→concrete casts | Moved the `sequence_mode` weight switch into the MPC (§4) |
| #3 | [M1.3] Shared pipeline data types package surface | **This document** |
| #5 | [M1.5] Contract tests / compile smoke | Complementary guard, different axis — [`contract_tests.md`](contract_tests.md) |
| #6 | [M2.1] pluginlib dependency and plugin description XML | Added the interface headers to the surface (§6); see [`plugin_discovery.md`](plugin_discovery.md) |
| #7 | [M2.2] Stage plugin base + loader helper | Added `stage_loader.hpp` to the surface and the surface check; ran into the `common` transitive-dependency wart of §7 — see [`stage_loading.md`](stage_loading.md) |
