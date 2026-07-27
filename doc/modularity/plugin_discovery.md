# Plugin Discovery: dependency, exported surface, description schema

**Status:** added in M2.1 (issue #6) ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`plugin_lifecycle.md`](plugin_lifecycle.md) — the `StagePlugin` lifecycle ·
[`stage_loading.md`](stage_loading.md) — the M2.2 loader that consumes this schema ·
[`stage_contracts.md`](stage_contracts.md) — the frozen stage APIs ·
[`pipeline_types.md`](pipeline_types.md) — the exported data types

[`plugin_lifecycle.md`](plugin_lifecycle.md) specified *how a stage comes to life* through
`pluginlib`. This document is the M2.1 groundwork that makes that loadable at all: the `pluginlib`
dependency, the stage interface headers joining the exported surface, and the **plugin description
schema** — the XML files and base-class-type strings that `pluginlib::ClassLoader` discovers.

M2.1 is deliberately *foundation only*. It ships **no plugin classes**: the stock wrappers are M2.3
(issue #8). What it fixes in place is the vocabulary every later milestone builds on.

## 1. The dependency

`controllers` now declares `pluginlib` (`package.xml`, `find_package(pluginlib REQUIRED)`). The
package `ament_export_dependencies(... rclcpp interfaces pluginlib)`, so an out-of-package stage
plugin that does `find_package(controllers)` inherits them transitively instead of re-listing them —
the exported interface headers include `rclcpp/parameter_value.hpp`, `rclcpp/time.hpp` and
`interfaces/msg/gait_state.hpp`, and `StagePlugin` is a `pluginlib` base.

## 2. The exported surface

The frozen stage interface headers join the installed surface alongside the M1.3 pipeline types (see
[`pipeline_types.md`](pipeline_types.md) §6, which scheduled this for M2.1):

| Installed header | Destination |
|---|---|
| `gait_sequencer_interface.hpp` (+ `gait_sequencer_types.hpp`) | `include/mit_controller/` |
| `mpc_interface.hpp` | `include/mit_controller/` |
| `swing_leg_controller_interface.hpp` | `include/mit_controller/` |
| `wbc_interface.hpp` (+ `joint_commands.hpp`) | `include/mit_controller/` |
| `contact_logic_interface.hpp` | `include/mit_controller/` |
| `stage_plugin.hpp` | `include/mit_controller/` |
| `model_adaptation_interface.hpp` | `include/model_adaptation/` |

It is still an explicit allowlist, not `install(DIRECTORY include/)`: node and algorithm internals
stay private so "stable surface" keeps meaning something.

Three include-path fixes were needed to make these headers self-contained for an out-of-package
consumer (all behaviour-neutral, include lines only):

1. `mpc_interface.hpp` and `gait_sequencer_interface.hpp` included `potato_sim/potato_model.hpp`, a
   node-internal header. The include was **dead** — neither header (nor any exported header) uses
   `BrickModel`/`BrickState` — so it was removed. This closes follow-up #2 in
   [`pipeline_types.md`](pipeline_types.md) §7.
2. `model_adaptation_interface.hpp` included `"gait_sequence.hpp"` unqualified, which only resolved
   via the in-tree `include_directories(include/mit_controller)`. Qualified to
   `"mit_controller/gait_sequence.hpp"` so it resolves against the exported `include/` root once
   installed to `include/model_adaptation/`.
3. `joint_commands.hpp` included the **host-only** `mit_controller_params.hpp` (which carries the
   `PUBLISH_*` switches and `USE_WBC`, itself gated on the `ROBOT_MODEL` compile definition only this
   package's targets set) purely for the `N_LEGS`/`N_JOINTS_PER_LEG` constants. Retargeted to
   `mit_controller/pipeline_constants.hpp` (the M1.3 split), so no host-only header leaks onto the
   surface through the WBC interface.

`src/tools/pipeline_types_surface_check.cpp` — compiled with an export-only include path — now
includes all of these too, so any regression to their self-containment fails the ordinary
`colcon build`.

## 3. The plugin description schema

One description file per stage base class, under `plugins/`:

| File | `base_class_type` | Stock classes (M2.3, #8) |
|---|---|---|
| `gait_sequencer_plugins.xml` | `StagePlugin<GaitSequencerInterface>` | `simple_gait`, `adaptive_gait` |
| `mpc_plugins.xml` | `StagePlugin<MPCInterface>` | `acados_mpc` |
| `slc_plugins.xml` | `StagePlugin<SwingLegControllerInterface>` | `bezier_swing` |
| `wbc_plugins.xml` | `StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>` | `wbc_arc_opt` |
| `wbc_plugins.xml` | `StagePlugin<WBCInterface<CartesianCommands>>` | `inverse_dynamics` |
| `model_adaptation_plugins.xml` | `StagePlugin<ModelAdaptationInterface>` | `kf_adaptation`, `rls_adaptation` |
| `contact_logic_plugins.xml` | `StagePlugin<ContactLogicInterface>` | `default_contact_logic` |

`wbc_plugins.xml` declares **two** base-class-types in one library because `WBCInterface` is still a
template: `wbc_arc_opt` under the `JointTorqueVelocityPositionCommands` instantiation and
`inverse_dynamics` under the `CartesianCommands` one (`stage_plugin_bases::kWBCCartesian`, added in
M2.3). #13 (M3.2) collapses them into one once the interface is de-templated.

**These base-class-type strings are the schema.** M2.2's loader helper and M2.3's wrapper
`PLUGINLIB_EXPORT_CLASS` calls must use them verbatim. Changing one requires updating this file, the
XML, and the loader in the same PR — the same rule [`plugin_lifecycle.md`](plugin_lifecycle.md) §1
applies to the lifecycle. M2.2 spells them once in code as `stage_plugin_bases::k*`
(`mit_controller/stage_loader.hpp`), and a test compares each constant against the literal in this
table.

**In XML they must be escaped.** `<` is not legal in an attribute value, so a real `<class>` entry
reads `base_class_type="StagePlugin&lt;GaitSequencerInterface&gt;"`; tinyxml2 unescapes before
pluginlib compares against the C++-side string. M2.3's `<class>` entries use the escaped spelling
(the WBC ones nest it: `StagePlugin&lt;WBCInterface&lt;CartesianCommands&gt;&gt;`); see
[`stage_loading.md`](stage_loading.md) §5.

Design decisions:

- **One file per stage base, not one combined file.** M2.3 fills them in one stage at a time
  (reviewable per-stage diffs); M3.1 added the contact class without touching the others; and #13
  (M3.2) reworks **only** `wbc_plugins.xml` when the WBC interface is de-templated
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §6). Isolating the WBC declaration now contains that
  future churn.
- **Class lists filled in M2.3.** M2.1 shipped each file as an empty `<class_libraries>` root
  (`pluginlib` reads that as zero declared classes); M2.3 (#8) added one `<library>` per stage —
  `libgait_sequencer_plugins`, `libmpc_plugins`, `libslc_plugins`, `libwbc_plugins`,
  `libmodel_adaptation_plugins` — each with the stock `<class>` entries above, and M3.1 (#12) added
  `default_contact_logic` in `libcontact_logic_plugins`. The `type:` key vocabulary is M2.5's (#10); these files are
  naming-agnostic (the `name=` values are the suggested stock IDs).
- **The contact file was defined ahead of its class.** `ContactLogicInterface` was frozen in M1.4 and
  `StagePlugin<ContactLogicInterface>` already compiles, so issue #6's "(and Contact when ready)" is
  satisfiable at that point. M3.1 (#12) was then purely additive: one `<library>` block, no change to
  the other five files.
- **Two WBC bases until #13.** `WBCInterface` is still a class template (gap G8), so M2.3 registers
  the Go2 `JointTorqueVelocityPositionCommands` instantiation (`wbc_arc_opt`) and the
  ULab/Cartesian `CartesianCommands` instantiation (`inverse_dynamics`) as two `base_class_type`s in
  the one `libwbc_plugins`. Both are stock plugins now (issue #8 wanted every current algorithm
  exported); #13 (M3.2) collapses the two bases into one when the interface is de-templated
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §6).

Each file is registered with `pluginlib_export_plugin_description_file(controllers plugins/<f>.xml)`,
which installs it to `share/controllers/plugins/` and registers the
`controllers__pluginlib__plugin` ament-index resource `ClassLoader` reads for discovery.

## 4. What guards it

`test/test_plugin_discovery.cpp` (runs under `colcon test`):

1. Asserts the `controllers__pluginlib__plugin` resource exists for package `controllers` and lists
   all six description files — the real proof of "exports plugin XML correctly for discovery".
2. Constructs a `pluginlib::ClassLoader` for each of the six stage bases from the exported header +
   XML and asserts zero declared classes (M2.1 is schema-only). In M2.3 these assertions flip to
   expecting the stock IDs, so the test grows with the milestone.

M2.2 adds `test/test_stage_loader.cpp`, which re-checks point 2 through `StageLoader` — the API the
host will actually use — and additionally loads *real* plugins from a description file that is
deliberately **not** exported, so this test's zero-class expectation stays valid. See
[`stage_loading.md`](stage_loading.md) §7.

Because the ament resource lives in the install space, the CMake target extends `AMENT_PREFIX_PATH`
with `CMAKE_INSTALL_PREFIX` for the test process; under `colcon test` the package is already
installed, so the resource is present.

## 5. Performance

M2.1 changes no compiled `.cpp`: the changes are dependency metadata, header installation, three
behaviour-neutral include-line edits, XML data files, and a test. `mitcontrollernode`'s call graph,
optimisation flags and binary behaviour are unchanged; removing the dead includes can only reduce
compile time. `pluginlib` costs only at *load* time (library `dlopen` at bring-up), never in the
control loops — and in M2.1 nothing is loaded yet. The performance discipline to carry forward:
**M2.3** stock-plugin `.so`s must keep today's optimisation flags, and **M2.4** must keep calling
stages through the existing single virtual indirection. Both held: the libraries inherit the node's
Release flags, and the only per-cycle addition is the stock wrapper's forwarding call
([`pipeline_host.md`](pipeline_host.md) §6).

## 6. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #6 | [M2.1] pluginlib dependency and plugin description XML | **This document** |
| #7 | [M2.2] Stage plugin base + loader helper | Consumes the base-class-type strings — [`stage_loading.md`](stage_loading.md) |
| #8 | [M2.3] Wrap existing stages as stock plugins | Fills the `<class>` entries; flips the discovery test's expected count |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | Owns the loaders; keeps per-cycle indirection unchanged — [`pipeline_host.md`](pipeline_host.md) |
| #10 | [M2.5] YAML schema for stage selection | Owns the `type:` key vocabulary |
| #12 | [M3.1] Extract contact FSM | **Done.** Populated `contact_logic_plugins.xml` with `default_contact_logic` |
| #13 | [M3.2] Runtime WBC / command-type profile | Reworks `wbc_plugins.xml` once the interface is de-templated |
