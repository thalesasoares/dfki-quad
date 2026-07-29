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
| `wbc_plugins.xml` | `StagePlugin<WBCInterface>` | `wbc_arc_opt`, `inverse_dynamics` |
| `model_adaptation_plugins.xml` | `StagePlugin<ModelAdaptationInterface>` | `kf_adaptation`, `rls_adaptation` |
| `contact_logic_plugins.xml` | `StagePlugin<ContactLogicInterface>` | `default_contact_logic` |

`wbc_plugins.xml` declared **two** base-class-types in one library until M3.2, because `WBCInterface`
was a class template: `wbc_arc_opt` under the `JointTorqueVelocityPositionCommands` instantiation and
`inverse_dynamics` under the `CartesianCommands` one (`stage_plugin_bases::kWBCCartesian`, added in
M2.3). #13 (M3.2) de-templated the interface and collapsed them into the single
`StagePlugin<WBCInterface>` above — which is what makes `wbc.type` a launch choice rather than a
rebuild, since both classes were always compiled into the one robot-agnostic `libwbc_plugins`.

**These base-class-type strings are the schema.** M2.2's loader helper and M2.3's wrapper
`PLUGINLIB_EXPORT_CLASS` calls must use them verbatim. Changing one requires updating this file, the
XML, and the loader in the same PR — the same rule [`plugin_lifecycle.md`](plugin_lifecycle.md) §1
applies to the lifecycle. M2.2 spells them once in code as `stage_plugin_bases::k*`
(`mit_controller/stage_loader.hpp`), and a test compares each constant against the literal in this
table.

**In XML they must be escaped.** `<` is not legal in an attribute value, so a real `<class>` entry
reads `base_class_type="StagePlugin&lt;GaitSequencerInterface&gt;"`; tinyxml2 unescapes before
pluginlib compares against the C++-side string. M2.3's `<class>` entries use the escaped spelling
(before M3.2 the WBC ones nested it: `StagePlugin&lt;WBCInterface&lt;CartesianCommands&gt;&gt;`); see
[`stage_loading.md`](stage_loading.md) §5.

Design decisions:

- **One file per stage base, not one combined file.** M2.3 fills them in one stage at a time
  (reviewable per-stage diffs); M3.1 added the contact class without touching the others; and #13
  (M3.2) reworked **only** `wbc_plugins.xml` when the WBC interface was de-templated
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §6) — the prediction held, so isolating the WBC
  declaration did contain that churn.
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
- **Two WBC bases until #13; one since.** `WBCInterface` was a class template (gap G8), so M2.3 had
  to register the Go2 `JointTorqueVelocityPositionCommands` instantiation (`wbc_arc_opt`) and the
  ULab/Cartesian `CartesianCommands` instantiation (`inverse_dynamics`) as two `base_class_type`s in
  the one `libwbc_plugins`. Both were stock plugins from then on (issue #8 wanted every current
  algorithm exported), but nothing could *select* between them without a rebuild, because the host
  could only instantiate a loader for the base matching its build flavour. M3.2 (#13) de-templated
  the interface — the command family moved onto `WBCCommandMode SupportedCommandMode()` — so the two
  collapse into one base and either is loadable from any build
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §6).

Each file is registered with `pluginlib_export_plugin_description_file(controllers plugins/<f>.xml)`,
which installs it to `share/controllers/plugins/` and registers the
`controllers__pluginlib__plugin` ament-index resource `ClassLoader` reads for discovery.

## 3a. The first out-of-package plugin (M4.2, #17)

Everything above describes plugins that ship *inside* `controllers`. M4.2 adds the first one that
does not: `example_passthrough_slc`, in `ws/src/examples/example_stage_plugins`
([its README](../../ws/src/examples/example_stage_plugins/README.md) is the contributor-facing
version of this section). It is the first exercise of the promise
[`stage_loading.md`](stage_loading.md) has carried since M2.2 — that a third party joins the
pipeline without an allowlist — and it establishes the pattern #18 (M4.3) generalised into
[`adding_a_stage.md`](adding_a_stage.md), the step-by-step guide for adding a stage from your own
package. What follows here is the *mechanism*; that document is the procedure.

The mechanism is the *same single line*, with the same first argument:

```cmake
pluginlib_export_plugin_description_file(controllers plugins/example_slc_plugins.xml)
```

The first argument names the **resource**, not the exporting project. `controllers` owns the base
classes, so `controllers__pluginlib__plugin` is the resource `StageLoader` searches
(`kStagePluginPackage`), and pluginlib resolves it across *every* installed package. A package that
registers there is found; nothing in `controllers` is edited, and there is no list to be added to.
Two details follow from that and are worth stating once:

- The `<library path>` resolves against the install prefix of the package that owns the XML, so an
  out-of-package plugin installs its `.so` to **its own** `lib/`, not to `controllers`'.
- `base_class_type` must still match `stage_plugin_bases::k*` character for character. The schema
  in §3 is workspace-wide, not package-local — that is what makes an out-of-package class
  interchangeable with a stock one.

The one thing #17 found that this document did not already say is a *consumer* constraint rather
than a discovery one: **`controllers` must be consumed as an include path, never as a link
dependency.** `ament_target_dependencies(<target> controllers)` puts `${controllers_LIBRARIES}` —
the closure re-exported in §1, `common` → `quad_model` → `drake` — on the link line, and fails with
`links to target "drake::drake" but the target was not found`. It is also unnecessary: a stage
plugin references no compiled symbol of `controllers` or `common`, only pure-virtual interfaces,
header-only helpers and plain data structs, with the host supplying the implementations through the
vtable at runtime. `target_include_directories(... ${controllers_INCLUDE_DIRS})` is the correct
form, and it is the out-of-package expression of the same "include-only" rule
[`stock_plugins.md`](stock_plugins.md) §4 keeps internally for `common`. (One gap in the §1
re-export is worth recording: `${controllers_INCLUDE_DIRS}` carries `common`'s and `interfaces`'
include directories, but not Eigen's — Eigen ships an INTERFACE target rather than ament include
variables — so a consumer names `Eigen3::Eigen` explicitly.)

## 4. What guards it

`test/test_plugin_discovery.cpp` (runs under `colcon test`):

1. Asserts the `controllers__pluginlib__plugin` resource exists for package `controllers` and lists
   all six description files — the real proof of "exports plugin XML correctly for discovery".
2. Constructs a `pluginlib::ClassLoader` for each of the six stage bases from the exported header +
   XML and asserts zero declared classes (M2.1 is schema-only). In M2.3 these assertions flip to
   expecting the stock IDs, so the test grows with the milestone.

**Point 2 asserts containment, not equality, since M4.2 (#17)** — every stock ID is declared, rather
than *only* the stock IDs. This is the milestone succeeding, not the test being weakened. Once
another package may legitimately declare a class against one of these bases (§3a), the exact set for
a base is a property of the *workspace*, not of this package: an equality assertion would fail on a
correct workspace, and would flip on whether the other package happened to be built — environment,
not correctness. Containment still catches everything the assertion exists for: a stock plugin
dropped from its XML, a library that failed to install, a `base_class_type` that drifted between the
XML and `stage_loader.hpp`.

Exactness is kept wherever it remains well defined, so nothing goes unpinned:

| Assertion | Scope | Still exact? |
|---|---|---|
| Point 1, the resource file list | package `controllers` | **Yes** — an out-of-package plugin registers under *its own* package's resource entry |
| Point 2, declared classes per base | the whole workspace | No — containment |
| `test_stage_loader.cpp`, the synthetic test loader | one explicit XML path | **Yes** — bypasses ament discovery entirely |
| `example_stage_plugins`' own suite | its own IDs | **Yes** — each plugin package pins what it ships |

`test_stage_loader.cpp`'s `ProductionLoadersDeclareStockPluginsForEveryStageBase` moved to
containment in the same change and for the same reason: it is this assertion made through
`StageLoader` rather than a bare `ClassLoader`, and the two must not disagree.

M2.2 adds `test/test_stage_loader.cpp`, which re-checks point 2 through `StageLoader` — the API the
host will actually use — and additionally loads *real* plugins from a description file that is
deliberately **not** exported, so this test's zero-class expectation stays valid. See
[`stage_loading.md`](stage_loading.md) §7.

Because the ament resource lives in the install space, the CMake target extends `AMENT_PREFIX_PATH`
with `CMAKE_INSTALL_PREFIX` for the test process; under `colcon test` the package is already
installed, so the resource is present.

## 5. Performance

M4.2 (#17) changes no compiled `.cpp` in this package either — only two test files and this
document — so `mitcontrollernode` and the six stock plugin libraries are binary-identical across it.
The added package costs one more entry in the resource scan at loader construction (once, at
bring-up, off-loop) and nothing at all unless its plugin is actually selected, since pluginlib
`dlopen`s a library only on demand. The discipline carried forward to any future out-of-package
plugin is the one in §3a's third bullet and in that package's `CMakeLists.txt`: **it must mirror the
node's Release flags**, because a stage compiled at a lower optimisation level than the node is an
on-robot regression no test would catch.

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
| #13 | [M3.2] Runtime WBC / command-type profile | **Done.** Collapsed `wbc_plugins.xml` onto the single `StagePlugin<WBCInterface>` base |
| #17 | [M4.2] Example passthrough / logging plugin | **Done.** First out-of-package plugin — §3a; moved the declared-class assertions to containment — §4 |
| #18 | [M4.3] Contributor guide: add a control stage | **Done.** Turns §3a into a procedure — [`adding_a_stage.md`](adding_a_stage.md) §4 |
