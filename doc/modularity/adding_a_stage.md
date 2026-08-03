# Adding a Control Stage

**Status:** added in M4.3 (issue #18) ·
**Applies to:** any package that ships a control-pipeline stage; the host is `ws/src/controllers` ·
**Companion documents:** [`stage_contracts.md`](stage_contracts.md) — what each stage must *do* ·
[`plugin_lifecycle.md`](plugin_lifecycle.md) — the `StagePlugin` / `Init` contract ·
[`plugin_discovery.md`](plugin_discovery.md) — the description-XML schema and the ament resource ·
[`stage_loading.md`](stage_loading.md) — the loader that resolves `type:` to your class ·
[`stock_plugins.md`](stock_plugins.md) — the in-package plugins your stage sits beside ·
[`example_stage_plugins`](../../ws/src/examples/example_stage_plugins/README.md) — the worked
example this guide walks you through

This is the contributor guide for **adding a control stage from your own package**. The claim it
makes good on is M4's exit criterion: a control stage can be added from a separate package, selected
by one parameter value, **with no edit to the host**. There is no allowlist to join and nothing in
`controllers` to touch.

Seven steps, in the order you will actually hit them:

| Step | Section |
|---|---|
| 1. Pick your stage | [§1](#1-pick-your-stage) |
| 2. Write the class | [§2](#2-write-the-class) |
| 3. Build it — ⚠️ three traps | [§3](#3-build-it) |
| 4. Export it | [§4](#4-export-it) |
| 5. Select it | [§5](#5-select-it) |
| 6. Test it | [§6](#6-test-it) |
| 7. Checklist before you PR | [§7](#7-checklist-before-you-pr) |

> **Two templates, and they are not the same file.** The stock wrappers in
> [`ws/src/controllers/src/plugins/`](../../ws/src/controllers/src/plugins/) are the **code**
> template — the `Init`-plus-forward shape is exactly right. Their **build wiring is not copyable**,
> because it depends on being inside `controllers` (directory-scope include paths, a
> package-internal spelling of `common`, inherited compiler flags). For the build, copy
> [`ws/src/examples/example_stage_plugins`](../../ws/src/examples/example_stage_plugins/), which is
> a real out-of-package package and is annotated for exactly this purpose. Copying a stock plugin's
> CMake block into your own package produces the failure in [§3.1](#31-never-link-controllers).

## 1. Pick your stage

Six stage interfaces are frozen and loadable. Pick the one whose contract you want to re-implement;
[`stage_contracts.md`](stage_contracts.md) §4 defines what each one must do, including call order
and timing.

| Stage | Interface header | Base-class-type string | Selection key | Stock ids |
|---|---|---|---|---|
| Gait sequencer | `mit_controller/gait_sequencer_interface.hpp` | `StagePlugin<GaitSequencerInterface>` | `gs.type` | `simple_gait`, `adaptive_gait`, `bio_gait` |
| MPC | `mit_controller/mpc_interface.hpp` | `StagePlugin<MPCInterface>` | `mpc.type` | `acados_mpc` |
| Swing leg controller | `mit_controller/swing_leg_controller_interface.hpp` | `StagePlugin<SwingLegControllerInterface>` | `slc.type` | `bezier_swing` |
| Whole-body control | `mit_controller/wbc_interface.hpp` | `StagePlugin<WBCInterface>` | `wbc.type` | `wbc_arc_opt`, `inverse_dynamics` |
| Model adaptation | `model_adaptation/model_adaptation_interface.hpp` | `StagePlugin<ModelAdaptationInterface>` | `model_adaptation.type` | `kf_adaptation`, `rls_adaptation` |
| Contact logic | `mit_controller/contact_logic_interface.hpp` | `StagePlugin<ContactLogicInterface>` | `contact_logic.type` | `default_contact_logic` |

The base-class-type strings are spelled once in code as `stage_plugin_bases::k*`
(`mit_controller/stage_loader.hpp`) and **that string is the schema**
([`plugin_discovery.md`](plugin_discovery.md) §3). It must match character for character in three
places: your `PLUGINLIB_EXPORT_CLASS` call, your description XML's `base_class_type` attribute, and
that header. A mismatch is not a compile error — it is a class that is never found.

Two things to know before you commit to a stage:

- **Timing.** The gait sequencer, MPC and model adaptation run at 100 Hz; the swing leg controller,
  WBC and contact logic run at 500 Hz ([`stage_contracts.md`](stage_contracts.md) §3). Your
  implementation inherits that budget — see [§7](#7-checklist-before-you-pr).
- **The WBC has one extra obligation.** Since M3.2 the command family is a *runtime* property:
  implement `WBCCommandMode SupportedCommandMode() const`, produce that family from the
  corresponding getter, and return `{false, 0, 0}` from the other. The host pairs your reported mode
  against `leg_control_mode` once, right after load, and a mismatch is fatal
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §6).

## 2. Write the class

A stage plugin is a `final` class deriving from `StagePlugin<Interface>`, and it obeys **two-phase
initialisation** ([`plugin_lifecycle.md`](plugin_lifecycle.md) §1): pluginlib constructs it through
its no-argument constructor, so the constructor must be cheap and infallible, and every fallible
thing — parameter parsing, solver setup, model loading — happens in `Init`.

```cpp
#include <pluginlib/class_list_macros.hpp>

#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"   // your stage's interface

namespace my_stage_plugins {

class MyStagePlugin final : public StagePlugin<SwingLegControllerInterface> {
 public:
  // No constructor written at all: the implicit one is cheap and cannot throw.

  void Init(StageInit init) override {
    // init.model / init.state: clones you take ownership of, already valid.
    // init.params:             the host's parameter map, flat name -> value.
    // Throw StageInitError, naming the key, on anything you cannot start without.
    model_ = std::move(init.model);
  }

  // ... the frozen interface methods, per stage_contracts.md §4 ...

  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    return false;  // `false` = "not my key". See below — this return value is load-bearing.
  }

 private:
  std::unique_ptr<ModelInterface> model_;
};

}  // namespace my_stage_plugins

// The base string must match stage_plugin_bases::kSwingLegController and your XML exactly.
PLUGINLIB_EXPORT_CLASS(my_stage_plugins::MyStagePlugin, StagePlugin<SwingLegControllerInterface>)
```

Read [`passthrough_slc_plugin.cpp`](../../ws/src/examples/example_stage_plugins/src/passthrough_slc_plugin.cpp)
next: it is the smallest *complete* implementation of a stage contract, written to be read as a
template rather than as an algorithm. For a stage that wraps an existing algorithm class instead of
implementing one, the stock wrappers in
[`ws/src/controllers/src/plugins/`](../../ws/src/controllers/src/plugins/) are the shape to copy —
a `unique_ptr` member built in `Init` and about seven one-line forwarders.

### The parameter contract you are responsible for

`controllers` has helpers for reading `StageInit::params` (`Require<T>` / `GetOr<T>` in
`src/plugins/plugin_param_utils.hpp`), but they are **internal to that package and not installed**,
so out-of-package code writes the read itself — about ten lines for one optional key. Installing
them is [#40](https://github.com/thalesasoares/dfki-quad/issues/40); until then, what you must
reproduce is not the helpers' shape but their **contract**:

| Case | Required behaviour |
|---|---|
| Required key absent | throw `StageInitError` **naming the key** |
| Value present but malformed or wrongly typed | throw `StageInitError` **naming the key** |
| Optional key absent | use the stage's own default — the default lives in the stage, never in the host |
| `SetParameter` with a key you do not own | return `false` |

The "naming the key" part is the whole point. The loader layer exists so that a misconfigured
pipeline refuses to start instead of walking with the wrong controller
([`stage_loading.md`](stage_loading.md) §1); an error that does not say *which* key is barely better
than the silent fallback that design rules out. `SetParameter` returning `false` is equally
load-bearing: the host uses that return value to warn a human that a runtime parameter change did
not apply, so silently accepting an unknown key makes a typo look like a working setting.

`ReadLogPeriod` in the example plugin is a worked instance of all four rows.

## 3. Build it

Three traps, in the order you hit them. The first two are hard build failures **whose error messages
point at the wrong thing**; the third fails nothing at all, which is what makes it the dangerous one.
All three are annotated at their site in the example's
[`CMakeLists.txt`](../../ws/src/examples/example_stage_plugins/CMakeLists.txt), which is the file to
copy.

Your `package.xml` needs very little, because `controllers` re-exports what the stage headers need
([`plugin_discovery.md`](plugin_discovery.md) §1):

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>controllers</depend>   <!-- re-exports common, Eigen3, rclcpp, interfaces, pluginlib -->
<depend>pluginlib</depend>     <!-- named explicitly: you call its macros yourself -->
<test_depend>ament_cmake_gtest</test_depend>
```

### 3.1 Never link `controllers`

The obvious first thing to write is also the wrong one:

```cmake
find_package(controllers REQUIRED)
ament_target_dependencies(my_plugin controllers pluginlib rclcpp Eigen3)   # ✗ do not
```

```
CMake Error at CMakeLists.txt:45 (add_library):
  Target "my_plugin" links to target "drake::drake" but the target was not
  found.  Perhaps a find_package() call is missing for an IMPORTED target, or
  an ALIAS target is missing?
```

**Do not go looking for drake.** The error names it, but drake is not your dependency and pulling it
in would drag an enormous closure into a package that needs none of it. `ament_target_dependencies`
puts `${controllers_LIBRARIES}` on the link line, and `controllers` re-exports `common`, whose own
closure is `common` → `quad_model` → `drake::drake`.

```cmake
target_include_directories(my_plugin SYSTEM PRIVATE ${controllers_INCLUDE_DIRS})   # ✓ headers
ament_target_dependencies(my_plugin pluginlib rclcpp)                              # ✓ real links
```

**Why headers really are enough** — this is the part worth internalising, because it explains the
plugin model rather than just unblocking the build: **a stage plugin references no compiled symbol
of `controllers` or `common`.** Everything it touches is a pure-virtual interface
(`ModelInterface`, `StateInterface`, your stage interface), a header-only helper (`StageInit`,
`StageInitError`, `StageLoader`) or a plain data struct (`FeetTargets`, `GaitSequence`). The *host*
supplies every implementation at runtime through the vtable.

This is also the out-of-package form of a rule `controllers` already keeps internally — "`common` is
include-only, never linked" ([`stock_plugins.md`](stock_plugins.md) §4) — and it is what keeps the
non-PIC static `libfmt.a` closure off a shared object, which would otherwise fail to link at all
([`stage_loading.md`](stage_loading.md) §6). Same underlying fact, one package boundary further out.

### 3.2 Name Eigen through its imported target

With §3.1 fixed, the next failure:

```
In file included from .../controllers/include/mit_controller/stage_plugin.hpp:10,
                 from .../src/my_plugin.cpp:38:
.../common/include/common/model_interface.hpp:3:10: fatal error: Eigen/Geometry: No such file or directory
    3 | #include <Eigen/Geometry>
      |          ^~~~~~~~~~~~~~~~
```

```cmake
target_link_libraries(my_plugin Eigen3::Eigen)   # header-only: adds an include path, links nothing
```

`${controllers_INCLUDE_DIRS}` carries `common`'s and `interfaces`' include directories, but **not**
Eigen's: Eigen ships an INTERFACE target rather than ament include variables, so
`ament_export_dependencies(Eigen3 …)` populates no include directory for a consumer to inherit. The
re-export still gives you the `Eigen3::Eigen` *target* — that is why you need no
`find_package(Eigen3)` of your own — just not the include path. This is a real (small) gap in the
re-export promise of [`plugin_discovery.md`](plugin_discovery.md) §1, stated here rather than fixed
silently, because you will hit it in your first five minutes.

### 3.3 Match the host's optimisation flags

Nothing fails if you skip this. That is exactly why it is on the checklist in [§7](#7-checklist-before-you-pr)
with its consequence attached:

```cmake
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
endif ()
```

A stage runs inside the 100 Hz / 500 Hz control loops. A plugin `.so` built at default or debug
optimisation while the node is built at `-Ofast` is **an on-robot regression that no test catches**:
every unit test passes, discovery passes, the simulator probably looks fine, and the loop timing
degrades on hardware. The stock plugins get these flags for free from `controllers`' directory
scope; your package does not, and nothing will tell you.

One related line to copy rather than clean up:

```cmake
target_compile_options(my_plugin PRIVATE -Wall -Wextra -Wpedantic -Werror -Wno-error=maybe-uninitialized)
```

`SHARED` forces `-fPIC`, and GCC + `-Ofast` + `-fPIC` makes Eigen emit a `-Wmaybe-uninitialized`
false positive that the non-PIC node never hits. Only that one warning is demoted; everything else
stays a hard error ([`stage_loading.md`](stage_loading.md) §6).

### 3.4 The whole file

The three rules above plus ordinary ament boilerplate, assembled — this is a complete, working
`CMakeLists.txt` for a stage plugin package, and the [§4](#4-export-it) lines are already in it:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_stage_plugins)

# §3.3 — mirror the host's Release flags. Not optional; see the checklist.
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
endif ()

find_package(ament_cmake REQUIRED)
find_package(controllers REQUIRED)   # the only pipeline package you name
find_package(pluginlib REQUIRED)     # you call its macros by name, so find it explicitly

add_library(my_stage_plugins SHARED src/my_plugin.cpp)   # SHARED: pluginlib dlopens it

target_include_directories(my_stage_plugins SYSTEM PRIVATE ${controllers_INCLUDE_DIRS})  # §3.1
ament_target_dependencies(my_stage_plugins pluginlib rclcpp)                             # §3.1
target_link_libraries(my_stage_plugins Eigen3::Eigen)                                    # §3.2
target_compile_options(my_stage_plugins PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wno-error=maybe-uninitialized)                 # §3.3

install(TARGETS my_stage_plugins DESTINATION lib)                                        # §4
pluginlib_export_plugin_description_file(controllers plugins/my_stage_plugins.xml)       # §4

ament_package()
```

## 4. Export it

Two things make your class discoverable: a description XML, and one CMake line.

```xml
<class_libraries>
  <library path="my_stage_plugins">
    <class name="my_stage"
           type="my_stage_plugins::MyStagePlugin"
           base_class_type="StagePlugin&lt;SwingLegControllerInterface&gt;">
      <description>One or two sentences: what it does, and anything a human selecting it should
        know before pointing it at a robot.</description>
    </class>
  </library>
</class_libraries>
```

- **`path` is your CMake target name with no `lib` prefix.** pluginlib adds the platform's prefix
  and suffix itself. It accepts the prefixed spelling but warns
  (`given plugin name 'libX' should be 'X' for better portability`). Five of the six stock XMLs in
  `controllers` still carry the prefix and predate this rule — copy the spelling above, not theirs
  ([#41](https://github.com/thalesasoares/dfki-quad/issues/41)).
- **`<` must be XML-escaped** as `&lt;` / `&gt;` in `base_class_type`. tinyxml2 unescapes before
  pluginlib compares against the C++ string, so the two spellings match
  ([`stage_loading.md`](stage_loading.md) §5).
- **Name the id for the human who reads it in a config.** If your stage is a demo or a diagnostic
  rather than production locomotion, say so in the id — the example uses `example_passthrough_slc`
  for precisely that reason.

Then the one line that does the work, plus the install rule it depends on:

```cmake
install(TARGETS my_stage_plugins DESTINATION lib)
pluginlib_export_plugin_description_file(controllers plugins/my_stage_plugins.xml)
```

**The first argument is `controllers`, not your project.** It names the ament *resource*
(`controllers__pluginlib__plugin`) that the host's `StageLoader` searches, because `controllers` owns
the base classes. pluginlib resolves that resource across every installed package, so registering
there is sufficient and nothing in `controllers` is edited — that absence of an allowlist is the
whole mechanism ([`plugin_discovery.md`](plugin_discovery.md) §3a).

The `.so` goes to **your** package's `lib/`, because pluginlib resolves `<library path>` relative to
the install prefix of the package that owns the XML — which is yours, not `controllers`'.

## 5. Select it

Your stage is chosen by one string, the `<stage>.type` key from the [§1](#1-pick-your-stage) table.
Nothing in `controllers` changes:

```bash
ros2 launch controllers mit_controller.launch.py sim:=go2 slc:=my_stage
```

That argument exists for each of the six stages (M4.4, [#19](https://github.com/thalesasoares/dfki-quad/issues/19)).
If your stage also needs *parameters* — and past the smallest example it will — put both in an
overlay file instead, layered over the stock config without replacing it:

```yaml
# my_stage_overlay.yaml
mit_controller_node:
  ros__parameters:
    slc:
      type: my_stage
    my_stage:
      my_key: 1.0
```

```bash
ros2 launch controllers mit_controller.launch.py sim:=go2 stage_overlay:=my_stage_overlay.yaml
```

Both are described in full, with the precedence between them, in
[`stage_overlays.md`](stage_overlays.md).

Misspell it and the pipeline **refuses to start**, naming what was available — there is no silent
fallback to some other stage, which is the entire reason the loader exists:

```
no stage plugin named 'my_stge' is declared for base
'StagePlugin<SwingLegControllerInterface>'; declared stage plugins:
'bezier_swing', 'example_passthrough_slc', 'my_stage'
```

Seeing your own id in that list is also the quickest confirmation that [§4](#4-export-it) worked.

### Your stage's own parameters are not declared by the host

`slc.type` and its five siblings are declared by the host, so overriding them works. **Keys that
only your stage knows about are a different matter**, and this catches people:

The host builds `StageInit::params` from `list_parameters()`, which returns only parameters the node
has **declared** — and the node declares its own ~88 keys explicitly, with neither
`automatically_declare_parameters_from_overrides` nor `allow_undeclared_parameters` enabled
(`mit_controller_node.cpp`, `MakeStageInit`). A key the host has never heard of therefore behaves
in neither of the two ways you would expect:

- **At launch**, `-p my_stage.gain:=2.0` is accepted on the command line without complaint, but the
  parameter is **never declared** and so never reaches your `Init`, which duly applies its own
  default. Asking for it afterwards answers `Parameter not set`.
- **At runtime**, `ros2 param set` fails outright:
  ```
  Setting parameter failed: parameter 'my_stage.gain' cannot be set because it was not declared
  ```

The first case is the dangerous one: it looks like it worked.

So today an out-of-package stage is **configurable only through the keys the host already declares**
(the shared ones your stage may legitimately read, listed per stage in
[`stock_plugins.md`](stock_plugins.md) §3). Design accordingly: give every optional key a default
that is correct on its own, and do not build a stage that is unusable until someone sets a private
key. Note that this affects the *host* path only — your own tests construct `StageInit` directly and
so can exercise every key freely ([§6](#6-test-it)).

This is a known gap, not a design intent: [`plugin_lifecycle.md`](plugin_lifecycle.md) §3 records the
decision to postpone `automatically_declare_parameters_from_overrides(true)` — an auto-declared
parameter takes its type from the YAML literal, so a `200` written where a stage expects a `double`
would turn a working config into a `StageInitError` — and assigns the switch to the YAML/parameter
work of [#21](https://github.com/thalesasoares/dfki-quad/issues/21) (M5.2).

## 6. Test it

Test through **`StageLoader` with production discovery** — the same API and the same ament resource
the running node uses, with no explicit XML path:

```cpp
StageLoader<SwingLegControllerInterface> loader{stage_plugin_bases::kSwingLegController};
auto plugin = loader.Load(stage_selection::kSwingLegControllerTypeKey, MakeInit("my_stage", params));
```

That is what makes the test evidence about your *plugin* rather than about your class: it proves the
XML is installed, the resource is registered, the library is findable and `dlopen`s, the base string
matches, and `Init` runs. A test that constructs your class directly proves none of those.
[`test_passthrough_slc_plugin.cpp`](../../ws/src/examples/example_stage_plugins/test/test_passthrough_slc_plugin.cpp)
is the worked pattern, and pins the five things worth pinning: discovery across the package boundary
(without displacing the stock plugin), the full load → `Init` → one-cycle lifecycle, fail-fast on a
malformed parameter with the key named, `SetParameter` returning `true` only for owned keys, and the
unknown-selection error listing your plugin.

Two practical notes:

- **You write your own test doubles.** `controllers` keeps `BrickModel`/`BrickState` private, so you
  cannot borrow them. This is not a missing export to work around — writing the fakes is the honest
  demonstration that the *exported* pure-virtual interfaces suffice to test a stage in isolation.
  There is a worked `FakeModel`/`FakeState` pair in the example's test file to start from.
- **The test process needs the install space on `AMENT_PREFIX_PATH`**, because the resource lives
  there:
  ```cmake
  ament_add_gtest(test_my_stage test/test_my_stage.cpp
          APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  add_dependencies(test_my_stage my_stage_plugins)   # the .so must exist before the test dlopens it
  ```
  Give the test the same include-only treatment as the library ([§3.1](#31-never-link-controllers));
  it drives everything through pure-virtual interfaces too.

If your stage consumes model updates, [`model_update_broadcast.md`](model_update_broadcast.md) §3 is
the contract for `UpdateModel` — the host broadcasts an adapted model to every stage that wants one,
and your `UpdateModel` must be cheap and must not assume it is ever called.

## 7. Checklist before you PR

| ✅ | Check | Why it matters |
|---|---|---|
| ☐ | Release flags mirror the host (`-O3` aarch64 / `-Ofast` otherwise) | An under-optimised `.so` is an invisible on-robot loop-timing regression that **no test catches** ([§3.3](#33-match-the-hosts-optimisation-flags)) |
| ☐ | No link against `controllers` or `common` — include paths only | The drake link error means you got this wrong ([§3.1](#31-never-link-controllers)) |
| ☐ | `Eigen3::Eigen` named explicitly | It does not arrive through the re-export ([§3.2](#32-name-eigen-through-its-imported-target)) |
| ☐ | `<library path>` has no `lib` prefix | Portability warning in every downstream log otherwise ([§4](#4-export-it)) |
| ☐ | `base_class_type` identical in XML, `PLUGINLIB_EXPORT_CLASS` and `stage_plugin_bases::k*` | A mismatch is not a compile error — it is a class that is never found ([§1](#1-pick-your-stage)) |
| ☐ | `Init` throws `StageInitError` **naming the key** | The fail-fast contract the whole loader layer exists to provide ([§2](#the-parameter-contract-you-are-responsible-for)) |
| ☐ | `SetParameter` returns `false` for keys you do not own | The host uses it to warn that a change did not apply ([§2](#the-parameter-contract-you-are-responsible-for)) |
| ☐ | Every optional key has a working default | Private keys do not reach your stage through the host today ([§5](#your-stages-own-parameters-are-not-declared-by-the-host)) |
| ☐ | No heap allocation and no I/O on any per-cycle path | You are in a 100 Hz or 500 Hz loop; the example's §5 is the reference discipline |
| ☐ | Tests load through `StageLoader` with production discovery | Anything less does not test the plugin, only the class ([§6](#6-test-it)) |
| ☐ | Selected in a Go2 sim run at least once | Discovery, load, `Init` and the loop all hold together only in the real host |
| ☐ | **PR base: `feature/modularization_central`** | Where the modular pipeline lives while the milestones are in flight; it merges to `dev` as a whole later |

## 8. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #18 | [M4.3] Contributor guide: add a control stage | **This document** |
| #17 | [M4.2] Example passthrough / logging SLC plugin | The worked example this guide walks through |
| #16 | [M4.1] Bio gait sequencer via plugin param | The in-package half of the same proof |
| #19 | [M4.4] Launch overlay to swap one stage | Provides the launch argument and overlay file of §5 ([`stage_overlays.md`](stage_overlays.md)) |
| #21 | [M5.2] Parameter reference for all stage keys | Owns the key-by-key reference §2 and §5 only contract |
| #40 | Install the stage parameter helpers for out-of-package plugins | Would replace the hand-rolled reads of §2 |
| #41 | Portable `<library path>` spelling in the stock XMLs | Would remove §4's footnote about the stock files |
