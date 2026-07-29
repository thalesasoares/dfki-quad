# `example_stage_plugins` — adding a control stage from outside the host

**Status:** added in M4.2 (issue #17) ·
**Applies to:** `ws/src/examples/example_stage_plugins` ·
**Companion documents:** [`plugin_lifecycle.md`](../../../../doc/modularity/plugin_lifecycle.md) — the `StagePlugin` contract implemented here ·
[`plugin_discovery.md`](../../../../doc/modularity/plugin_discovery.md) — the description-XML schema and the ament resource ·
[`stage_loading.md`](../../../../doc/modularity/stage_loading.md) — the loader that finds this package ·
[`stock_plugins.md`](../../../../doc/modularity/stock_plugins.md) — the in-package plugins this one sits beside

This package ships **`example_passthrough_slc`**, a demonstration swing-leg-controller stage. It
exists to prove one claim, which is M4's exit criterion: **a control stage can be added from a
separate package, selected by one YAML value, with no edit to the host.** Nothing in `controllers`
knows this package exists.

## 1. What the plugin does — and what it is *not* for

> **This is not production locomotion.** `example_passthrough_slc` commands every foot to stay where
> it already is. The robot stands. It does not step, and a walk command while this stage is loaded is
> a documented no-op, not a bug. Do not use it to move a real robot.

What it *is* good for:

- **A worked example.** It is the smallest complete implementation of the swing-leg-controller
  contract, so it reads as a template rather than as an algorithm.
- **A diagnostic probe.** Load it and the swing stage stops contributing motion, while the throttled
  log reports what the gait sequencer is scheduling. It answers "is the gait sequencer asking for
  what I think it is?" without swing trajectories confusing the picture.

Behaviour, per the frozen `SwingLegControllerInterface`:

| Method | Behaviour |
|---|---|
| `GetFeetTargets` | Each foot's current world position; zero velocity, zero acceleration. |
| `GetProgress` | Scheduled stance → `STANCE` at progress `0`. Scheduled swing → `REACHED` at progress `1` — a swing that is instantly complete where it started, which is what keeps the downstream contact logic benign. |
| `GetCurrentTrajs` | Degenerate: `start == end == ` the held position. |

## 2. Parameters

| Key | Req? | Type | Default | Notes |
|---|---|---|---|---|
| `example_passthrough_slc.log_period` | optional | `double` | `0.0` | Seconds between throttled log lines. `0` disables logging. Negative or wrongly typed → `StageInitError` naming the key. Also a **runtime** key: `SetParameter` applies it and returns `true`. |

Logging defaults to off. A demo plugin that spams the console by default is one people mute rather
than read.

## 3. How to select it

Nothing in `controllers` is edited. Override `slc.type` at launch:

```bash
ros2 run controllers mitcontrollernode --ros-args \
    --params-file <your usual mit_controller_sim_go2.yaml> \
    -p slc.type:=example_passthrough_slc \
    -p example_passthrough_slc.log_period:=1.0
```

or set it in a params file of your own that overlays the stock one. (A first-class launch overlay for
swapping a single stage is #19, M4.4; until then a parameter override is the mechanism.)

Change the log cadence while it runs:

```bash
ros2 param set /mit_controller_node example_passthrough_slc.log_period 5.0
```

Misspell the type and the pipeline refuses to start, naming what *was* available — never a silent
fallback to some other stage:

```
no stage plugin named 'example_pasthrough_slc' is declared for base
'StagePlugin<SwingLegControllerInterface>'; declared stage plugins:
'bezier_swing', 'example_passthrough_slc'
```

## 4. How it is wired — the whole checklist

Adding a stage from outside the host is four things. All four are in this package; none are anywhere
else.

1. **A class deriving from `StagePlugin<Interface>`** ([`src/passthrough_slc_plugin.cpp`](src/passthrough_slc_plugin.cpp)),
   default-constructible, doing its real setup in `Init`, exported with `PLUGINLIB_EXPORT_CLASS`.
2. **A description XML** ([`plugins/example_slc_plugins.xml`](plugins/example_slc_plugins.xml))
   naming the class, the library and the base-class-type string.
3. **`pluginlib_export_plugin_description_file(controllers plugins/example_slc_plugins.xml)`** —
   the one line that makes it discoverable. The first argument is `controllers`, *not* this project:
   it names the ament resource (`controllers__pluginlib__plugin`) the host's `StageLoader` searches,
   because `controllers` owns the base classes. pluginlib resolves that resource across every
   installed package, so registering here is sufficient. There is no allowlist to join.
4. **The library installed to this package's `lib/`**, because pluginlib resolves `<library path>`
   relative to the install prefix of the package that owns the XML.

### Three things that are easy to get wrong

These cost real time to diagnose, so they are called out rather than left to be rediscovered. All
three are commented at their site in [`CMakeLists.txt`](CMakeLists.txt).

- **Consume `controllers` as an include path, never as a link dependency.**
  `ament_target_dependencies(<target> controllers)` fails with
  `links to target "drake::drake" but the target was not found`, because it puts
  `${controllers_LIBRARIES}` — the whole re-exported closure, `common` → `quad_model` → `drake` — on
  the link line. A stage plugin needs none of it: everything it touches is a pure-virtual interface,
  a header-only helper or a plain data struct, and the *host* supplies the implementations at runtime
  through the vtable. Use `target_include_directories(... ${controllers_INCLUDE_DIRS})` instead.
  This is the out-of-package form of the rule `controllers` already keeps internally — "`common` is
  include-only, never linked" ([`stock_plugins.md`](../../../../doc/modularity/stock_plugins.md) §4) —
  and it is what keeps the non-PIC static-`libfmt` closure off a shared object
  ([`stage_loading.md`](../../../../doc/modularity/stage_loading.md) §6).
- **Name Eigen through its imported target.** `${controllers_INCLUDE_DIRS}` carries `common`'s and
  `interfaces`' include directories but not Eigen's — Eigen ships an INTERFACE target rather than
  ament include variables — so the interface headers' `#include <Eigen/Geometry>` will not resolve.
  `target_link_libraries(<target> Eigen3::Eigen)` fixes it and links nothing.
- **Match the host's optimisation flags.** A stage runs in the 100 Hz / 500 Hz control loops. A
  plugin `.so` built at a lower optimisation level than the node is an invisible on-robot regression
  that no test catches, so this package mirrors `controllers`' Release flags exactly
  (`-O3` on aarch64, `-Ofast` otherwise).

## 5. Performance

A stage that visibly perturbed loop timing would teach the wrong lesson, so this one is cheap by
construction:

- **Per 500 Hz cycle:** four `CalcFootPositionInWorld` calls plus fixed-size copies — strictly less
  work than the stock `bezier_swing`, which does the same forward kinematics and *then* evaluates a
  Bezier spline per leg.
- **Per 100 Hz cycle:** four boolean reads and the throttle check.
- **No heap allocation on any per-cycle path** (every member is a fixed-size array), and no I/O
  except when the log period has actually elapsed. With logging off — the default — the throttle is
  one predictable branch.
- Built with the node's own Release flags, so none of the above is silently deoptimised.

Selecting this plugin is also the only way it costs anything. It is `dlopen`ed only if `slc.type`
names it; otherwise the sole effect of installing this package is one extra entry in the ament
resource scan at bring-up, which happens once, off-loop.

## 6. Tests

[`test/test_passthrough_slc_plugin.cpp`](test/test_passthrough_slc_plugin.cpp) drives the plugin
through the host's own `StageLoader`, built on ament discovery with no explicit XML path — the same
API and the same resource the running node uses. It pins:

1. the plugin is discoverable across the package boundary, and does **not** displace `bezier_swing`;
2. the full lifecycle — load, `Init`, one interface cycle — including that `Init` seeds the held
   positions from the state clone, so the first `GetFeetTargets` does not command the feet to the
   world origin;
3. fail-fast on a negative or wrongly typed `log_period`, with the key named in the message;
4. `SetParameter` returning `true` for the key it owns and `false` for everything else, including
   stock SLC keys — the host relies on that return value to warn that a change did not apply;
5. the loader's unknown-selection error listing this plugin among the alternatives.

The model and state doubles are written from scratch in that file rather than borrowed:
`controllers` keeps its `BrickModel`/`BrickState` private, so a third party cannot reuse them.
Writing them is the honest demonstration that the *exported* pure-virtual interfaces suffice to test
a stage in isolation.

## 7. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #17 | [M4.2] Example passthrough / logging MPC or SLC plugin | **This package** |
| #16 | [M4.1] Enable Bio gait sequencer via plugin param | Proved *in-package* stage addition; this proves the out-of-package half |
| #18 | [M4.3] Contributor guide: add a control stage | Will generalise §4 of this file into the guide |
| #19 | [M4.4] Launch overlay to swap one stage | Will replace the parameter override of §3 with a first-class overlay |
| #21 | [M5.2] Parameter reference for all stage keys | Consumes §2 |
