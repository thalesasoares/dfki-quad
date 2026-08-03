# Stage Contract Test / Compile Smoke

**Status:** added in M1.5 (issue #5) ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`stage_contracts.md`](stage_contracts.md) · [`pipeline_types.md`](pipeline_types.md)

`stage_contracts.md` specifies the *methods* each control-pipeline stage must provide and
`pipeline_types.md` the *data* they exchange. This document describes the automated guard that keeps
those two specifications from silently drifting away from the code: a fake-stub contract test that
fails the build when a stage interface changes shape.

It is the last piece of milestone **M1 — Contracts**. Its purpose is forward-looking: it freezes the
seven stage APIs *before* the M2 plugin-host refactor (issue #9) starts moving them behind
`pluginlib`, so a breaking signature change during that work is caught at compile time rather than by
a robot on the floor.

## 1. What it is

Two files under `ws/src/controllers/test/`:

| File | Role |
|---|---|
| `fake_stages.hpp` | A minimal `final` stub for every stage interface (the five stages of `stage_contracts.md` §4, the sixth contact-logic stage §4.6, and the small `GaitInterface` seam). Each method is `override`. |
| `test_stage_contracts.cpp` | Compile-time `static_assert`s over the contract shape, plus gtest cases that drive each fake once through a `std::unique_ptr<Interface>`. |

The stubs are behaviour-free by design — the concrete stages have their own behaviour and their own
tests. This is a *shape and plumbing* guard, not a behavioural one.

## 2. What it guards, and how each break is caught

| If someone… | …this breaks |
|---|---|
| changes a stage method's signature (params/return/const) | the stub's `override` no longer overrides → **compile error** in `fake_stages.hpp` |
| removes or renames a stage method | same — the `override` fails to bind |
| adds a new pure virtual to an interface | the `final` stub turns abstract → `static_assert(!std::is_abstract_v<Fake…>)` fails and the `std::make_unique<Fake…>` in the test fails |
| drops a virtual destructor from an interface | `static_assert(std::has_virtual_destructor_v<…>)` fails (guards the G4 regression fixed in M1.1) |
| makes an interface constructible on its own | `static_assert(!std::is_default_constructible_v<…>)` fails |
| changes `N_LEGS` without updating the contact aggregates | the `std::tuple_size_v<…> == N_LEGS` asserts fail (migrated verbatim from the retired M1.4 check) |
| desyncs `ContactLogicInterface::{FootContacts,Wrenches}` from `WBCInterface::{FootContact,Wrenches}` | the `std::is_same_v<…>` asserts fail — the host passes these straight through (§4.6), so the layouts must match. Since #13 these asserts are the *only* thing keeping the two spellings one type: neither interface includes the other, and there is no longer a template parameter to blame a mismatch on |
| changes the plugin lifecycle layer (`StagePlugin::Init`, `StageInit`, `StageInitError` — [`plugin_lifecycle.md`](plugin_lifecycle.md)) | `FakePluginGaitSequencer` stops compiling, the `StagePlugin<…>` shape asserts fail, or the `StagePluginLifecycle` gtest cases fail |

Because every one of these is a **compile** failure, the ordinary `colcon build` (CI's `cbg`) is
already the compile smoke the issue asks for. `colcon test` only adds the runtime pass, which walks
each fake through the documented `Update*`→`Get*` call order and checks the `SetParameter`
false-for-unknown-key convention.

The `WBCInterface` block used to be instantiated for **all three** joint command types in
`joint_commands.hpp`, because the interface was a class template (gap G8) and none of the three
instantiations could be allowed to rot. Since #13 (M3.2) de-templated it there is one interface, and
the block pins instead what replaced the template: the signatures of both getters and of
`SupportedCommandMode()`, plus a `FakeWBC` constructed in each mode — including the contract that the
getter of the *other* mode fails rather than returning junk the host might publish.

## 3. What it does *not* guard

- **Stage behaviour.** The fakes compute nothing; they only satisfy the contract. Correctness of
  `MPC`, `SwingLegController`, etc. is out of scope.
- **Export self-containment.** That axis stays with `src/tools/pipeline_types_surface_check.cpp`
  (M1.3, extended in M2.1), which compiles the exported type *and* stage interface headers under a
  *restricted, export-only* include path to prove an out-of-package plugin can include them. This
  test deliberately uses the package's normal include paths.
- **Plugin discovery.** That the plugin description XML is exported and a `ClassLoader` finds each
  stage base is `test/test_plugin_discovery.cpp` (M2.1, issue #6) — see
  [`plugin_discovery.md`](plugin_discovery.md) §4. Different axis: the ament resource and XML, not the
  interface shape.
- **Plugin loading.** That a `type:` string resolves to an initialised stage, and that a missing or
  invalid one fails loudly instead of falling back, is `test/test_stage_loader.cpp` (M2.2, issue #7) —
  see [`stage_loading.md`](stage_loading.md) §7. This test drives a fake through `Init` **by hand**;
  that one drives real, `dlopen`-ed plugins through `StageLoader`.

## 4. How to run it

The test needs only `common`, `interfaces`, `rclcpp` and `Eigen3` — no drake, acados, ARC-OPT or
hardware — so it builds and runs headless on both CI lanes (x86_64 and the `WITHOUT_DRAKE` aarch64
onboard build).

```bash
# Compile smoke — this alone catches every API break above.
# Runs implicitly in CI via the `cbg` build.
colcon build --packages-select controllers --cmake-args -DROBOT_NAME=go2

# Run the contract test (adds the runtime call-order / SetParameter pass).
colcon test --packages-select controllers --ctest-args -R test_stage_contracts
colcon test-result --verbose
```

## 5. History

This test supersedes the narrower `src/tools/contact_logic_interface_check.cpp` from M1.4, whose
own comment scheduled the replacement; its `N_LEGS` sizing asserts are migrated here and the file is
removed. The M1.3 surface check (`pipeline_types_surface_check.cpp`) is kept — it guards the
different, export-only axis described in §3.

## 6. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #1 | [M1.1] Audit and freeze stage interface APIs | Asserts the §4 method tables it froze; guards its G4 fix |
| #3 | [M1.3] Shared pipeline data types package surface | Complements the surface check (different axis, §3) |
| #4 | [M1.4] Define `ContactLogicInterface` | Absorbs and removes its compile check |
| #5 | [M1.5] Contract tests / compile smoke | **This document** |
| #6 | [M2.1] pluginlib dependency and plugin description XML | Exported the interface headers (extending the surface check) and added `test_plugin_discovery.cpp` — [`plugin_discovery.md`](plugin_discovery.md) |
| #7 | [M2.2] Stage plugin base + loader helper | Reuses `fake_stages.hpp`'s plugin stub pattern; adds `test_stage_loader.cpp` on the loading axis (§3) — [`stage_loading.md`](stage_loading.md) |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | The refactor this test protects |
| #13 | [M3.2] Runtime WBC / command-type profile | **Done.** Removed the template the WBC block worked around (gap G8); the block now pins the two getters and the mode query |
| #17 | [M4.2] Example passthrough / logging plugin | Starts from `FakeContactLogic`'s pass-through `Reconcile` |
