# Stage Loading: resolving `type:` to an initialised stage

**Status:** added in M2.2 (issue #7) ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`plugin_lifecycle.md`](plugin_lifecycle.md) — the `StagePlugin` / `Init`
contract this drives · [`plugin_discovery.md`](plugin_discovery.md) — the dependency, exported surface
and XML schema this consumes · [`stage_contracts.md`](stage_contracts.md) — the frozen stage APIs

[`plugin_discovery.md`](plugin_discovery.md) made the stage plugin schema *discoverable* and
[`plugin_lifecycle.md`](plugin_lifecycle.md) specified how a stage *comes to life* once instantiated.
This document covers the step between them: **`mit_controller/stage_loader.hpp`**, the one place that
turns a `type:` string in the YAML into a stage the host can drive.

Issue #7's title says "stage plugin base + loader helper". The base — `StagePlugin<Interface>`,
`StageInit`, `StageInitError` — was specified and implemented ahead of M2.2 (commit `921f69d`) and
exported in M2.1, so M2.2 is the **loader half**: `StageLoader`, the naming convention below, and the
tests that pin the failure behaviour.

M2.2 still ships **no production plugin classes**. Every declared-class list is empty until M2.3
(#8), by design — `test_plugin_discovery.cpp` asserts it.

## 1. The API

```cpp
template <class StageInterface>
class StageLoader {
 public:
  using Plugin    = StagePlugin<StageInterface>;
  using PluginPtr = pluginlib::UniquePtr<Plugin>;

  explicit StageLoader(std::string base_class_type, std::vector<std::string> plugin_xml_paths = {});

  PluginPtr Load(const std::string& type_key, StageInit init);  // resolve → create → Init
  PluginPtr Create(const std::string& lookup_name);             // create only (§3)

  std::vector<std::string> DeclaredClasses() const;
  const std::string& BaseClassType() const;
};
```

One instantiation per stage base, constructed with the matching
`stage_plugin_bases::k*` string (§5). `Load` performs the whole start-up path:

| Step | Failure |
|---|---|
| `type_key` present in `init.params` | `StageLoadError` naming the key **and listing the declared plugins** |
| value is a non-empty string | `StageLoadError` naming the key and the actual type |
| value names a declared class | `StageLoadError` naming the bad value and listing the declared plugins |
| pluginlib loads the library and constructs the class | `StageLoadError` carrying pluginlib's message |
| `Init(std::move(init))` returns | `StageInitError` naming the stage, the selecting key and the stage's own message |

**No step has a fallback.** There is no default stage, no "closest match", no continuing with the
previously loaded instance. That is issue #7's second acceptance criterion, and it is the reason the
class exists at all: a quadruped that walks with the wrong controller because a parameter was
misspelled is a worse outcome than one that refuses to start.

Errors carry the *alternatives*, not just the complaint — a wrong `type:` should be a one-line diff,
not a hunt through the XML. When nothing at all is declared the message says so explicitly, which is
the symptom of a plugin library that failed to install rather than of a typo.

## 2. Two error types, on purpose

| Exception | Meaning | Thrown by |
|---|---|---|
| `StageLoadError` | "the stage you asked for does not exist / cannot be loaded" | the loader |
| `StageInitError` | "the stage you asked for exists but cannot configure itself" | the stage's `Init`, re-thrown by the loader with the stage named |

Both are fatal for pipeline bring-up, so a host that only wants to abort can catch
`std::runtime_error`. The distinction is for everyone else: it tells a human reading a crash log
whether to fix the `type:` line or the stage's parameters, and it lets M2.4 report the two cases
differently if it chooses. `pipeline_types_surface_check.cpp` `static_assert`s that neither type
derives from the other, so the distinction cannot quietly collapse.

## 3. Loader lifetime, and why `Create` exists

Destroying a `pluginlib::ClassLoader` unloads the library; any instance still alive then holds a
dangling vtable ([`plugin_lifecycle.md`](plugin_lifecycle.md) §5). Two things enforce that here:

- **`StageLoader` is non-copyable and non-movable.** A loader cannot be moved out from under the
  instances it created.
- **The host declares loaders before stage pointers.** Members are destroyed in reverse declaration
  order, so this makes the ordering correct by construction. M2.4 (#9) must follow it:

  ```cpp
  StageLoader<GaitSequencerInterface> gs_loader_{stage_plugin_bases::kGaitSequencer};  // declared FIRST
  StageLoader<GaitSequencerInterface>::PluginPtr gs_;                                  // destroyed FIRST
  ```

**The stage member is a `PluginPtr`, not a `std::unique_ptr<Interface>`.** pluginlib's deleter is
part of the pointer's type — it decrements the loaded library's reference count — so moving into a
plain `unique_ptr` would silently substitute the default deleter and destroy the stage outside
pluginlib's bookkeeping. This is the one place M2.4 deviates from the "host owns
`unique_ptr<Interface>`" phrasing of [`stage_contracts.md`](stage_contracts.md) §3, and it is a
declaration-type change only: `StagePlugin<I>` **is** an `I`, so every call site still goes through
the frozen interface and the per-cycle dispatch is identical (§6).

`Create` returns an instance **without** calling `Init`. That is the reconfiguration path of
[`plugin_lifecycle.md`](plugin_lifecycle.md) §5 — build the replacement, `Init` it off-loop, swap the
pointer under the stage's lock, destroy the old instance after the swap — which needs the expensive
part (library load, construction, parameter parsing) to happen *before* the lock is taken. `Load` is
`Create` + `Init` for the ordinary start-up case.

## 4. The stage selection parameter

The proposed convention, **one key per stage, string-valued, naming the pluginlib class**:

| Stage | Key |
|---|---|
| gait sequencer | `gs.type` |
| MPC | `mpc.type` |
| swing leg controller | `slc.type` |
| WBC | `wbc.type` |
| model adaptation | `model_adaptation.type` |
| contact logic | `contact_logic.type` |

Rules: the key is **required** — absent, blank, non-string or unknown is a fatal bring-up error, and
no stage is substituted. If a default is ever wanted it belongs in the shipped YAML, where it is
visible, not inside the loader where it is not.

**M2.5 (#10) owns this vocabulary and may change it.** `StageLoader` therefore never hard-codes a
key: `Load` takes it as an argument. Renaming `gs.type` costs a change in the host and the YAML and
nothing in this layer. The key lives in the same flat parameter map as every other stage parameter
(the one key-space of [`plugin_lifecycle.md`](plugin_lifecycle.md) §3) rather than in a second
channel, so start-up configuration stays one vocabulary.

This supersedes the legacy `gait_sequencer` parameter (`"Simple"` / `"Adaptive"` / `"Bio"`,
`mit_controller_node.cpp:256`), which M2.4/M2.5 retire. M2.2 does not touch it.

## 5. The base-class-type strings, and escaping them in XML

`stage_plugin_bases::{kGaitSequencer, kMPC, kSwingLegController, kWBC, kModelAdaptation,
kContactLogic}` and `kStagePluginPackage` mirror
[`plugin_discovery.md`](plugin_discovery.md) §3 exactly. They are spelled once here so the host, the
tests and the XML share one definition; changing one still means changing the XML, this header and
`plugin_discovery.md` in the same pull request. `test_stage_loader.cpp` compares each constant
against a hard-coded literal so drift fails a test rather than a robot.

**M2.3 must escape these in the XML.** `<` is not legal in an XML attribute value, so a `<class>`
entry has to read

```xml
base_class_type="StagePlugin&lt;GaitSequencerInterface&gt;"
```

tinyxml2 unescapes before pluginlib compares against the C++-side string, so the two spellings match.
M2.1's description files carry the base strings only in comments, so this did not come up there;
`test/stage_loader_test_plugins.xml` is the first file to declare a real class and proves the escaped
form resolves.

The loader looks classes up in the ament resource of package `controllers`
(`controllers__pluginlib__plugin`), which pluginlib resolves across **all** installed packages. An
out-of-package plugin (the M4 goal) is found as soon as it calls
`pluginlib_export_plugin_description_file(controllers …)`; nothing here needs an allowlist.

## 6. Performance

**M2.2 changes no production code path.** No `.cpp` that ends up in `mitcontrollernode` is touched:
the additions are one exported header nothing in the product includes yet (M2.4 is its first
consumer), a test, a test-only plugin library, documentation, and two lines of build metadata. The
node's call graph, optimisation flags and behaviour are unchanged.

The design constraints that keep it that way once the host does use it:

- **Loading is a bring-up cost, never a loop cost.** XML parsing, `dlopen` and string resolution all
  happen inside `Load`, which runs from host initialisation context, off the control loops
  ([`plugin_lifecycle.md`](plugin_lifecycle.md) §1 rule 3). Reconfiguration uses `Create` +
  swap-under-lock (§3) so even that work happens off-loop.
- **No new per-cycle indirection.** The host keeps calling stages through the single virtual dispatch
  it already uses (`unique_ptr<Interface>`); `StageLoader` returns a pointer to the *same* interface
  and then steps out of the way. Nothing in this header is on a per-cycle path — there is no loader
  lookup, no string comparison and no map access per control step, and M2.4 must keep it that way.
- **Zero cost to non-users.** `stage_loader.hpp` is a template header including only
  `stage_plugin.hpp` and pluginlib, so a translation unit pays only for the bases it instantiates.

One finding for **M2.3** (#8), learned while building the test plugin library: this package links a
non-PIC static `libfmt.a` (through `common` → `quad_model_pino` → drake, and directly in
`mitcontrollernode`), and a non-PIC archive **cannot** go into a shared object — the link fails with
`relocation R_X86_64_PC32 … can not be used when making a shared object`. The stock plugin `.so`s
will hit this the moment they link the real algorithm code. Either the affected dependencies get
rebuilt with `-fPIC` or the plugin libraries must be arranged to avoid them. The test plugin library
sidesteps it by consuming `common`/`interfaces` as headers only; a real wrapper cannot.

## 7. What guards it

`test/test_stage_loader.cpp` (runs under `colcon test`), 15 cases, weighted towards the failure
paths because that is where the acceptance criterion lives:

| Area | Cases |
|---|---|
| success | resolve → create → `Init` → drive through the frozen interface; the required parameter's value and an unrelated optional parameter both arrive; an absent optional key defaults *inside the stage* |
| fail-fast | missing / non-string / empty / unknown `type:` — each asserting the message names the key or value, and that the unknown case lists the declared classes |
| taxonomy | a stage whose `Init` throws surfaces as `StageInitError` naming the stage; a missing required stage parameter does too; neither is catchable as the other |
| reconfiguration | `Create` does not run `Init`; `Create` rejects unknown classes; a replacement is `Init`ed off-loop and move-assigned into the host's `PluginPtr` member, proving M2.4's swap pattern compiles and works |
| schema | the six base strings and the package name match the literals in `plugin_discovery.md` §3; a production loader constructs for every stage base and declares nothing yet |

The plugins it loads are **real**: `test/stage_loader_test_plugins.cpp` is built into its own shared
library and registered with `PLUGINLIB_EXPORT_CLASS`, so the test exercises the whole chain — XML,
lookup, `dlopen`, default construction, `Init` — rather than a mock of it. Everything it observes
about `Init` is read back through the frozen `GaitSequencerInterface`, never through a cross-library
global, because "the host can drive a loaded stage through its interface" is part of what is being
tested.

`test/stage_loader_test_plugins.xml` is deliberately **not** registered with
`pluginlib_export_plugin_description_file`: it stays out of the `controllers__pluginlib__plugin`
resource, so production discovery keeps reporting zero declared classes and
`test_plugin_discovery.cpp` is unaffected. The test reaches it through `StageLoader`'s explicit
`plugin_xml_paths` argument. The file lives in the source tree because pluginlib determines a
description file's owning package by walking up to the nearest `package.xml`, and resolves the
library only relative to *that* package's install prefix — absolute paths in `path=` are not
honoured. The library is therefore installed into `controllers`' `lib/`, under `BUILD_TESTING` only.

`src/tools/pipeline_types_surface_check.cpp` covers the other axis: it includes `stage_loader.hpp`
through an export-only include path, so the header staying self-contained for an out-of-package
consumer is checked by the ordinary `colcon build`, along with the non-copyable/non-movable shape and
the error-type distinction.

```bash
colcon build --packages-select controllers --cmake-args -DROBOT_NAME=go2
colcon test  --packages-select controllers --ctest-args -R test_stage_loader
colcon test-result --verbose
```

## 8. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #6 | [M2.1] pluginlib dependency and plugin description XML | Provides the schema and exported surface this consumes — [`plugin_discovery.md`](plugin_discovery.md) |
| #7 | [M2.2] Stage plugin base + loader helper | **This document** |
| #8 | [M2.3] Wrap existing stages as stock plugins | Fills the `<class>` entries the loader resolves; must escape the base strings (§5) and solve the non-PIC link (§6) |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | First consumer: owns the loaders, declares them before the stage pointers (§3), keeps per-cycle dispatch unchanged (§6) |
| #10 | [M2.5] YAML schema for stage selection | Ratifies or renames the `<stage>.type` convention (§4) |
| #12 | [M3.1] Extract contact FSM | Uses `StageLoader<ContactLogicInterface>` unchanged |
| #13 | [M3.2] Runtime WBC / command-type profile | Collapses the WBC instantiation; `stage_plugin_bases::kWBC` changes with the XML |
| #17 | [M4.2] Example passthrough / logging plugin | Loaded by the same path, from outside this package (§5) |
