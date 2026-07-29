# Stage Overlays: swapping one stage at launch

**Status:** added in M4.4 (issue #19) ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`stage_loading.md`](stage_loading.md) — the loader that resolves the values
set here, and the key vocabulary · [`stock_plugins.md`](stock_plugins.md) — what each stock value
does · [`adding_a_stage.md`](adding_a_stage.md) — writing a stage of your own ·
[`plugin_discovery.md`](plugin_discovery.md) — why an out-of-package plugin is selectable at all

Every stage of the control pipeline is a `pluginlib` class chosen by one string. M2.5 (#10) ratified
where those strings live — a `<stage>.type` key in the robot's YAML — and M2.4 (#9) made the host
resolve them through `StageLoader` instead of a factory. What was still missing is the last, smallest
step: **changing one of them for one run** without editing a file that thirty other parameters share.

Until M4.4 the answer was to bypass the launch file entirely (`ros2 run ... --ros-args -p
slc.type:=…`, re-deriving the config paths by hand) or to copy the whole Go2 YAML to change one line.
This document describes the two arguments that replace both.

## 1. The two arguments

```bash
# One stage, one plugin id.
ros2 launch controllers mit_controller.launch.py sim:=go2 gs:=bio_gait

# A params file layered over the robot config.
ros2 launch controllers mit_controller.launch.py sim:=go2 stage_overlay:=go2_bio_gait.yaml
```

| Argument | Values | Use it when |
|---|---|---|
| `gs:=` `mpc:=` `slc:=` `wbc:=` `model_adaptation:=` `contact_logic:=` | any declared plugin id | the swap is *only* a choice of implementation |
| `stage_overlay:=` | a path, or the bare filename of a file in `config/overlays/` | the swap also needs the stage's **parameters**, or you want it reproducible in a script |

Both compose, and both work for `sim:=` and `real:=` on either robot — they set parameters, and know
nothing about which config those parameters are layered onto.

### Why both

A `<stage>:=` argument can only carry a type. That is the whole swap for `bio_gait`, which reads the
gait-sequencer keys the Go2 configs already set, but not for a stage with parameters of its own:
`example_passthrough_slc` also has a `log_period`, and a selection that cannot carry it is only half
an answer. The overlay covers that, and is the form to name in a demo script or a bug report, since
it is a file that can be read and diffed rather than a shell line to reproduce.

Neither is a superset of the other in practice, which is why M4.4 ships both rather than picking.

## 2. Precedence

Weakest to strongest:

```
robot config  <  common config  <  stage_overlay file  <  <stage>:= argument
```

This is not a rule the launch file implements — it *is* the order of the `parameters` list it builds.
`launch_ros` emits one `--params-file` / `-p` per entry in list order and `rcl` lets the later
assignment win, so the chain and the code are the same thing.

The consequence worth knowing:

```bash
ros2 launch controllers mit_controller.launch.py sim:=go2 \
    stage_overlay:=go2_example_passthrough_slc.yaml slc:=bezier_swing
```

selects the **stock** swing leg controller. The overlay's `slc.type` loses to the explicit argument;
everything else in the overlay — here `example_passthrough_slc.log_period` — still applies. Narrow
and explicit beating broad and implicit is the order that lets you keep an overlay and override one
line of it.

## 3. The shipped overlays

`ws/src/controllers/config/overlays/`, resolved by bare filename. Each file documents itself; both
are also the worked examples for the two halves of M4's exit criterion.

| Overlay | Swaps | Notes |
|---|---|---|
| [`go2_bio_gait.yaml`](../../ws/src/controllers/config/overlays/go2_bio_gait.yaml) | `gs.type` → `bio_gait` | In-package stock stage (M4.1, #16). A visible change of gait in sim. One key, so `gs:=bio_gait` is equivalent. |
| [`go2_example_passthrough_slc.yaml`](../../ws/src/controllers/config/overlays/go2_example_passthrough_slc.yaml) | `slc.type` → `example_passthrough_slc`, plus its `log_period` | Ships from **another package** (M4.2, #17) — build `example_stage_plugins` first. **The robot stands and does not walk**, by design. |

> The passthrough overlay holds every foot where it is. A walk command while it is loaded is a
> documented no-op, not a regression — see
> [`example_stage_plugins/README.md`](../../ws/src/examples/example_stage_plugins/README.md) §1. It
> is a template and a diagnostic probe, not locomotion, and it must not be used to move a real robot.

The second one is the interesting one: nothing in `controllers` knows that package exists, and no
allowlist was edited to make its class selectable. The `<stage>.type` value crosses the package
boundary because `pluginlib` resolves the `controllers__pluginlib__plugin` resource across every
installed package ([`plugin_discovery.md`](plugin_discovery.md) §3a). Selecting your own stage from
your own package works exactly the same way — [`adding_a_stage.md`](adding_a_stage.md) §5.

## 4. Writing your own overlay

```yaml
mit_controller_node:
  ros__parameters:
    slc:
      type: my_stage
    my_stage:
      my_key: 1.0
```

Three things to get right:

- **Address `mit_controller_node`.** An overlay written for another node name is layered on and
  silently does nothing. (`/**` also works, and is worth using if you run the host under a
  non-default name.)
- **Spell the selection key exactly** — `gs.type`, not `gs.typ`. A misspelled key is valid YAML that
  sets a parameter nothing reads, so the host keeps the stock stage and reports nothing. This is the
  one mistake in this area the loader's fail-fast cannot catch, because it never sees a wrong value;
  it is why `test_stage_overlays.cpp` checks the shipped files for it.
- **Set only what you are changing.** Everything omitted keeps coming from the robot config, which is
  the point.

Get the *value* wrong and you are back on solid ground — the loader refuses to start and lists what
is declared:

```
no stage plugin named 'bio_gat' is declared for base
'StagePlugin<GaitSequencerInterface>'; declared stage plugins:
'simple_gait', 'adaptive_gait', 'bio_gait'
```

`stage_overlay:=` fails the same way for a path that is not a file, naming both places it looked. Both
checks run **before** `safe_start()`, so a typo aborts immediately rather than after three seconds of
standing on a robot that is about to be told to walk.

## 5. Limits and interactions

- **Bring-up only, with one exception.** Stage selection is read when the pipeline is built. Changing
  a `<stage>.type` on a running node does nothing except for `gs.type`, which the host handles
  specially: it rebuilds the sequencer off-loop and swaps it in (M2.5). An overlay that selects a gait
  does not interfere with that — joystick `Y` and `ros2 param set` still work afterwards.
- **`wbc.type` is checked against `leg_control_mode`.** The two must agree, and the host refuses at
  bring-up if they do not, naming the pairing (M3.2, #13). If you swap the WBC you may need to set
  `leg_control_mode` too — which, being a parameter rather than a selection, is a case for an overlay
  file rather than a `wbc:=` argument.
- **`mpc:=` is not `mpc_solver:=`.** The first chooses the MPC *stage plugin*; the second chooses the
  QP backend inside the stock one, and keeps its existing behaviour untouched. They are matched on
  the full `name:=` prefix and do not collide.
- **No effect on the control loop.** Everything here is resolved before the node is spawned; M4.4
  changes no file under `src/` or `include/`. The host still calls every stage through the same single
  virtual dispatch ([`stage_loading.md`](stage_loading.md) §"No new per-cycle indirection").
- **ULab configs** work with both arguments, but pin fewer keys than the Go2 ones; see
  [`stage_selection.hpp`](../../ws/src/controllers/include/stage_selection.hpp) for what still comes
  from the legacy derivation there until #23 (M5.4).

## 6. What is tested

`ws/src/controllers/test/test_stage_overlays.cpp` pins the shipped files: that they install, that
each selects at least one stage with a non-empty string, that each stays smaller than the config it
overlays, and that each selected class is declared for its stage's base. The out-of-package selection
is asserted conditionally — whether `example_stage_plugins` is built is a property of the workspace,
not of this package, so the test asserts the implication and skips loudly when it is absent (the same
reasoning [`plugin_discovery.md`](plugin_discovery.md) records for its containment assertions).

What no test covers is the launch file itself: the argument parsing is exercised by hand, per the
checklist in the M4.4 pull request. #20 (M5.1) folds these arguments into a single modular launch
entry, which is the point at which they become worth a launch-level test rather than sooner.

## 7. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #19 | [M4.4] Launch overlay to swap one stage | **This document** |
| #10 | [M2.5] YAML schema for stage selection (Go2) | Ratified the keys these arguments set |
| #16 | [M4.1] Bio gait sequencer via plugin param | The stage `go2_bio_gait.yaml` selects |
| #17 | [M4.2] Example passthrough / logging SLC plugin | The stage `go2_example_passthrough_slc.yaml` selects |
| #18 | [M4.3] Contributor guide: add a control stage | §5 of the guide now points here |
| #20 | [M5.1] Unified Go2 modular controller launch entry | Will absorb these arguments into one entry point |
| #21 | [M5.2] Parameter reference for all stage keys | Will own the key-by-key reference §1 only summarises |
| #23 | [M5.4] Deprecate monolithic factory paths | Removes the legacy derivation §5 refers to |
