# Prompt: "Kennel Console" — Web UI for the dfki-quad Go2 simulation lab

**Status:** Full UI specification (design + integration reference)
**Companion documents:** [`PRFAQ.md`](../PRFAQ.md) — product vision (Kennel);
[`modularity/stage_contracts.md`](../modularity/stage_contracts.md) — pipeline stage contracts;
[`kennel_console_ui_build.md`](kennel_console_ui_build.md) — paste-ready build prompt for the
mock-data UI phase

---

You are building **Kennel Console**, a browser-based front-end for the dfki-quad ROS 2 quadruped
stack (Unitree Go2, Drake simulation). It is a **researcher's control room**: the user composes a
simulation experiment (map + controller pipeline + parameters), the UI produces the exact config
files and launch commands, and once the stack is running the UI becomes a **live diagnosis
dashboard** that tells the researcher *what is happening and why* in real time.

**Hard constraint — configure + monitor only:** the UI never spawns or kills ROS processes. It
(a) generates/edits YAML configs and launch command lines for the user to run in a terminal, and
(b) connects to the running stack over a ROS–web bridge (rosbridge or Foxglove WebSocket) for live
data and service calls. Sim-level service calls (`/reset_sim`, `/step_sim`, disturbance injection)
are allowed — they interact with an already-running sim, not with processes.

## Users

1. **Controls researcher** (primary): swaps pipeline stages/solvers, tunes parameters, injects
   disturbances, needs solver-level diagnostics and reproducible configs.
2. **New lab member / student** (secondary): follows the same flow with defaults; must reach
   "watch the Go2 walk and understand the dashboard" without reading source code.

## Layout

Single-page app, three top-level views in a left nav rail, persistent status bar at the bottom
(bridge connection state, sim real-time factor, controller heartbeat age, current run ID).

### View 1 — Compose (experiment builder)

Two-column layout: left = configuration tree, right = live preview pane.

**a) Map picker.** Card grid of available worlds, each with a thumbnail and physics summary
(friction, hydroelastic properties): *Flat plane* (`plane.urdf`), *Obstacle terrain*
(`terrain.urdf` — stairs, ramps, rough mesh), *Brick* (`brick.urdf`). Selecting a card sets
`world_urdf` in the generated `simulator_params_go2.yaml`. Also expose sim options: initial robot
height/pose, IMU/joint noise toggles, `simulator_realtime_rate`, manual-stepping mode,
`publish_quad_state` (bypass state estimator — surface this clearly as "ground-truth state vs.
state estimation").

**b) Pipeline composer.** The centerpiece. Render the control pipeline as a horizontal block
diagram matching the real architecture (see
[`modularity/stage_contracts.md`](../modularity/stage_contracts.md)):

```
Target ─▶ [Gait Sequencer 100Hz] ─▶ [MPC 100Hz] ─▶ [WBC 500Hz] ─▶ /leg_cmd
                 └─▶ [Swing Leg Ctrl 500Hz] ─▶ [Contact Logic 500Hz] ─▶ WBC
          [Model Adaptation 100Hz] ···▶ (broadcasts model to all)
```

Each stage block is a dropdown of registered implementations:

- **Gait Sequencer**: Simple / Adaptive / Bio-inspired
- **MPC**: acados QP, with solver sub-select: HPIPM (partial/full condensing,
  SPEED_ABS/SPEED/BALANCE/ROBUST modes), OSQP, qpOASES + condensed-size field — these map to
  existing launch args `mpc_solver:=`, `mpc_hpipm_mode:=`, `mpc_condensed_size:=`
- **SLC**: SwingLegController (single stock option today)
- **WBC**: WBCArcOPT (Go2) — show QP solver choice (`wbc.solver`, e.g. EiquadprogSolver) and
  `leg_control_mode` (WBC vs. inverse-kinematics)
- **Model Adaptation**: Off / Kalman Filter / Least Squares
- **Contact Logic**: Default (inline today; behind interface after M3.1)

Clicking a block opens its **parameter panel**: form fields generated from the stage's YAML schema
(MPC weights, μ, f_min/f_max, WBC gains, swing height, gait params) with the stock Go2 defaults
pre-filled and a "modified" badge on anything changed. Include raw-YAML editing mode for
researchers.

**c) Export.** A "Generate run" action produces: (1) the composed `mit_controller_sim_go2.yaml` +
`simulator_params_go2.yaml` written to a run directory, (2) a copy-paste command block — one
terminal tab each for simulation, controller, and (if enabled) state estimation launch, with all
launch args (`sim:=go2`, `mpc_solver:=…`, `safe_start:=…`) baked in, (3) a run manifest (JSON)
capturing the full config + git hash for reproducibility. Named presets: save/load/duplicate
experiment configurations; ship with "Stock Go2 walk", "Stairs + Adaptive gait",
"Solver benchmark" presets.

### View 2 — Live dashboard (diagnosis in time)

Activates when the bridge connects. Grid of resizable panels:

- **3D scene**: embedded Meshcat viewer (iframe to the Drake Meshcat URL) — do not rebuild 3D
  rendering.
- **Pipeline health strip**: the same block diagram from Compose, now live — each stage tinted
  green/amber/red from real signals: MPC block from `/solve_time` (`MPCDiagnostics`: solve time
  vs. 10 ms deadline, `num_iter`, return status, QP residuals), WBC block from `/wbc_solve_time`
  (`WBCReturn.success`, timings vs. 2 ms deadline), Gait Sequencer from `/gait_state` freshness,
  Model Adaptation from `num_model_updates`. Clicking a block opens its detail plots.
- **Gait/contact timeline**: horizontal 4-row strip chart (FL/FR/RL/RR) showing planned contact
  phases from `/gait_state` (period, duty factor, phase offsets) overlaid with actual contact from
  `/quad_state.foot_contact` and measured forces from `/contact_state` — mismatches (early/late
  touchdown) highlighted. This is the single most informative legged-locomotion diagnostic.
- **Health counters**: from `/controller_heartbeat` (`ControllerInfo`): `num_early_contacts`,
  `num_mpc_solver_overtime`, `num_wbc_overtime`, `num_mpc_solver_fail`, `num_wbc_solver_fail` —
  shown as rate-of-change sparklines, not just totals; any increase flashes the corresponding
  pipeline block.
- **State plots**: body height/attitude, commanded vs. actual velocity (`/quad_control_target`
  vs. `/quad_state` twist), joint torques.
- **Event feed (the "diagnosis")**: an auto-scrolling annotated log that converts raw signals into
  plain-language, timestamped events: *"14:02:31.2 — MPC exceeded 10 ms deadline (12.4 ms, HPIPM,
  23 iters)"*, *"14:02:33.0 — Early contact FL (+40 ms before schedule)"*, *"14:02:35.1 — FALL:
  belly_contact=true, pitch −38°"*. Fall detection = `belly_contact` OR attitude/height
  thresholds; a fall raises a prominent banner with the last 5 s of events pinned for post-mortem
  reading.
- **Interventions toolbar**: velocity/gait target commands (publish `/quad_control_target` —
  joystick widget + fields), disturbance injection (call the `DisturbSim` service: push force
  vector, magnitude, duration), sim reset (`/reset_sim`), and single-step controls when
  `manually_step_sim` is on.

### View 3 — Runs

Table of past run manifests (config, map, stage choices, duration, verdict: completed / fell /
solver-failed, headline counters). Selecting two runs shows a config diff side by side. (Bag
replay/scrubbing is out of scope for v1; the run manifest + counters summary is the v1 record.)

## Developer notes / contracts

- **Bridge**: subscribe via rosbridge/Foxglove WS. Throttle aggressively — `/quad_state` and WBC
  topics are 500 Hz; downsample to ≤ 60 Hz for plots, keep full-rate only for windowed statistics
  computed client- or bridge-side. Message definitions live in `ws/src/interfaces/msg/`
  (`QuadState`, `GaitState`, `ContactState`, `MPCDiagnostics`, `WBCReturn`, `ControllerInfo`,
  `GaitSequence`).
- **Heartbeat topic is spelled `controller_heartbeat`** in the node; a legacy script uses
  `controller_hartbeat` — subscribe to the code's spelling and don't trust script spellings.
- **Stage `type:` selection keys are not yet in the YAMLs** (issue #10 / milestone M2.5); until
  they land, the composer maps stage choices to today's mechanisms (launch args for MPC solver,
  existing YAML keys) and must be built so the mapping layer is swappable for the `type:`-key
  schema without UI changes.
- Degrade gracefully: every panel must render a meaningful empty state when its topic is silent
  (e.g. controller not launched yet) and tell the user *which launch command is missing*.
- Config generation must be **round-trip safe**: loading a generated YAML back into the composer
  reproduces the exact same UI state.
- Visual identity: information-dense, dark-theme-first engineering console (think
  Foxglove/Grafana, not consumer app); every plotted quantity labelled with units and the topic it
  comes from.

## Out of scope (v1)

Process orchestration (starting/stopping ROS nodes), real-robot connection, bag replay/scrubbing,
multi-robot, editing URDFs/meshes, user accounts.

## Known upstream gaps this UI presupposes

- Issue #10 (M2.5): explicit stage `type:` keys in the Go2 controller YAMLs.
- A launch argument for `world_urdf` — today terrain choice is a YAML edit only; a small
  `world:=` launch arg would simplify the console's mapping layer considerably.
