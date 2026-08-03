# Build prompt: Kennel Console UI (mock-data phase)

**Status:** Paste-ready prompt to scaffold the UI now, without a running ROS stack ·
**Companion document:** [`kennel_console_ui_spec.md`](kennel_console_ui_spec.md) — full spec with
ROS topic/message contracts, for the later integration phase

---

Build a single-page web app called **Kennel Console** — an engineering control room for running
quadruped (Unitree Go2) simulation experiments. Stack: React + TypeScript + Vite, Tailwind.
Dark-theme-first, information-dense, Foxglove/Grafana aesthetic. All live data comes from a
`DataSource` interface — implement only a `MockDataSource` now (simulated 10 Hz streams, scripted
fall event after ~30 s); a ROS bridge implementation will be swapped in later.

**Shell:** left nav rail with three views (Compose, Dashboard, Runs) + persistent bottom status
bar: connection state, sim real-time factor, heartbeat age, run ID.

## View 1 — Compose

Experiment builder, two columns.

- *Map picker:* card grid — Flat plane, Obstacle terrain (stairs/ramps/rough), Brick — plus sim
  options (initial height, sensor noise toggles, real-time rate, ground-truth-state toggle).
- *Pipeline composer* (centerpiece): horizontal block diagram of 6 stages — Gait Sequencer
  (Simple/Adaptive/Bio) → MPC (solver: HPIPM/OSQP/qpOASES + mode) → WBC (ArcOPT); side branch
  Swing Leg Ctrl → Contact Logic → WBC; Model Adaptation (Off/KF/LeastSquares) broadcasting to
  all. Each block: dropdown of implementations; clicking opens a parameter drawer (form fields
  with defaults, "modified" badges, raw-YAML tab).
- *Generate run:* renders two YAML files + a copy-paste terminal command block (3 commands:
  simulator, controller, state estimation) and saves a run manifest. Preset save/load.

## View 2 — Dashboard

Resizable panel grid, live from `DataSource`:

- 3D scene placeholder panel (will embed Meshcat iframe).
- *Pipeline health strip:* same 6-block diagram, blocks tinted green/amber/red from solve-time
  vs. deadline (MPC 10 ms, WBC 2 ms), solver failures, topic freshness; click → detail plots.
- *Gait/contact timeline:* 4-row (FL/FR/RL/RR) scrolling strip chart, planned contact phases as
  bands, actual contacts overlaid, early/late touchdown mismatches highlighted in amber.
- *Health counters:* sparklines of failure/overtime counter rates; increases flash the matching
  pipeline block.
- *State plots:* body height/attitude, commanded vs. actual velocity.
- *Event feed:* auto-scrolling plain-language diagnosis log ("MPC exceeded 10 ms deadline
  (12.4 ms, 23 iters)", "Early contact FL +40 ms", "FALL: belly contact, pitch −38°"); a fall
  raises a red banner and pins the last 5 s of events.
- *Interventions toolbar:* virtual joystick + fields for velocity target, disturbance injector
  (force vector/magnitude/duration), sim reset, single-step buttons.

## View 3 — Runs

Table of past run manifests (map, stages, duration, verdict badge: completed/fell/solver-failed,
headline counters); select two → side-by-side config diff.

## Rules

- Every panel has a meaningful empty state naming the missing launch command.
- Every plot labels units and source topic.
- Config load must round-trip to identical composer state.
- No process orchestration anywhere in the UI (generate commands, never run them).
- Seed the mock with a demo run: 30 s of stable trot on terrain, degrading MPC solve times, then
  a fall — so every panel demonstrates its purpose immediately.
