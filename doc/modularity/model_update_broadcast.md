# Model Update Broadcast: pushing an adapted model to every stage

**Status:** added in M3.4 (issue #15) ·
**Applies to:** `ws/src/controllers` ·
**Companion documents:** [`stage_contracts.md`](stage_contracts.md) §4.5 — the
`ModelAdaptationInterface` contract this consumes · [`pipeline_host.md`](pipeline_host.md) — the host
that owns the registry · [`plugin_lifecycle.md`](plugin_lifecycle.md) §5 — the stage replacement rule
that dictates how stages are captured

Every stage owns a **clone** of the quadruped model, handed to it at `Init`
([`plugin_lifecycle.md`](plugin_lifecycle.md) §3). One stage can change it:
`ModelAdaptationInterface::DoModelAdaptation` takes the host's model by non-const reference and
returns `true` when it adapted it — the only mutating stage method in the pipeline
([`stage_contracts.md`](stage_contracts.md) §4.5). On `true`, every *other* stage is holding a clone
that is now stale, and the host has to push the new model out.

This document covers **`ws/src/controllers/include/model_update_broadcast.hpp`**, the one place that
knows who gets that push.

## 1. What it replaces, and why

Until M3.4 the fan-out was written out by hand inside `MITController::ModelAdaptationCallback`:

```cpp
mpc_lock_.lock();
mpc_->UpdateModel(quad_model_);
gs_->UpdateModel(quad_model_);
mpc_lock_.unlock();
RCLCPP_INFO(this->get_logger(), "Updated used Model in MPC and GS");
wbc_lock_.lock();
wbc_->UpdateModel(quad_model_);
contact_logic_->UpdateModel(quad_model_);
wbc_lock_.unlock();
// ... and the same again for the SLC
```

Five stage calls, three lock/unlock pairs, three log lines — inside a callback whose subject is the
adaptation, not the distribution. It was the last hardcoded per-stage list in the host after M2.4
removed the construction ones, and it had the property [`stage_contracts.md`](stage_contracts.md) §5
flagged: **adding a stage meant editing it**. M3.1 proved the point by having to thread the contact
stage into the middle of it.

It is now one registration block at bring-up plus one call in the callback:

```cpp
model_update_broadcast_.Broadcast(quad_model_, this->get_logger());
```

## 2. The API

```cpp
class ModelUpdateBroadcast {
 public:
  template <class StagePtr>
  void Register(std::string name, std::mutex& lock, const StagePtr& slot);

  void Broadcast(const ModelInterface& model, const rclcpp::Logger& logger) const;

  std::size_t size() const;
};
```

Header-only, non-copyable, host-internal. It sits in `include/` next to
[`stage_selection.hpp`](../../ws/src/controllers/include/stage_selection.hpp) and **not** in
`include/mit_controller/`: it is not part of the exported plugin surface
([`plugin_discovery.md`](plugin_discovery.md) §2) and no plugin ever includes it.

## 3. How a stage opts in

Two steps, and the first one is usually already done:

1. **Implement `UpdateModel(const ModelInterface&)`.** All five stage interfaces
   (`GaitSequencerInterface`, `MPCInterface`, `SwingLegControllerInterface`, `WBCInterface`,
   `ContactLogicInterface`) already declare it with identical semantics — see the §4 method tables of
   [`stage_contracts.md`](stage_contracts.md). Refresh whatever the stage derived from the model:
   solver matrices, inertia terms, cached foot geometry. It is called outside the control loops, on
   a rare event, so it may be as expensive as it needs to be.
2. **Add one `Register` line** to the registration block in `MITController`'s constructor, right
   after the stages are loaded.

That is the whole contract. There is deliberately **no** `ModelUpdateReceiver` base class to derive
from: the five stage interfaces are frozen (M1.1) and already carry the method, so a common base
would edit five headers and change every shipped plugin's vtable to express something the interfaces
say already. Issue #15's "stages opt in via interface method (**existing** or extended)" is satisfied
by the existing one; what M3.4 centralizes is the host's *list*, not the stages' API.

The broadcast is duck-typed on `UpdateModel` — it names no stage interface and no stage type.

## 4. The registration table

| Order | Stage | Registered under | Why that lock |
|---|---|---|---|
| 1 | `MPC` | `mpc_lock_` | the MPC's own stage lock |
| 2 | `GS` | `mpc_lock_` | **gap G6** — see below |
| 3 | `WBC` | `wbc_lock_` | the WBC's own stage lock |
| 4 | `contact logic` | `wbc_lock_` | the contact stage runs inside the control loop under `wbc_lock_`, between the SLC's output and the WBC's input ([`stage_contracts.md`](stage_contracts.md) §4.6) |
| 5 | `SLC` | `slc_lock_` | the SLC's own stage lock |

The model adaptation stage itself is **not** registered: it is the producer, and `DoModelAdaptation`
already has the model in its hands.

**Registration order is broadcast order, and consecutive entries sharing a mutex are updated under
one acquisition of it.** So the table above is exactly the three critical sections the hand-written
version had, in the same order — that is how M3.4 is a no-behaviour-change refactor rather than a
re-synchronisation. Reordering the block would change lock behaviour, not just log wording;
`test_model_update_broadcast.cpp` pins that a lock used by two non-consecutive groups is genuinely
taken twice.

**G6 lives on line 2.** `gs_->UpdateModel` running under `mpc_lock_` rather than
`gait_sequencer_lock_` is gap G6 of [`stage_contracts.md`](stage_contracts.md) §7, open since M1.1
and deferred out of M2.4 and M3.1 on the grounds that the lock discipline should be decided in one
place rather than fixed piecemeal ([`pipeline_host.md`](pipeline_host.md) §8). M3.4 does not fix it
— it preserves it deliberately, and makes it a one-argument edit on one line. That is the place §8
was asking for.

## 5. Threading

`Broadcast` runs in the model-adaptation callback group, only on an actual model change. It takes
each group's mutex in turn and **never holds two at once**, so it cannot deadlock against a control
loop regardless of the order that loop takes them in. The log line for a group is emitted after its
mutex is released, and the joined stage names are built before it is taken, so the critical section
contains nothing but the `UpdateModel` calls themselves.

`quad_model_` — the host's model, the one being broadcast — is still mutated by `DoModelAdaptation`
without a lock of its own. That is the other half of G6 and is likewise unchanged here.

## 6. Stage replacement

`Register` captures the host's stage **slot** by reference, not the stage instance:

```cpp
model_update_broadcast_.Register("GS", mpc_lock_, gs_);   // gs_ is the member, read at broadcast time
```

This matters because reconfiguration is *replace, not re-`Init`*
([`plugin_lifecycle.md`](plugin_lifecycle.md) §5): a gait parameter change builds a fresh sequencer
off-loop and swaps it into `gs_` under `gait_sequencer_lock_`. A registry holding the old instance
would either need re-registering on every reload or would hand the model to a stage that is about to
be destroyed. Reading the slot at broadcast time makes the reload path invisible to the registry.

A stage member must therefore outlive the broadcast; `model_update_broadcast_` is declared after the
stage pointers and the mutexes in `mit_controller_node.hpp` so that it is destroyed before them, the
same ordering rule the loaders and stage pointers already follow
([`stage_loading.md`](stage_loading.md) §3).

## 7. Performance

The broadcast is **event-driven, not periodic**: it runs only when `DoModelAdaptation` reports a
change, in the model-adaptation callback group. None of the three control loops touch it, so nothing
at 500 Hz / 100 Hz changed.

Per event, the five direct virtual calls became five `std::function` invocations bound once at
bring-up — one extra indirection each, against `UpdateModel` bodies that re-seed solvers and copy
Pinocchio models. The synchronisation is identical by construction: same three acquisitions, same
mutexes, same order, taken sequentially. The only allocation is the joined log label, built outside
the lock, next to an `RCLCPP_INFO` that was already there.

## 8. Tests

`ws/src/controllers/test/test_model_update_broadcast.cpp` — the first pin this fan-out has had; its
behaviour was previously reachable only by running a robot until the adaptation changed the model.
It covers registration order and once-only delivery, that each update runs under its registered
mutex, the group/acquisition structure (including that only one mutex is held at a time), the
capture-by-slot rule of §6, and the empty registry. Lock state is probed from a helper thread,
because `std::mutex::try_lock` from the thread that already owns the mutex is undefined behaviour.

## 9. Related issues

| Issue | Title | Relationship |
|---|---|---|
| #24 | [Meta] Modular Go2 control | Parent |
| #1 | [M1.1] Audit and freeze stage interface APIs | Froze the `UpdateModel` this dispatches — §3 |
| #9 | [M2.4] Refactor `MITController` into thin `PipelineHost` | Removed the *construction* lists; left this one — [`pipeline_host.md`](pipeline_host.md) §8 |
| #12 | [M3.1] Extract contact FSM into `ContactLogic` plugin | Added the fifth entry by hand — the last time that was necessary |
| #15 | [M3.4] Model update broadcast helper | **This document** |
| #17 | [M4.2] Example passthrough / logging plugin | Consumes §3 — one `Register` line |
| #18 | [M4.3] Contributor guide: add a control stage | **Done.** [`adding_a_stage.md`](adding_a_stage.md) §6 links §3 as the model-update step a new stage must honour |
