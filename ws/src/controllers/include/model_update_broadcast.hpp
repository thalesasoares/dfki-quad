#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "common/model_interface.hpp"

/**
 * The model-update broadcast (issue #15, M3.4).
 *
 * `ModelAdaptationInterface::DoModelAdaptation` is the only mutating stage
 * method in the pipeline (`doc/modularity/stage_contracts.md` §4.5): it takes
 * the host's model by non-const reference and returns `true` when it changed
 * it. Every other stage holds its own model clone, so on `true` the host has to
 * push the new model out to all of them. Until this class that fan-out was five
 * hardcoded `stage_->UpdateModel(...)` calls interleaved with three
 * lock/unlock pairs and three log lines, inline in `ModelAdaptationCallback` —
 * a list that every new stage had to be added to by hand, in the middle of a
 * callback that is about something else.
 *
 * Here it is data instead: the host registers each stage once at bring-up with
 * the mutex that stage's `UpdateModel` runs under, and the callback makes one
 * `Broadcast` call. Adding a stage to the pipeline means adding one `Register`
 * line and nothing else, which is the acceptance criterion of issue #15 and
 * what `doc/modularity/model_update_broadcast.md` tells a stage author.
 *
 * **Stages opt in through the interface method they already have.** All five
 * stage interfaces declare `void UpdateModel(const ModelInterface&)` with the
 * same semantics; this class does not add a base class, and the frozen M1.1
 * interfaces are untouched by it.
 *
 * Host-internal, deliberately: it lives next to `stage_selection.hpp` rather
 * than in `include/mit_controller/`, because a stage author implements
 * `UpdateModel` and never sees this file. It is not part of the exported plugin
 * surface and is not installed.
 */
class ModelUpdateBroadcast {
 public:
  ModelUpdateBroadcast() = default;

  // Entries capture references to host members — the stage slots and the
  // mutexes. Copying one would hand out entries still bound to the original
  // host's members, which is never what a copy is for. Non-copyable and
  // non-movable, for the same reason `StageLoader` is.
  ModelUpdateBroadcast(const ModelUpdateBroadcast &) = delete;
  ModelUpdateBroadcast &operator=(const ModelUpdateBroadcast &) = delete;

  /**
   * Registers a stage to receive model updates.
   *
   * `slot` is the host's stage *member* and is captured **by reference**, not
   * dereferenced here: the entry reads it at broadcast time. That is what makes
   * the registry survive a runtime stage swap — the gait sequencer is replaced
   * in place when a gait parameter changes (`plugin_lifecycle.md` §5, the
   * reload path in `MITController`'s parameter callback) and the broadcast must
   * reach the *new* instance without being re-registered. It also means the
   * registry can never hold a stage pointer that outlived its loader.
   *
   * Registration order is broadcast order, and consecutive entries sharing a
   * mutex are broadcast under a single acquisition of it (see `Broadcast`), so
   * the order the host registers in is the lock discipline it gets.
   *
   * Call at bring-up only, after the stages are loaded and before the loops
   * start. Never from a control loop: this allocates.
   *
   * @param name how the stage is named in the broadcast's log line
   * @param lock the mutex this stage's `UpdateModel` runs under; must outlive
   *             this object, as must `slot`
   * @param slot the host's stage member (any pointer-like whose `operator->`
   *             yields something with `UpdateModel(const ModelInterface&)`)
   */
  template <class StagePtr>
  void Register(std::string name, std::mutex &lock, const StagePtr &slot) {
    entries_.push_back({std::move(name), &lock, [&slot](const ModelInterface &model) {
                          slot->UpdateModel(model);
                        }});
  }

  /**
   * Pushes `model` to every registered stage, in registration order.
   *
   * Consecutive entries registered under the same mutex are updated under one
   * acquisition of it and reported in one "Updated used Model in ..." line —
   * the grouping is the host's lock discipline, expressed as registration
   * order rather than as hand-written lock/unlock pairs.
   *
   * Runs only when the model actually changed, from the model-adaptation
   * callback group. It takes each group's mutex in turn and never holds two at
   * once, so it cannot deadlock against the control loops.
   */
  void Broadcast(const ModelInterface &model, const rclcpp::Logger &logger) const {
    for (std::size_t group_begin = 0; group_begin < entries_.size();) {
      std::mutex *const lock = entries_[group_begin].lock;
      std::size_t group_end = group_begin + 1;
      while (group_end < entries_.size() && entries_[group_end].lock == lock) {
        ++group_end;
      }

      // Built before the lock is taken, so the critical section is exactly the
      // UpdateModel calls it was before this class existed — no allocation
      // inside it.
      std::string updated = entries_[group_begin].name;
      for (std::size_t i = group_begin + 1; i < group_end; ++i) {
        updated += " and " + entries_[i].name;
      }

      {
        std::lock_guard<std::mutex> guard(*lock);
        for (std::size_t i = group_begin; i < group_end; ++i) {
          entries_[i].update(model);
        }
      }
      RCLCPP_INFO(logger, "Updated used Model in %s", updated.c_str());

      group_begin = group_end;
    }
  }

  /** How many stages are registered. For tests and diagnostics. */
  std::size_t size() const { return entries_.size(); }

 private:
  struct Entry {
    std::string name;
    std::mutex *lock;
    std::function<void(const ModelInterface &)> update;
  };

  std::vector<Entry> entries_;
};
