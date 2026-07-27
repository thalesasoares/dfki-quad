// Model-update broadcast test (issue #15, M3.4).
//
// The fan-out this class replaces was five hardcoded UpdateModel calls wrapped
// in three lock/unlock pairs inside ModelAdaptationCallback. Its behaviour was
// only observable by running a robot until the model adaptation actually
// changed the model, so nothing pinned it. This test is that pin: it asserts
// the four properties the host relies on, and that a future stage registration
// must not break.
//
//   1. every registered stage is updated, once, in registration order
//   2. each update runs with the mutex it was registered under held
//   3. entries sharing a mutex share one acquisition of it, and only that one
//      mutex is held at a time — this is the lock *grouping* that reproduces
//      the host's three critical sections
//   4. the registry follows a stage swapped in place at runtime — the gait
//      sequencer reload path depends on it
//
// Lock state is probed from a helper thread throughout: std::mutex::try_lock
// from the thread that already owns the mutex is undefined behaviour, so
// "is it held?" can only be asked from somewhere else. LockedElsewhere() does
// exactly that and is the only reason this file needs <thread>.
//
// The model double is potato_sim's BrickModel, as in the other stage tests. The
// broadcast only passes the reference through; what the fakes check is that
// they all received the *same* one.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "model_update_broadcast.hpp"
#include "potato_sim/potato_model.hpp"

namespace {

// True if `mutex` is currently locked by someone. Asked from a fresh thread,
// because try_lock on a mutex the calling thread already owns is UB.
bool LockedElsewhere(std::mutex &mutex) {
  bool acquired = false;
  std::thread probe([&mutex, &acquired] {
    acquired = mutex.try_lock();
    if (acquired) {
      mutex.unlock();
    }
  });
  probe.join();
  return !acquired;
}

// A stage as far as the broadcast is concerned: something with UpdateModel.
// Deliberately not derived from any stage interface — the broadcast is
// duck-typed on the method all five stage interfaces already declare, and
// pinning that here is the point of issue #15's "stages opt in via an existing
// interface method".
class FakeStage {
 public:
  FakeStage(std::string name, std::vector<std::string> *call_log) : name_(std::move(name)), call_log_(call_log) {}

  void UpdateModel(const ModelInterface &model) {
    call_log_->push_back(name_);
    ++calls_;
    last_model_ = &model;
    for (std::mutex *const mutex : expect_held_) {
      held_.push_back(LockedElsewhere(*mutex));
    }
    for (std::mutex *const mutex : expect_free_) {
      free_.push_back(!LockedElsewhere(*mutex));
    }
  }

  /** Mutexes this stage asserts are held while it runs. */
  void ExpectHeldWhileRunning(std::mutex *mutex) { expect_held_.push_back(mutex); }

  /** Mutexes this stage asserts are *not* held while it runs. */
  void ExpectFreeWhileRunning(std::mutex *mutex) { expect_free_.push_back(mutex); }

  int calls() const { return calls_; }
  const ModelInterface *last_model() const { return last_model_; }

  /** Whether every ExpectHeldWhileRunning mutex was held on every call. */
  bool sawExpectedHeld() const {
    return !held_.empty() && std::all_of(held_.begin(), held_.end(), [](bool held) { return held; });
  }

  /** Whether every ExpectFreeWhileRunning mutex was free on every call. */
  bool sawExpectedFree() const {
    return !free_.empty() && std::all_of(free_.begin(), free_.end(), [](bool was_free) { return was_free; });
  }

 private:
  std::string name_;
  std::vector<std::string> *call_log_;
  std::vector<std::mutex *> expect_held_;
  std::vector<std::mutex *> expect_free_;
  std::vector<bool> held_;
  std::vector<bool> free_;
  int calls_ = 0;
  const ModelInterface *last_model_ = nullptr;
};

std::unique_ptr<BrickModel> MakeModel() { return std::make_unique<BrickModel>(Eigen::Matrix3d::Identity(), 1.0); }

rclcpp::Logger TestLogger() { return rclcpp::get_logger("test_model_update_broadcast"); }

// --- 1. order ---------------------------------------------------------------

TEST(ModelUpdateBroadcast, UpdatesEveryStageOnceInRegistrationOrder) {
  std::vector<std::string> call_log;
  std::mutex lock_a;
  std::mutex lock_b;
  auto first = std::make_unique<FakeStage>("first", &call_log);
  auto second = std::make_unique<FakeStage>("second", &call_log);
  auto third = std::make_unique<FakeStage>("third", &call_log);
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("first", lock_a, first);
  broadcast.Register("second", lock_a, second);
  broadcast.Register("third", lock_b, third);
  ASSERT_EQ(broadcast.size(), 3u);

  broadcast.Broadcast(*model, TestLogger());

  EXPECT_EQ(call_log, (std::vector<std::string>{"first", "second", "third"}));
  EXPECT_EQ(first->calls(), 1);
  EXPECT_EQ(second->calls(), 1);
  EXPECT_EQ(third->calls(), 1);
  // All three saw the very model the host handed in, not a copy of it.
  EXPECT_EQ(first->last_model(), model.get());
  EXPECT_EQ(second->last_model(), model.get());
  EXPECT_EQ(third->last_model(), model.get());
}

TEST(ModelUpdateBroadcast, RepeatedBroadcastsUpdateEveryStageAgain) {
  std::vector<std::string> call_log;
  std::mutex lock;
  auto stage = std::make_unique<FakeStage>("stage", &call_log);
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("stage", lock, stage);

  broadcast.Broadcast(*model, TestLogger());
  broadcast.Broadcast(*model, TestLogger());

  EXPECT_EQ(stage->calls(), 2);
}

// --- 2. the update runs under the registered lock ---------------------------

TEST(ModelUpdateBroadcast, UpdatesRunUnderTheRegisteredLock) {
  std::vector<std::string> call_log;
  std::mutex lock;
  auto stage = std::make_unique<FakeStage>("stage", &call_log);
  stage->ExpectHeldWhileRunning(&lock);
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("stage", lock, stage);
  broadcast.Broadcast(*model, TestLogger());

  EXPECT_TRUE(stage->sawExpectedHeld());
  // And the broadcast gave it back: the control loops must be able to take it
  // again on the next cycle.
  EXPECT_FALSE(LockedElsewhere(lock));
}

// --- 3. lock grouping -------------------------------------------------------

TEST(ModelUpdateBroadcast, ConsecutiveStagesShareOneAcquisitionOfTheirLock) {
  std::vector<std::string> call_log;
  std::mutex lock_a;
  std::mutex lock_b;
  auto first_a = std::make_unique<FakeStage>("first_a", &call_log);
  auto second_a = std::make_unique<FakeStage>("second_a", &call_log);
  auto only_b = std::make_unique<FakeStage>("only_b", &call_log);
  first_a->ExpectHeldWhileRunning(&lock_a);
  // The second entry of the group still sees lock_a held: it was never released
  // between the two, i.e. one acquisition spans both. While it runs, lock_b is
  // free — the broadcast holds one group's mutex at a time and never two, which
  // is why it cannot deadlock against a control loop that takes them in some
  // other order.
  second_a->ExpectHeldWhileRunning(&lock_a);
  second_a->ExpectFreeWhileRunning(&lock_b);
  // Symmetrically, lock_a is released again by the time the lock_b group runs.
  only_b->ExpectHeldWhileRunning(&lock_b);
  only_b->ExpectFreeWhileRunning(&lock_a);
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("first_a", lock_a, first_a);
  broadcast.Register("second_a", lock_a, second_a);
  broadcast.Register("only_b", lock_b, only_b);

  broadcast.Broadcast(*model, TestLogger());

  EXPECT_TRUE(first_a->sawExpectedHeld());
  EXPECT_TRUE(second_a->sawExpectedHeld());
  EXPECT_TRUE(second_a->sawExpectedFree());
  EXPECT_TRUE(only_b->sawExpectedHeld());
  EXPECT_TRUE(only_b->sawExpectedFree());
}

TEST(ModelUpdateBroadcast, TheSameLockUsedByNonConsecutiveGroupsIsTakenTwice) {
  // Registration order *is* the grouping: a-b-a is three critical sections, not
  // two. Pinned so that reordering the host's registration block is understood
  // to change lock behaviour, not just log wording.
  std::vector<std::string> call_log;
  std::mutex lock_a;
  std::mutex lock_b;
  auto first = std::make_unique<FakeStage>("first", &call_log);
  auto middle = std::make_unique<FakeStage>("middle", &call_log);
  auto last = std::make_unique<FakeStage>("last", &call_log);
  first->ExpectHeldWhileRunning(&lock_a);
  middle->ExpectHeldWhileRunning(&lock_b);
  middle->ExpectFreeWhileRunning(&lock_a);  // released before the b group
  last->ExpectHeldWhileRunning(&lock_a);    // and taken again for the third entry
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("first", lock_a, first);
  broadcast.Register("middle", lock_b, middle);
  broadcast.Register("last", lock_a, last);

  broadcast.Broadcast(*model, TestLogger());

  EXPECT_EQ(call_log, (std::vector<std::string>{"first", "middle", "last"}));
  EXPECT_TRUE(first->sawExpectedHeld());
  EXPECT_TRUE(middle->sawExpectedHeld());
  EXPECT_TRUE(middle->sawExpectedFree());
  EXPECT_TRUE(last->sawExpectedHeld());
}

// --- 4. runtime stage swap --------------------------------------------------

TEST(ModelUpdateBroadcast, FollowsAStageSwappedInPlaceAfterRegistration) {
  // The gait sequencer is replaced in place when a gait parameter changes
  // (plugin_lifecycle.md §5). The registry captures the host's *slot*, not the
  // instance, so a broadcast reaches whatever is in the slot at the time — no
  // re-registration on reload, and never a pointer to a destroyed stage.
  std::vector<std::string> call_log;
  std::mutex lock;
  std::unique_ptr<FakeStage> slot = std::make_unique<FakeStage>("original", &call_log);
  FakeStage *const original = slot.get();
  const auto model = MakeModel();

  ModelUpdateBroadcast broadcast;
  broadcast.Register("stage", lock, slot);
  broadcast.Broadcast(*model, TestLogger());
  ASSERT_EQ(original->calls(), 1);

  auto fresh = std::make_unique<FakeStage>("fresh", &call_log);
  FakeStage *const replacement = fresh.get();
  slot.swap(fresh);
  fresh.reset();  // the previous stage dies here, exactly as on the host's reload path

  broadcast.Broadcast(*model, TestLogger());

  EXPECT_EQ(replacement->calls(), 1);
  EXPECT_EQ(call_log, (std::vector<std::string>{"original", "fresh"}));
}

// --- edge case --------------------------------------------------------------

TEST(ModelUpdateBroadcast, BroadcastingWithNoRegisteredStagesIsANoOp) {
  const auto model = MakeModel();
  const ModelUpdateBroadcast broadcast;

  EXPECT_EQ(broadcast.size(), 0u);
  broadcast.Broadcast(*model, TestLogger());  // must not crash, and logs no stage line
}

}  // namespace
