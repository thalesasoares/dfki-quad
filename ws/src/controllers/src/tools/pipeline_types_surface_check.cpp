// Compile-smoke check for the exported shared pipeline type surface (issue #3,
// M1.3).
//
// This translation unit is built by the `pipeline_types_surface_check` OBJECT
// library, whose include path is deliberately restricted to `include/` plus the
// `common` and Eigen dependencies — it does *not* get `include/mit_controller`
// or `src/` the way the rest of the package does. It therefore sees the headers
// exactly as an out-of-package stage plugin sees them after `find_package(
// controllers)`, and fails the build if a surface header stops being
// self-contained or starts reaching into node internals.
//
// Contract tests proper are issue #5 (M1.5); this file is only the
// self-containment and layout guard. Keep the header list in sync with the
// `install(FILES ...)` block in CMakeLists.txt and with
// doc/modularity/pipeline_types.md.

// Each header is included first in its own block so that none of them can rely
// on a previous include for its own dependencies.
#include "mit_controller/target.hpp"  // IWYU pragma: keep

#include "mit_controller/gait_sequence.hpp"  // IWYU pragma: keep

#include "mit_controller/wrench_sequence.hpp"  // IWYU pragma: keep

#include "mit_controller/feet_targets.hpp"  // IWYU pragma: keep

#include "mit_controller/mpc_prediction.hpp"  // IWYU pragma: keep

#include "mit_controller/pipeline_constants.hpp"  // IWYU pragma: keep

// Stage interface + pluginlib surface (issue #6, M2.1). Each is included in its
// own block, seen through the same restricted include path an out-of-package
// plugin gets, so this fails the build if any of them reaches into node internals
// (e.g. the potato_sim/ headers or host-only mit_controller_params.hpp).
#include "mit_controller/gait_sequencer_interface.hpp"  // IWYU pragma: keep

#include "mit_controller/mpc_interface.hpp"  // IWYU pragma: keep

#include "mit_controller/swing_leg_controller_interface.hpp"  // IWYU pragma: keep

#include "mit_controller/wbc_interface.hpp"  // IWYU pragma: keep

#include "mit_controller/contact_logic_interface.hpp"  // IWYU pragma: keep

#include "mit_controller/stage_plugin.hpp"  // IWYU pragma: keep

#include "mit_controller/stage_loader.hpp"  // IWYU pragma: keep

#include "model_adaptation/model_adaptation_interface.hpp"  // IWYU pragma: keep

#include <type_traits>

namespace {

// The stages exchange these by value across the pipeline; copy assignment and
// default construction are part of the contract.
static_assert(std::is_default_constructible_v<Target>);
static_assert(std::is_default_constructible_v<GaitSequence>);
static_assert(std::is_default_constructible_v<WrenchSequence>);
static_assert(std::is_default_constructible_v<FeetTargets>);
static_assert(std::is_default_constructible_v<MPCPrediction>);

static_assert(std::is_copy_assignable_v<Target>);
static_assert(std::is_copy_assignable_v<GaitSequence>);
static_assert(std::is_copy_assignable_v<WrenchSequence>);
static_assert(std::is_copy_assignable_v<FeetTargets>);
static_assert(std::is_copy_assignable_v<MPCPrediction>);

// Sizes are defined in terms of pipeline_constants.hpp, not hard-coded. A change
// to either side that is not mirrored in the other breaks the build here.
static_assert(std::tuple_size_v<decltype(GaitSequence::contact_sequence)> == GAIT_SEQUENCE_SIZE);
static_assert(std::tuple_size_v<decltype(GaitSequence::contact_sequence)::value_type> == N_LEGS);
static_assert(std::tuple_size_v<decltype(GaitSequence::foot_position_sequence)> == GAIT_SEQUENCE_SIZE);
static_assert(std::tuple_size_v<decltype(GaitSequence::gait_swing_time)> == N_LEGS);

static_assert(std::tuple_size_v<decltype(WrenchSequence::forces)> == MPC_PREDICTION_HORIZON);
static_assert(std::tuple_size_v<decltype(WrenchSequence::forces)::value_type> == N_LEGS);

static_assert(std::tuple_size_v<decltype(FeetTargets::positions)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(FeetTargets::velocities)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(FeetTargets::accelerations)> == N_LEGS);

// The prediction spans the horizon plus the current knot; the WBC tracks index 1.
static_assert(std::tuple_size_v<decltype(MPCPrediction::position)> == MPC_PREDICTION_HORIZON + 1);
static_assert(std::tuple_size_v<decltype(MPCPrediction::orientation)> == MPC_PREDICTION_HORIZON + 1);
static_assert(std::tuple_size_v<decltype(MPCPrediction::raw_data)> == MPC_PREDICTION_HORIZON + 1);

// The gait sequence carries its own mode; the MPC reads it instead of the host
// pushing a weight switch (issue #2).
static_assert(static_cast<int>(GaitSequence::KEEP) == 0);
static_assert(static_cast<int>(GaitSequence::MOVE) == 1);

// The stage interfaces and their pluginlib bases are abstract: a plugin must
// override the frozen interface, and StagePlugin adds the Init lifecycle phase on
// top without becoming instantiable itself (plugin_lifecycle.md §7). The contract
// test (test/test_stage_contracts.cpp) drives them through create/Init/run; here
// we only assert the surface headers yield the right abstract shape when compiled
// as an out-of-package consumer sees them.
static_assert(std::is_abstract_v<GaitSequencerInterface>);
static_assert(std::is_abstract_v<MPCInterface>);
static_assert(std::is_abstract_v<SwingLegControllerInterface>);
static_assert(std::is_abstract_v<WBCInterface<JointTorqueVelocityPositionCommands>>);
static_assert(std::is_abstract_v<ContactLogicInterface>);
static_assert(std::is_abstract_v<ModelAdaptationInterface>);

static_assert(std::is_abstract_v<StagePlugin<GaitSequencerInterface>>);
static_assert(std::is_abstract_v<StagePlugin<MPCInterface>>);
static_assert(std::is_abstract_v<StagePlugin<SwingLegControllerInterface>>);
static_assert(std::is_abstract_v<StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>>);
static_assert(std::is_abstract_v<StagePlugin<ContactLogicInterface>>);
static_assert(std::is_abstract_v<StagePlugin<ModelAdaptationInterface>>);

// The loader (issue #7, M2.2) instantiates for every stage base through the same
// export-only include path. Its non-copyable/non-movable shape is contractual,
// not incidental: the pluginlib ClassLoader it owns must outlive every instance
// it created, so a loader that could be moved out from under its stages would be
// a dangling-vtable trap (doc/modularity/stage_loading.md §3).
static_assert(!std::is_copy_constructible_v<StageLoader<GaitSequencerInterface>>);
static_assert(!std::is_move_constructible_v<StageLoader<GaitSequencerInterface>>);
static_assert(!std::is_copy_assignable_v<StageLoader<GaitSequencerInterface>>);
static_assert(!std::is_move_assignable_v<StageLoader<GaitSequencerInterface>>);

static_assert(std::is_same_v<StageLoader<MPCInterface>::Plugin, StagePlugin<MPCInterface>>);
static_assert(std::is_same_v<StageLoader<SwingLegControllerInterface>::Plugin,
                             StagePlugin<SwingLegControllerInterface>>);
static_assert(std::is_same_v<StageLoader<WBCInterface<JointTorqueVelocityPositionCommands>>::Plugin,
                             StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>>);
static_assert(std::is_same_v<StageLoader<ContactLogicInterface>::Plugin, StagePlugin<ContactLogicInterface>>);
static_assert(std::is_same_v<StageLoader<ModelAdaptationInterface>::Plugin, StagePlugin<ModelAdaptationInterface>>);

// The host (M2.4) holds stages as PluginPtr members it default-constructs and
// later move-assigns from Load — including the reconfiguration swap of
// plugin_lifecycle.md §5. Both properties are required for that pattern to
// compile at all, so they are pinned here rather than discovered in M2.4.
static_assert(std::is_default_constructible_v<StageLoader<GaitSequencerInterface>::PluginPtr>);
static_assert(std::is_move_assignable_v<StageLoader<GaitSequencerInterface>::PluginPtr>);
// And it really is a pointer to the frozen interface, so call sites do not change.
static_assert(std::is_convertible_v<StageLoader<GaitSequencerInterface>::PluginPtr::pointer,
                                    GaitSequencerInterface*>);

// A StageLoadError is not a StageInitError and vice versa: the host (and a human
// reading a bring-up crash) must be able to tell "no such stage" apart from "that
// stage could not configure itself" (stage_loading.md §2).
static_assert(!std::is_base_of_v<StageInitError, StageLoadError>);
static_assert(!std::is_base_of_v<StageLoadError, StageInitError>);
static_assert(std::is_base_of_v<std::runtime_error, StageLoadError>);

// Instantiate every type once and read every instance back, so the definitions
// are odr-used rather than merely parsed.
[[maybe_unused]] bool InstantiateSurfaceTypes() {
  Target target{};
  GaitSequence gait_sequence{};
  WrenchSequence wrench_sequence{};
  FeetTargets feet_targets{};
  MPCPrediction mpc_prediction{};

  target.active.x = true;
  gait_sequence.sequence_mode = GaitSequence::MOVE;
  wrench_sequence.forces[0][0].setZero();
  feet_targets.positions[0].setZero();
  mpc_prediction.position[1].setZero();

  return target.active.x && gait_sequence.sequence_mode == GaitSequence::MOVE
         && wrench_sequence.forces[0][0].isZero() && feet_targets.positions[0].isZero()
         && mpc_prediction.position[1].isZero();
}

}  // namespace

// Host-only configuration must not be reachable through the exported surface.
// These are the names `mit_controller/mit_controller_params.hpp` defines at
// global scope; re-defining them here with internal linkage is a redefinition
// error if any surface header ever starts pulling that host header in.
//
// Note this cannot be spelled as `#ifdef ROBOT_MODEL`: the `common` package
// exports `ROBOT_MODEL=<robot>` as an INTERFACE compile definition on its
// `quad_model_symbolic` target, so every consumer of `common` — including a
// future out-of-package plugin — inherits it. The macro's presence therefore
// proves nothing; the absence of these variables does.
[[maybe_unused]] static constexpr bool USE_WBC = false;
[[maybe_unused]] static const bool PUBLISH_GAIT_STATE = false;
[[maybe_unused]] static const bool PUBLISH_HEARTBEAT = false;
