#pragma once

// Structural constants of the control pipeline.
//
// This header is part of the *exported* shared pipeline type surface (issue #3,
// M1.3): it is installed to `include/mit_controller/` and may be included by
// out-of-package stage plugins. It carries only sizes and periods that the
// shared data types (`GaitSequence`, `WrenchSequence`, `FeetTargets`,
// `MPCPrediction`) and the stage contracts are defined in terms of.
//
// Host-only configuration — the `PUBLISH_*` diagnostic switches and the
// `ROBOT_MODEL`-conditional `USE_WBC` selector — deliberately stays in
// `mit_controller/mit_controller_params.hpp`, which is *not* exported. A plugin
// compiled in another package has no `ROBOT_MODEL` compile definition, so those
// symbols must not be reachable through the public surface.
//
// See doc/modularity/pipeline_types.md. Changing any value here changes the
// layout of the shared types and requires updating that document in the same
// pull request.

static constexpr int N_LEGS = 4;
static constexpr int N_JOINTS_PER_LEG = 3;
static constexpr int GAIT_SEQUENCE_SIZE = 100;
static constexpr int MPC_PREDICTION_HORIZON = 10;
static constexpr double MPC_DT = 0.05;
static constexpr double MPC_CONTROL_DT = 0.01;
static constexpr double SWING_LEG_DT = 0.002;
static constexpr double CONTROL_DT = 0.002;
static constexpr double WBC_CYCLE_DT = CONTROL_DT;
static constexpr double MODEL_ADAPTATION_DT = 0.01;
static constexpr double MODEL_ADAPTATION_BATCH_SIZE = 100;

static constexpr int FEET_POSITION_SEQUENCE_SIZE = int((MPC_DT / MPC_CONTROL_DT) * GAIT_SEQUENCE_SIZE);
