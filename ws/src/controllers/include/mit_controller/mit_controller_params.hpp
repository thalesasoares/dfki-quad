#pragma once

// Host configuration for `mit_controller_node`.
//
// The structural constants of the pipeline (sizes, horizons, loop periods) moved
// to `mit_controller/pipeline_constants.hpp` in M1.3 (issue #3) so that they can
// be exported to out-of-package stage plugins. They are re-included here, so
// every existing include site of this header keeps seeing the same set of names.
//
// What remains below is host-only and is *not* part of the exported surface: the
// `PUBLISH_*` diagnostic switches and the `USE_WBC` selector, which depends on
// the `ROBOT_MODEL` compile definition that only this package's targets set.

#include "mit_controller/pipeline_constants.hpp"

// message publishers
static const bool PUBLISH_SWING_LEG_TRAJECTORIES = false;
static const bool PUBLISH_GAIT_STATE = true;
static const bool PUBLISH_OPEN_LOOP_TRAJECTORY = true;
static const bool PUBLISH_SOLVE_TIME = true;
static const bool PUBLISH_WBC_SOLVE_TIME = true;
static const bool PUBLISH_WBC_TARGET = true;
static const bool PUBLISH_GAIT_SEQUENCE = true;
static const bool PUBLISH_HEARTBEAT = true;

#ifdef ROBOT_MODEL
#if ROBOT_MODEL == GO2
static constexpr bool USE_WBC = true;
#elif ROBOT_MODEL == ULAB
static constexpr bool USE_WBC = false;
#endif
#endif

// #define DEBUG_PRINTS
