#!/bin/bash
# Build the dfki-quad software stack for the GO2 simulation.
#
# Run this INSIDE the dfki_quad container (./run_docker.sh or ./new_docker_shell.sh):
#
#     ./src/build_go2_sim.sh              # build everything needed for sim:=go2
#     source ./src/build_go2_sim.sh       # ... and source the sim environment afterwards
#
# It is the scripted equivalent of the `cbg` shortcut, but restricted to the
# packages the GO2 simulation actually needs and with the simulation-friendly
# CMake configuration applied.

# Don't enable `set -e` when sourced: a failed build must not kill the user's shell.
if ! (return 0 2>/dev/null); then
    # no `set -u`: the ROS/colcon setup scripts read unbound variables
    set -eo pipefail
fi

# ---------------------------------------------------------------------------
# defaults
# ---------------------------------------------------------------------------
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
# simulation env: FastRTPS, no CycloneDDS (the `sr` shortcut in the container)
SIM_ENV_SETUP="${SIM_ENV_SETUP:-/root/setup_ulab_workspace.bash}"

# Packages required to launch `ros2 launch simulator simulator.launch.py sim:=go2`
# and the accompanying driver / state estimation / controller nodes.
# --packages-up-to pulls in the dependencies (interfaces, common) automatically.
PACKAGES=(simulator drivers state_estimation controllers)

WITH_VICON=ON
WITH_EXAMPLES=0
WITH_EXPERIMENTS=0
CLEAN=0
JOBS=""
EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./src/build_go2_sim.sh [options] [-- <extra colcon args>]

Options:
  -c, --clean            Remove build/ install/ log/ before building
      --no-vicon         Build state_estimation without Vicon support
      --with-examples    Also build examples/example_stage_plugins
      --with-experiments Also build solver_experiments
  -j, --jobs N           Limit colcon parallel workers (and make jobs) to N
  -a, --all              Build every package in the workspace
  -h, --help             Show this help

Everything after `--` is passed straight to colcon build.
Note: the packages pin CMAKE_BUILD_TYPE=Release themselves, so -DCMAKE_BUILD_TYPE
has no effect here; edit the package CMakeLists if you need a debug build.

After a successful build, source the simulation environment with:
    source install/setup.bash && source /root/setup_ulab_workspace.bash
(or simply `sr`, or run this script with `source ./src/build_go2_sim.sh`).
EOF
}

BUILD_ALL=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--clean)          CLEAN=1 ;;
        --no-vicon)          WITH_VICON=OFF ;;
        --with-examples)     WITH_EXAMPLES=1 ;;
        --with-experiments)  WITH_EXPERIMENTS=1 ;;
        -a|--all)            BUILD_ALL=1 ;;
        -j|--jobs)           JOBS="${2:?--jobs needs a number}"; shift ;;
        -h|--help)           usage; return 0 2>/dev/null || exit 0 ;;
        --)                  shift; EXTRA_ARGS+=("$@"); break ;;
        *)                   echo "Unknown option: $1" >&2; usage >&2
                             return 1 2>/dev/null || exit 1 ;;
    esac
    shift
done

# ---------------------------------------------------------------------------
# locate the workspace (parent of the src/ directory holding this script)
# ---------------------------------------------------------------------------
if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
    SCRIPT_PATH="${BASH_SOURCE[0]}"
else
    SCRIPT_PATH="$0"
fi
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

if [[ ! -d "$WS_DIR/src" ]]; then
    echo "error: expected a colcon workspace at $WS_DIR (no src/ directory)" >&2
    return 1 2>/dev/null || exit 1
fi

if [[ ! -f "$ROS_DISTRO_SETUP" ]]; then
    echo "error: $ROS_DISTRO_SETUP not found." >&2
    echo "       This script is meant to run inside the dfki_quad container." >&2
    return 1 2>/dev/null || exit 1
fi

cd "$WS_DIR"

# ---------------------------------------------------------------------------
# assemble the build configuration
# ---------------------------------------------------------------------------
[[ $WITH_EXAMPLES  -eq 1 ]] && PACKAGES+=(example_stage_plugins)
[[ $WITH_EXPERIMENTS -eq 1 ]] && PACKAGES+=(solver_experiments)

COLCON_ARGS=(build --symlink-install)
if [[ $BUILD_ALL -eq 0 ]]; then
    COLCON_ARGS+=(--packages-up-to "${PACKAGES[@]}")
fi
if [[ -n "$JOBS" ]]; then
    COLCON_ARGS+=(--parallel-workers "$JOBS")
    export MAKEFLAGS="-j$JOBS"
fi
[[ ${#EXTRA_ARGS[@]} -gt 0 ]] && COLCON_ARGS+=("${EXTRA_ARGS[@]}")
# --cmake-args swallows every following token, so it has to stay last
COLCON_ARGS+=(
    --cmake-args
    -DROBOT_NAME=go2            # GO2 kinematics/URDF and go2 driver
    -DWITHOUT_DRAKE=OFF         # the simulator is Drake based, it must stay on
    -DWITH_VICON="$WITH_VICON"
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1
)

# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------
if [[ $CLEAN -eq 1 ]]; then
    echo ">> removing build/ install/ log/"
    rm -rf build install log
fi

echo ">> workspace : $WS_DIR"
if [[ $BUILD_ALL -eq 1 ]]; then
    echo ">> packages  : (all)"
else
    echo ">> packages  : ${PACKAGES[*]} (+ dependencies)"
fi
echo ">> config    : ROBOT_NAME=go2, vicon=$WITH_VICON, drake=on, symlink-install"
echo

# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"
if ! colcon "${COLCON_ARGS[@]}"; then
    echo >&2
    echo ">> build FAILED, see $WS_DIR/log/latest_build/ for details" >&2
    return 1 2>/dev/null || exit 1
fi

# ---------------------------------------------------------------------------
# environment
# ---------------------------------------------------------------------------
echo
if (return 0 2>/dev/null); then
    # script was sourced: set up the simulation environment right away
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
    # shellcheck disable=SC1090
    source "$SIM_ENV_SETUP"
    echo ">> build finished, GO2 simulation environment sourced"
    echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "   ros2 launch simulator simulator.launch.py sim:=go2"
else
    cat <<EOF
>> build finished. Source the GO2 simulation environment with:

     source $WS_DIR/install/setup.bash
     source $SIM_ENV_SETUP          # alias: sr  (FastRTPS, simulation)

   then, in separate shells:

     ros2 launch simulator simulator.launch.py sim:=go2
     ros2 launch drivers leg_driver_launch.py sim:=go2
     ros2 launch state_estimation state_estimation.launch.py sim:=go2
     ros2 launch controllers quad_stand_up.launch.py sim:=go2
     ros2 launch controllers mit_controller.launch.py sim:=go2
EOF
fi
