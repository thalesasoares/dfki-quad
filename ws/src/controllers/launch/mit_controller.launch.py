import os
import sys

import numpy as np
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import QuadState
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- Stage selection at launch (issue #19, M4.4) ------------------------------
#
# The controller is a modular pipeline: each stage is a pluginlib class picked by
# one `<stage>.type` string (doc/modularity/stage_loading.md §4). These two launch
# arguments make that choice reachable without editing the shipped robot YAML:
#
#   <stage>:=<plugin>        one stage, one plugin id, nothing else
#   stage_overlay:=<file>    a params file layered over the robot config, so a
#                            swap that also needs the stage's *parameters* stays
#                            one argument
#
# Both are pure parameter plumbing resolved before the node starts; neither adds
# anything to the control loop. Full story: doc/modularity/stage_overlays.md.

# The six stages, spelled as in stage_selection.hpp. `<key>:=v` sets `<key>.type`.
STAGE_LAUNCH_KEYS = ("gs", "mpc", "slc", "wbc", "model_adaptation", "contact_logic")

STAGE_OVERLAY_KEY = "stage_overlay"
# Overlays shipped with the package, resolved by bare filename from here.
STAGE_OVERLAY_DIR = "overlays"


def launch_arg_value(argv, name):
    """
    The value of a `name:=value` launch argument, or None if it was not given.

    Repeating an argument is a hard error rather than a last-one-wins: which of
    two `gs:=` values applies is not something a demo should have to guess.
    """
    prefix = name + ":="
    values = [arg[len(prefix):] for arg in argv if arg.startswith(prefix)]
    if len(values) > 1:
        print('Error: launch param "%s" specified more than once: %s' % (name, values))
        exit(-1)
    return values[0] if values else None


def stage_type_overrides(argv):
    """
    `{'<stage>.type': '<plugin>'}` for every stage named on the command line.

    The plugin id is passed through verbatim. Validating it here would mean a
    second copy of the plugin vocabulary that drifts from the declared classes;
    the host's StageLoader already refuses an unknown id at bring-up and prints
    what *is* declared, which is the single source of truth.
    """
    overrides = {}
    for stage in STAGE_LAUNCH_KEYS:
        plugin = launch_arg_value(argv, stage)
        if plugin is None:
            continue
        if not plugin:
            print('Error: launch param "%s" needs a plugin name, e.g. %s:=simple_gait.'
                  % (stage, stage))
            exit(-1)
        overrides[stage + ".type"] = plugin
    return [overrides] if overrides else []


def stage_overlay_files(argv, pkg_controllers):
    """
    The resolved `stage_overlay:=<file>` params file, as a one-element list.

    Accepts a path to any file, or the bare name of an overlay shipped in
    `config/overlays/` so the common case is short. An overlay that resolves to
    neither aborts the launch naming both places it looked: silently starting the
    stock stack because a path was misspelled is the failure mode this whole
    layer exists to avoid.
    """
    overlay = launch_arg_value(argv, STAGE_OVERLAY_KEY)
    if overlay is None:
        return []
    shipped = os.path.join(pkg_controllers, "config", STAGE_OVERLAY_DIR, overlay)
    for candidate in (overlay, shipped):
        if os.path.isfile(candidate):
            return [candidate]
    print('Error: stage_overlay "%s" is not a file. Looked for:\n  %s\n  %s'
          % (overlay, overlay, shipped))
    exit(-1)


def safe_start():
    rclpy.init()
    node = rclpy.create_node("safe_start_launcher")
    node.get_logger().info("Safe start launcher is running")
    poses = []
    twists = []
    quad_state_subscription = node.create_subscription(QuadState, "/quad_state",
                                                       lambda state_msg: (
                                                           poses.append(
                                                               [state_msg.pose.pose.position.x,
                                                                state_msg.pose.pose.position.y,
                                                                state_msg.pose.pose.position.z,
                                                                state_msg.pose.pose.orientation.w,
                                                                state_msg.pose.pose.orientation.x,
                                                                state_msg.pose.pose.orientation.y,
                                                                state_msg.pose.pose.orientation.z]),
                                                           twists.append([state_msg.twist.twist.linear.x,
                                                                          state_msg.twist.twist.linear.y,
                                                                          state_msg.twist.twist.linear.z,
                                                                          state_msg.twist.twist.angular.x,
                                                                          state_msg.twist.twist.angular.y,
                                                                          state_msg.twist.twist.angular.z]
                                                                         )
                                                       ), 100)
    node.get_logger().info("Wait for quad_state message and collect it for 3 seconds")
    start_time = node.get_clock().now()
    end_time = start_time
    while ((end_time - start_time).nanoseconds <= rclpy.time.Time(seconds=3).nanoseconds):
        rclpy.spin_once(node, timeout_sec=1)
        if (len(poses) == 0):
            node.get_logger().warn("No quad state has been received yet", throttle_duration_sec=1)
            start_time = node.get_clock().now()
        end_time = node.get_clock().now()

    if (len(poses) <= 10 or len(twists) <= 10):
        node.get_logger().error('To less quad_states received: %d' % (len(poses)))
        exit(-1)

    poses = np.array(poses)
    twists = np.array(twists)
    poses_var = poses.var(0)
    twist_mean = twists.mean(0)
    if ((poses_var < 0.05).all()):
        node.get_logger().info("No pose drift: OK")
    else:
        node.get_logger().error("Pose drift in last 3 second quad state data to high: var=" + str(poses_var))
        exit(-1)

    if ((twist_mean < 0.05).all()):
        node.get_logger().info("No velocity: OK")
    else:
        node.get_logger().error("Velocity in last 3 second quad state data to high: mean=" + str(twist_mean < 0.1))
        exit(-1)

    node.get_logger().info("Safe standup successful, handing over to controller launch")


def generate_launch_description():
    pkg_controllers = get_package_share_directory("controllers")

    # Resolved before safe_start(): a mistyped stage argument should abort now,
    # not after three seconds of standing on a robot that is about to be told to
    # walk. Stage selection, weakest first — launch_ros emits one --params-file /
    # -p per entry of `parameters` in list order and rcl lets the later
    # assignment win, so this is the precedence chain:
    #   robot config < common config < stage_overlay file < <stage>:= argument
    # An explicit one-stage argument therefore beats an overlay that also names
    # that stage, and only that stage — the rest of the overlay still applies.
    stage_overlay_params = stage_overlay_files(sys.argv[4:], pkg_controllers)
    stage_type_params = stage_type_overrides(sys.argv[4:])

    if not "safe_start:=false" in sys.argv[4:]:
        safe_start()
    if "sim:=ulab" in sys.argv[4:]:
        sim = True
        unitree = False
        config_file = "mit_controller_sim_ulab.yaml"
        common_config_file = "common_config_ulab.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "mit_controller_sim_go2.yaml"
        common_config_file = "common_config_go2.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file = "mit_controller_real_ulab.yaml"
        common_config_file = "common_config_ulab.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "mit_controller_real_go2.yaml"
        common_config_file = "common_config_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    if "mpc_solver:=PARTIAL_CONDENSING_HPIPM" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=PARTIAL_CONDENSING_HPIPM']
    elif "mpc_solver:=FULL_CONDENSING_HPIPM" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=FULL_CONDENSING_HPIPM']
    elif "mpc_solver:=PARTIAL_CONDENSING_OSQP" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=PARTIAL_CONDENSING_OSQP']
    elif "mpc_solver:=FULL_CONDENSING_QPOASES" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=FULL_CONDENSING_QPOASES']
    else:
        mpc_solver_param = []

    substring = "mpc_condensed_size:="
    condensed_param = [s for s in sys.argv[4:] if substring in s]

    if len(condensed_param) == 1:
        mpc_condensed_size_param = ['-p', condensed_param[0]]
        print(mpc_condensed_size_param)
    elif len(condensed_param) > 1:
        print("Error: Launch param \"mpc_condensed_size\" specified more than once.")
    else:
        mpc_condensed_size_param = []

    if "mpc_hpipm_mode:=SPEED_ABS" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=SPEED_ABS']
    elif "mpc_hpipm_mode:=SPEED" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=SPEED']
    elif "mpc_hpipm_mode:=BALANCE" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=BALANCE']
    elif "mpc_hpipm_mode:=ROBUST" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=ROBUST']
    else:
        mpc_hpipm_mode_param = []

    # if "mpc_condensed_size:=NONE" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=20']
    # elif "mpc_condensed_size:=HALF" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=10']
    # elif "mpc_condensed_size:=FULL" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=1']

    pkg_common = get_package_share_directory("common")
    controller_config_path = os.path.join(pkg_controllers, "config", config_file)
    common_config_path = os.path.join(pkg_common, "config", common_config_file)
    cpu_power_logging_config_path = os.path.join(pkg_controllers, "config", "cpu_power_logging.yaml")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=str(
        sim))  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    joy = Node(
        package="joy",
        name="joy_node",
        executable="joy_node",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # joystick to goal traj node
    joy_to_target = Node(
        package="controllers",
        name="joy_to_target",
        executable="joy_to_target.py",
        parameters=[controller_config_path, {"use_sim_time": use_sim_time}],
        # parameters=[config["js2mpc"][mode]],
        output="screen",
    )

    return LaunchDescription([
        Node(
            package='controllers',  # Replace with the actual package name
            executable='mitcontrollernode',  # Replace with the executable name
            name='mit_controller_node',
            parameters=[controller_config_path, common_config_path]
                       + stage_overlay_params + stage_type_params
                       + [{"use_sim_time": use_sim_time}],
            output='screen',
            arguments=['--ros-args', '--log-level', ["mit_controller_node:=",
                                                     "info"]] + mpc_solver_param + mpc_condensed_size_param + mpc_hpipm_mode_param
        ),
        Node(
            package='controllers',  # Replace with the actual package name
            executable='log_cpu_power',  # Replace with the executable name
            name='cpu_power_logging_node',
            parameters=[cpu_power_logging_config_path],
            output='screen'
        ),
        joy, joy_to_target, declare_use_sim_time_cmd])
