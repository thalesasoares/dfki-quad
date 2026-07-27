#pragma once

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "common/custom_qos.hpp"
#include "common/eigen_msg_conversions.hpp"
#include "common/quad_model_pino.hpp"
#include "common/quad_state.hpp"
#include "common/sequence_containers.hpp"
#include "interfaces/msg/controller_info.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/mpc_diagnostics.hpp"
#include "interfaces/msg/position_sequence.hpp"
#include "interfaces/msg/quad_control_target.hpp"
#include "interfaces/msg/quad_model.hpp"
#include "interfaces/msg/quad_model_debug.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/msg/vector_sequence.hpp"
#include "interfaces/msg/wbc_return.hpp"
#include "interfaces/msg/wbc_target.hpp"
#include "interfaces/srv/change_leg_driver_mode.hpp"
#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/gait_sequence_to_msg.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/mit_controller_params.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "stage_selection.hpp"

/**
 * The pipeline host (issue #9, M2.4).
 *
 * The node owns everything that is *not* an algorithm: the ROS interface
 * (subscriptions, publishers, the leg driver service), the three control loops
 * with their timers and mutually exclusive callback groups, the locks that
 * separate them, and the lifetime of the six pipeline stages.
 *
 * It constructs **no** concrete algorithm. Every stage arrives through
 * `StageLoader` (`mit_controller/stage_loader.hpp`), selected by a `*.type`
 * parameter (`stage_selection.hpp`) and initialised through the two-phase plugin
 * lifecycle of `doc/modularity/plugin_lifecycle.md`. What the host does with a
 * stage afterwards is unchanged: it calls the frozen interfaces of
 * `doc/modularity/stage_contracts.md`, because `StagePlugin<I>` *is* an `I`.
 *
 * See doc/modularity/pipeline_host.md.
 */
class MITController : public rclcpp::Node {
 public:
  enum LEGControlMode {
    CARTESIAN_JOINT_CONTROL = 3,
    CARTESIAN_STIFFNESS_CONTROL = 2,
    JOINT_TORQUE_CONTROL = 1,
    JOINT_CONTROL = 0
  };

 private:
  // Parameters:
  LEGControlMode leg_control_mode_;
  Eigen::Vector3d cartesian_joint_control_swing_Kp_;
  Eigen::Vector3d cartesian_joint_control_swing_Kd_;
  Eigen::Vector3d cartesian_joint_control_stance_Kp_;
  Eigen::Vector3d cartesian_joint_control_stance_Kd_;
  Eigen::Vector3d cartesian_stiffness_control_swing_Kp_;
  Eigen::Vector3d cartesian_stiffness_control_swing_Kd_;
  Eigen::Vector3d cartesian_stiffness_control_stance_Kp_;
  Eigen::Vector3d cartesian_stiffness_control_stance_Kd_;
  Eigen::Vector3d joint_control_swing_Kp_;
  Eigen::Vector3d joint_control_swing_Kd_;
  Eigen::Vector3d joint_control_stance_Kp_;
  Eigen::Vector3d joint_control_stance_Kd_;
  bool use_model_adaptation_;

  // ROS related members
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_subscription_;
  rclcpp::Subscription<interfaces::msg::QuadControlTarget>::SharedPtr quad_control_target_subscription_;
  rclcpp::Client<interfaces::srv::ChangeLegDriverMode>::SharedPtr change_leg_driver_mode_client_;
  rclcpp::Publisher<interfaces::msg::LegCmd>::SharedPtr leg_cmd_publisher_;
  rclcpp::Publisher<interfaces::msg::JointCmd>::SharedPtr leg_joint_cmd_publisher_;

  interfaces::msg::LegCmd leg_cmd_;
  interfaces::msg::JointCmd leg_joint_cmd_;
  interfaces::msg::ControllerInfo controller_heartbeat_;

  rclcpp::TimerBase::SharedPtr mpc_loop_timer_;
  rclcpp::TimerBase::SharedPtr slc_loop_timer_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;  // SLC runs with a higher frequency
  rclcpp::TimerBase::SharedPtr
      model_adaptation_loop_timer_;  // Potentially, the model adaptation runs with a lower frequency
  rclcpp::TimerBase::SharedPtr heartbeat_loop_timer_;

  rclcpp::Publisher<interfaces::msg::VectorSequence>::SharedPtr swing_leg_trajs_publisher_;
  rclcpp::Publisher<interfaces::msg::GaitState>::SharedPtr gait_state_publisher_;
  rclcpp::Publisher<interfaces::msg::PositionSequence>::SharedPtr open_loop_publisher_;
  rclcpp::Publisher<interfaces::msg::MPCDiagnostics>::SharedPtr solve_time_publisher_;
  rclcpp::Publisher<interfaces::msg::WBCReturn>::SharedPtr wbc_solve_time_publisher_;
  rclcpp::Publisher<interfaces::msg::WBCTarget>::SharedPtr wbc_target_publisher_;
  rclcpp::Publisher<interfaces::msg::GaitSequence>::SharedPtr gait_sequence_publisher_;
  rclcpp::Publisher<interfaces::msg::QuadModel>::SharedPtr quad_model_publisher_;
  rclcpp::Publisher<interfaces::msg::QuadModelDebug>::SharedPtr quad_model_debug_publisher_;
  rclcpp::Publisher<interfaces::msg::ControllerInfo>::SharedPtr controller_heartbeat_publisher_;

  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> on_setparam_callback_handler_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> parameter_event_callback_handle_;

  // Controller related members
  //
  // Since #13 (M3.2) the WBC is an ordinary stage: `WBCInterface` is no longer a
  // class template, so there is one pluginlib base for every WBC and the command
  // family is a runtime property of the loaded plugin (`SupportedCommandMode`).
  // Which of the two getters the control loop calls follows `leg_control_mode_`,
  // validated against the loaded plugin once at bring-up.

  // One loader per stage base class. **Declared before the stage pointers on
  // purpose**: members are destroyed in reverse declaration order, and
  // destroying a pluginlib::ClassLoader unloads the library — a stage instance
  // outliving its loader would be a dangling vtable (stage_loader.hpp,
  // plugin_lifecycle.md §5). The loaders are also non-movable, so this
  // relationship cannot be broken by accident later.
  StageLoader<GaitSequencerInterface> gs_loader_{stage_plugin_bases::kGaitSequencer};
  StageLoader<MPCInterface> mpc_loader_{stage_plugin_bases::kMPC};
  StageLoader<SwingLegControllerInterface> slc_loader_{stage_plugin_bases::kSwingLegController};
  StageLoader<WBCInterface> wbc_loader_{stage_plugin_bases::kWBC};
  StageLoader<ModelAdaptationInterface> ma_loader_{stage_plugin_bases::kModelAdaptation};
  StageLoader<ContactLogicInterface> contact_logic_loader_{stage_plugin_bases::kContactLogic};

  // The stages themselves. `PluginPtr` carries pluginlib's own deleter, which is
  // part of the pointer type — moving one into a plain std::unique_ptr would
  // silently substitute the default deleter and destroy the stage outside
  // pluginlib's bookkeeping. Every *use* below is still the frozen stage
  // interface, because StagePlugin<I> derives from I.
  StageLoader<MPCInterface>::PluginPtr mpc_;
  StageLoader<GaitSequencerInterface>::PluginPtr gs_;
  // The gs.type value gs_ was actually loaded from — lets the parameter event
  // callback tell a real sequencer switch apart from the echo of its own
  // legacy-key re-derivation (see the gs.type branch there).
  std::string loaded_gs_type_;
  StageLoader<SwingLegControllerInterface>::PluginPtr slc_;
  StageLoader<WBCInterface>::PluginPtr wbc_;
  StageLoader<ModelAdaptationInterface>::PluginPtr ma_;
  // The contact reconciliation stage (issue #12, M3.1). It runs inside the
  // control loop under wbc_lock_, between the swing leg controller's output and
  // the WBC's input, and owns the per-leg FSM state that used to sit in the four
  // arrays below this line.
  StageLoader<ContactLogicInterface>::PluginPtr contact_logic_;
  Target target_;
  GaitSequence gait_sequence_;
  bool gs_updated_;
  WrenchSequence wrench_sequence_;
  MPCPrediction mpc_prediction_;
  FeetTargets feet_targets_;
  std::array<double, ModelInterface::N_LEGS> feet_swing_progress_;
  std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_;

  // For sync
  std::mutex quad_state_lock_;
  std::mutex gs_wrench_sequence_lock_;
  std::mutex targets_lock_;
  std::mutex gait_sequencer_lock_;
  std::mutex mpc_lock_;
  std::mutex slc_lock_;  // TODO: instead of this locks, maybe schedule the change to the repsective callback group
  std::mutex wbc_lock_;

  // For multithreading
  rclcpp::CallbackGroup::SharedPtr mpc_call_back_group_;
  rclcpp::CallbackGroup::SharedPtr slc_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_loop_call_back_group_;
  rclcpp::CallbackGroup::SharedPtr model_adaptation_callback_group_;

  // State and model
  bool first_quad_state_received_;
  QuadState quad_state_;
  QuadModelPino quad_model_;

  /**
   * The initialisation context handed to a stage's `Init`: a model and state
   * clone plus the node's full parameter set as a flat name -> value map
   * (plugin_lifecycle.md §3). Every stage receives the same map and reads the
   * keys it documents; defaults for absent keys live in the stage.
   *
   * Only ever called at bring-up or from the reconfiguration path, never from a
   * control loop — building it walks the parameter map, and the stages it feeds
   * do the work the deleted constructors did.
   */
  StageInit MakeStageInit();

  /**
   * Refuses a `wbc.type` / `leg_control_mode` pairing the pipeline cannot serve
   * (issue #13, M3.2). Called once, right after the WBC is loaded.
   *
   * `leg_control_mode` picks the command topic the host publishes on; the WBC
   * plugin decides which command family it can produce. Until M3.2 the two could
   * not disagree without recompiling, because the command type was a build
   * flavour. Now both are launch-time choices, so the check is the host's — done
   * here rather than per cycle, which is also why the control loop's dispatch has
   * no mismatch branch left.
   *
   * @throws StageLoadError naming both parameters and the fix
   */
  void ValidateWBCCommandMode();

 public:
  MITController(const std::string& nodeName);
  void QuadStateUpdateCallback(interfaces::msg::QuadState::SharedPtr quad_state_msg);
  void QuadControlTargetUpdateCallback(interfaces::msg::QuadControlTarget::SharedPtr quad_target_msg);

  void MPCLoopCallback();
  void SLCLoopCallback();
  void ControlLoopCallback();
  void ModelAdaptationCallback();
  void HartbeatCallback();
};
