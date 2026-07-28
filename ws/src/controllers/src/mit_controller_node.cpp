#include "mit_controller_node.hpp"

MITController::MITController(const std::string &nodeName)
    : Node(nodeName),
      first_quad_state_received_(false),
      quad_model_(*this) {
  this->declare_parameter("initial_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("slc_swing_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("wbc.inverse_dynamics.transformation_filter_size", 20);
  this->declare_parameter("slc_world_blend", 1.0);
  this->declare_parameter("mpc_alpha", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_state_weights_stand", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mpc_state_weights_move", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mpc_mu", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_fmin", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_fmax", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("gs_shoulder_positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("leg_control_mode", rclcpp::ParameterType::PARAMETER_INTEGER);
  this->declare_parameter("raibert.k", 0.03);
  this->declare_parameter("raibert.z_on_plane", false);
  this->declare_parameter("raibert.filtersize", 20);
  this->declare_parameter("fix_standing_position", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("fix_position_distance_threshold", 0.1);
  this->declare_parameter("fix_position_angular_threshold", 0.26);
  this->declare_parameter("fix_position_velocity_threshold", 0.1);
  this->declare_parameter("maximum_swing_leg_progress_to_update_target", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("early_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("late_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("lost_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("late_contact_reschedule_swing_phase", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.foot_position_based_on_target_height",
                          rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.foot_position_based_on_target_orientation",
                          rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.target_velocity_blend", 0.0);
  this->declare_parameter("wbc.arc_opt.model_urdf", rclcpp::ParameterType::PARAMETER_STRING);
  this->declare_parameter("wbc.arc_opt.feet_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  this->declare_parameter("wbc.arc_opt.joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  this->declare_parameter("wbc.arc_opt.mu", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("wbc.arc_opt.com_pose_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.com_pose_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.com_pose_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.foot_pose_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.foot_force_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.feet_pose_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.feet_pose_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter<std::vector<double>>("wbc.arc_opt.com_pose_saturation",
                                               {std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max()});
  this->declare_parameter<std::vector<double>>(
      "wbc.arc_opt.feet_pose_saturation",
      {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});

  this->declare_parameter("use_model_adaptation", false);
  this->declare_parameter("controller_heartbeat_dt", 0.5);
  this->declare_parameter<double>("ma_forgetting_factor", 0.5);
  this->declare_parameter<std::vector<double>>("ma_convergence_threshold", {0.695, 0.12, 0.11});
  this->declare_parameter<std::vector<double>>("ma_process_noise", {0.005, 0.0005, 0.0005});
  this->declare_parameter<std::vector<double>>("ma_measurement_noise",
                                               {1000.0, 1000.0, 10000.0, 10000.0, 10000.0, 1000.0});
  this->declare_parameter<int>("ma_mode", 0);
  this->declare_parameter<std::string>("mpc_solver", "PARTIAL_CONDENSING_HPIPM");
  this->declare_parameter<int>("mpc_condensed_size", MPC_PREDICTION_HORIZON / 2);
  this->declare_parameter<std::string>("mpc_hpipm_mode", "SPEED");
  this->declare_parameter<std::string>("wbc.arc_opt.solver", "EiquadprogSolver");
  this->declare_parameter<std::string>("wbc.arc_opt.scene", "AccelerationSceneReducedTSID");
  this->declare_parameter<int>("mpc_warm_start", 1);
  this->declare_parameter<double>("mpc_solver_tolerances", -1.0);
  this->declare_parameter<std::string>("mpc_osqp_linsys_solver", "qdldl");
  this->declare_parameter<double>("wbc_solver_tolerances", -1.0);

  leg_control_mode_ = static_cast<LEGControlMode>(this->get_parameter("leg_control_mode").as_int());
  use_model_adaptation_ = this->get_parameter("use_model_adaptation").as_bool();

  // The contact stage's four detection toggles, in the ratified `contact_logic.*`
  // spelling (contact_logic_interface.hpp). The host does not read them any more
  // — they belong to the stage, which gets them through StageInit — it only
  // declares them, so that they exist as ROS parameters and can be changed at
  // runtime.
  //
  // Their defaults are the flat keys the host used to read into its own members,
  // so every config written before M3.1 keeps configuring the same policy without
  // being touched. An explicitly set nested key wins. #23 (M5.4) drops the flat
  // spelling and with it this derivation (stage_selection.hpp).
  this->declare_parameter<bool>(
      contact_logic_params::kEarlyContactDetection,
      this->get_parameter(stage_selection::kLegacyEarlyContactDetectionKey).as_bool());
  this->declare_parameter<bool>(
      contact_logic_params::kLateContactDetection,
      this->get_parameter(stage_selection::kLegacyLateContactDetectionKey).as_bool());
  this->declare_parameter<bool>(
      contact_logic_params::kLostContactDetection,
      this->get_parameter(stage_selection::kLegacyLostContactDetectionKey).as_bool());
  this->declare_parameter<bool>(
      contact_logic_params::kLateContactRescheduleSwingPhase,
      this->get_parameter(stage_selection::kLegacyLateContactRescheduleSwingPhaseKey).as_bool());
  RCLCPP_INFO_EXPRESSION(this->get_logger(),
                         this->get_parameter(contact_logic_params::kEarlyContactDetection).as_bool(),
                         "Early contact detection is activated");
  RCLCPP_INFO_EXPRESSION(this->get_logger(),
                         this->get_parameter(contact_logic_params::kLateContactDetection).as_bool(),
                         "Late contact detection is activated");
  RCLCPP_INFO_EXPRESSION(this->get_logger(),
                         this->get_parameter(contact_logic_params::kLostContactDetection).as_bool(),
                         "Lost contact detection is activated");
  // Load PD gains
  switch (leg_control_mode_) {
    case CARTESIAN_JOINT_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in CARTESIAN_JOINT_CONTROL mode");
      this->declare_parameter("cartesian_joint_control_gains.swing_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.swing_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.stance_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.stance_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("cartesian_joint_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().size() == 3);
      cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.swing_Kp").as_double_array().data());
      cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.swing_Kd").as_double_array().data());
      cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.stance_Kp").as_double_array().data());
      cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().data());
      break;
    case CARTESIAN_STIFFNESS_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in CARTESIAN_STIFFNESS_CONTROL mode");
      this->declare_parameter("cartesian_stiffness_control_gains.swing_Kp",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.swing_Kd",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.stance_Kp",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.stance_Kd",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("cartesian_stiffness_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.stance_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.stance_Kd").as_double_array().size() == 3);
      cartesian_stiffness_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.swing_Kp").as_double_array().data());
      cartesian_stiffness_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.swing_Kd").as_double_array().data());
      cartesian_stiffness_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.stance_Kp").as_double_array().data());
      cartesian_stiffness_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.stance_Kd").as_double_array().data());
      break;
    case JOINT_CONTROL:
      [[fallthrough]];
    case JOINT_TORQUE_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in JOINT_CONTROL or JOINT_TORQUE_CONTROL mode");
      this->declare_parameter("joint_control_gains.swing_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.swing_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.stance_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.stance_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("joint_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.stance_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.stance_Kd").as_double_array().size() == 3);
      joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.swing_Kp").as_double_array().data());
      joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.swing_Kd").as_double_array().data());
      joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.stance_Kp").as_double_array().data());
      joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.stance_Kd").as_double_array().data());
      break;
  }

  // Prepare target
  target_.active.hybrid_x_dot = true;
  target_.active.hybrid_y_dot = true;
  target_.active.wz = true;
  target_.active.z = true;
  target_.active.z_dot = false;
  target_.active.roll = true;
  target_.active.pitch = true;
  target_.active.yaw = false;
  target_.z = this->get_parameter("initial_height").as_double();
  target_.hybrid_x_dot = 0.0;
  target_.hybrid_y_dot = 0.0;
  target_.z_dot = 0.0;
  target_.wz = 0.0;
  target_.roll = 0.0;
  target_.pitch = 0.0;
  target_.active.wx = true;
  target_.active.wy = true;

  //
  target_.active.x = false;
  target_.active.y = false;
  target_.x = 0;
  target_.y = 0;

  // prepare ROS related things
  control_loop_call_back_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  slc_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  mpc_call_back_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  model_adaptation_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Command publisher matching the configured leg control mode.
  //
  // The two `assert(typeid(wbc_.get()) == ...)` checks that used to guard this
  // switch are gone (issue #9): they ran before any WBC existed, and comparing
  // `typeid` of a *pointer* compares static types, so both were tautologies that
  // could never fire. The real guard is `ValidateWBCCommandMode`, which runs once
  // the WBC has actually been loaded (further down this constructor) and refuses
  // to start a pipeline whose WBC cannot fill the message created here.
  switch (leg_control_mode_) {
    case JOINT_CONTROL:
      [[fallthrough]];
    case JOINT_TORQUE_CONTROL:
      leg_joint_cmd_publisher_ =
          this->create_publisher<interfaces::msg::JointCmd>("leg_joint_cmd", QOS_RELIABLE_NO_DEPTH);
      break;
    case CARTESIAN_STIFFNESS_CONTROL:
      [[fallthrough]];
    case CARTESIAN_JOINT_CONTROL:
      model_adaptation_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      leg_cmd_publisher_ = this->create_publisher<interfaces::msg::LegCmd>("leg_cmd", QOS_RELIABLE_NO_DEPTH);
      break;
  }
  quad_model_publisher_ =
      this->create_publisher<interfaces::msg::QuadModel>("quad_model_update", QOS_RELIABLE_NO_DEPTH);
  quad_model_debug_publisher_ =
      this->create_publisher<interfaces::msg::QuadModelDebug>("quad_model_debug", QOS_BEST_EFFORT_NO_DEPTH);
  quad_state_subscription_ = this->create_subscription<interfaces::msg::QuadState>(
      "quad_state",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&MITController::QuadStateUpdateCallback, this, std::placeholders::_1));

  if (PUBLISH_SWING_LEG_TRAJECTORIES) {
    swing_leg_trajs_publisher_ =
        this->create_publisher<interfaces::msg::VectorSequence>("swing_leg_trajs", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_GAIT_STATE) {
    gait_state_publisher_ = this->create_publisher<interfaces::msg::GaitState>("gait_state", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_OPEN_LOOP_TRAJECTORY) {
    open_loop_publisher_ =
        this->create_publisher<interfaces::msg::PositionSequence>("open_loop_trajectory", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_SOLVE_TIME) {
    solve_time_publisher_ =
        this->create_publisher<interfaces::msg::MPCDiagnostics>("solve_time", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_WBC_SOLVE_TIME) {
    wbc_solve_time_publisher_ =
        this->create_publisher<interfaces::msg::WBCReturn>("wbc_solve_time", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_WBC_TARGET) {
    wbc_target_publisher_ = this->create_publisher<interfaces::msg::WBCTarget>("wbc_target", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_GAIT_SEQUENCE) {
    gait_sequence_publisher_ =
        this->create_publisher<interfaces::msg::GaitSequence>("gait_sequence", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_HEARTBEAT) {
    controller_heartbeat_publisher_ =
        this->create_publisher<interfaces::msg::ControllerInfo>("controller_heartbeat", QOS_BEST_EFFORT_NO_DEPTH);
    heartbeat_loop_timer_ =
        rclcpp::create_timer(this,
                             this->get_clock(),
                             std::chrono::duration<double>(this->get_parameter("controller_heartbeat_dt").as_double()),
                             std::bind(&MITController::HartbeatCallback, this));
  }

  change_leg_driver_mode_client_ = this->create_client<interfaces::srv::ChangeLegDriverMode>("switch_op_mode");
  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  // prepare gaits
  this->declare_parameter("gait_sequencer", "Simple");
  this->declare_parameter("simple_gait_sequencer.gait", "STAND");
  this->declare_parameter("simple_gait_sequencer.manual_gait.period", 0.5);
  this->declare_parameter<std::vector<double>>("simple_gait_sequencer.manual_gait.duty_factor", {0.6, 0.6, 0.6, 0.6});
  this->declare_parameter<std::vector<double>>("simple_gait_sequencer.manual_gait.phase_offset", {0.0, 0.5, 0.5, 0.0});
  this->declare_parameter("adaptive_gait_sequencer.gait.swing_time", 0.2);
  this->declare_parameter("adaptive_gait_sequencer.gait.min_v_cmd_factor", 0.5);
  this->declare_parameter("adaptive_gait_sequencer.gait.filter_size", 10);
  this->declare_parameter("adaptive_gait_sequencer.gait.zero_velocity_threshold", 0.03);
  this->declare_parameter("adaptive_gait_sequencer.gait.standing_foot_position_threshold", 0.08);
  this->declare_parameter("adaptive_gait_sequencer.gait.max_correction_cycles", 2.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.correct_all", false);
  this->declare_parameter("adaptive_gait_sequencer.gait.correction_period", 0.6);
  this->declare_parameter("adaptive_gait_sequencer.gait.disturbance_correction", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.min_v", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.offset_delay", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.max_stride_length", 0.0);

  this->declare_parameter("adaptive_gait_sequencer.gait.switch_offsets", true);
  this->declare_parameter<std::vector<double>>("adaptive_gait_sequencer.gait.phase_offset", {0.0, 0.5, 0.5, 0.0});
  this->declare_parameter<std::vector<double>>("adaptive_gait_sequencer.gait.gait_change_froude", {0.02, 0.006});

  // Which implementation each pipeline stage uses. These five strings are the
  // only thing the host knows about stage selection: they name a pluginlib class
  // that StageLoader resolves, and an unknown one is a loud bring-up failure, not
  // a fallback (stage_loading.md §1).
  //
  // Their *defaults* are derived from the parameters that used to pick the
  // implementation inside the host factories. An explicitly set value always
  // wins, and since #10 (M2.5) the Go2 YAMLs set all five explicitly, so the
  // derivation only still selects for the ULab configs and for out-of-tree
  // configs written before the schema. See stage_selection.hpp; #23 (M5.4)
  // drops it together with the legacy keys.
  this->declare_parameter<std::string>(
      stage_selection::kGaitSequencerTypeKey,
      stage_selection::GaitSequencerTypeFromLegacy(
          this->get_parameter(stage_selection::kLegacyGaitSequencerKey).as_string()));
  this->declare_parameter<std::string>(stage_selection::kMPCTypeKey, stage_selection::kAcadosMPCPlugin);
  this->declare_parameter<std::string>(stage_selection::kSwingLegControllerTypeKey,
                                       stage_selection::kBezierSwingPlugin);
  this->declare_parameter<std::string>(stage_selection::kWBCTypeKey, stage_selection::WBCTypeForBuild(USE_WBC));
  this->declare_parameter<std::string>(
      stage_selection::kModelAdaptationTypeKey,
      stage_selection::ModelAdaptationTypeFromLegacy(
          this->get_parameter(stage_selection::kLegacyModelAdaptationModeKey).as_int()));
  // The contact stage has no legacy selector to derive from: until #12 (M3.1)
  // there was no contact stage to select, only inline host code.
  this->declare_parameter<std::string>(stage_selection::kContactLogicTypeKey,
                                       stage_selection::kDefaultContactLogicPlugin);

  on_setparam_callback_handler_ =
      this->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult res;
        for (auto &param : params) {
          if (param.get_name().find("mpc_state_weights") != std::string::npos
              and param.as_double_array().size() != (MPC_STATE_SIZE - 1)) {
            res.successful = false;
            res.reason = "MPC state weight vector has wrong size";
            return res;
          } else if (param.get_name().find("cartesian_joint_control_gains") != std::string::npos
                     and param.as_double_array().size() != N_JOINTS_PER_LEG)

          {
            res.successful = false;
            res.reason = "cartesian_joint_control_gains vector has wrong size";
            return res;
          } else if (param.get_name().find("cartesian_stiffness_control_gains") != std::string::npos
                     and param.as_double_array().size() != N_JOINTS_PER_LEG)

          {
            res.successful = false;
            res.reason = "cartesian_stiffness_control_gains vector has wrong size";
            return res;
          }
        }
        res.successful = true;
        return res;
      });

  parameter_event_callback_handle_ = parameter_event_handler_->add_parameter_event_callback([this](
                                                                                                const rcl_interfaces::
                                                                                                    msg::ParameterEvent
                                                                                                        &param_event) {
    bool gait_update = false;
    for (const auto &param : param_event.changed_parameters) {
      // The host is a pure router: it hands each changed parameter to the owning stage through the
      // SetParameter contract (issue #2) and never needs the concrete stage type. A stage returning
      // false means it did not recognise the key; the host then warns, or for gait parameters falls
      // back to reloading the gait sequencer from scratch (the previous behaviour).
      if (param.name == stage_selection::kLegacyGaitSequencerKey) {
        // The legacy bridge derives gs.type from gait_sequencer at declare time
        // (stage_selection.hpp); a runtime change of the legacy key must re-derive it, or the
        // reload below resolves the startup value of gs.type and rebuilds the sequencer that is
        // already running instead of the newly selected one. Since #10 (M2.5) moved
        // joy_to_target onto gs.type, nothing in this repository takes this branch; it stays for
        // out-of-tree callers that still set the legacy key, until #23 (M5.4) removes both.
        this->set_parameter(rclcpp::Parameter(
            stage_selection::kGaitSequencerTypeKey,
            stage_selection::GaitSequencerTypeFromLegacy(param.value.string_value)));
        gait_update = true;
      } else if (param.name == stage_selection::kGaitSequencerTypeKey) {
        // The new spelling switches the sequencer directly. The set_parameter above re-enters
        // this callback with a gs.type event; comparing against what is actually loaded turns
        // that echo into a no-op instead of a second rebuild.
        if (param.value.string_value != loaded_gs_type_) {
          gait_update = true;
        }
      } else if (param.name.rfind("contact_logic.", 0) == 0) {
        // The contact stage's own keys. Routed under wbc_lock_, the lock the
        // stage runs under (contact_logic_interface.hpp). `contact_logic.type`
        // lands here too and is correctly refused: swapping a running stage is
        // the gait sequencer's special case, not a general capability.
        std::lock_guard<std::mutex> lock(wbc_lock_);
        if (!contact_logic_->SetParameter(param.name, rclcpp::ParameterValue(param.value))) {
          RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
          continue;
        }
      } else if (stage_selection::ContactLogicKeyFromLegacy(param.name) != nullptr) {
        // A config still using the pre-M3.1 flat spelling. Re-set the ratified
        // key, which re-enters this callback and takes the branch above; no echo
        // suppression is needed, because that branch only forwards the value to
        // the stage and does so idempotently. #23 (M5.4) removes this branch.
        this->set_parameter(rclcpp::Parameter(stage_selection::ContactLogicKeyFromLegacy(param.name),
                                              rclcpp::ParameterValue(param.value)));
      } else if (param.name.find("gait") != std::string::npos) {
        gait_sequencer_lock_.lock();
        bool applied = gs_->SetParameter(param.name, rclcpp::ParameterValue(param.value));
        gait_sequencer_lock_.unlock();
        if (!applied) {
          gait_update = true;
        }
      } else if (param.name.rfind("mpc_", 0) == 0) {
        std::lock_guard<std::mutex> lock(mpc_lock_);
        if (!mpc_->SetParameter(param.name, rclcpp::ParameterValue(param.value))) {
          RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
          continue;
        }
      } else if (param.name.rfind("slc_", 0) == 0
                 || param.name == "maximum_swing_leg_progress_to_update_target") {
        std::lock_guard<std::mutex> lock(slc_lock_);
        if (!slc_->SetParameter(param.name, rclcpp::ParameterValue(param.value))) {
          RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
          continue;
        }
      } else if (param.name.rfind("wbc.", 0) == 0) {
        std::lock_guard<std::mutex> lock(wbc_lock_);
        if (!wbc_->SetParameter(param.name, rclcpp::ParameterValue(param.value))) {
          RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
          continue;
        }
      } else if (param.name
                 == "cartesian_joint_control_gains.swing_Kp") {  // TODO: syncronisation, maybe use atomic vars?
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "use_model_adaptation") {
        use_model_adaptation_ = param.value.bool_value;
      } else if (param.name == "raibert.z_on_plane") {
        gait_update = true;
      } else if (param.name == "raibert.k") {
        gait_update = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Changed parameter %s sucessfully", param.name.c_str());
    }
    if (gait_update) {
      RCLCPP_INFO(this->get_logger(), "Gait related parameter has changed, reloading gait sequencer");
      // Reconfiguration is *replace*, not re-Init (plugin_lifecycle.md §5): build
      // and initialise a fresh instance off-loop — the expensive part — then swap
      // it in under the stage lock and let the old one die after the lock is
      // released. Same three steps as before this refactor, with the loader in
      // place of the deleted factory.
      try {
        auto fresh = gs_loader_.Load(stage_selection::kGaitSequencerTypeKey, MakeStageInit());
        fresh->UpdateTarget(target_);  // as at bring-up, before the stage goes live
        gait_sequencer_lock_.lock();
        gs_.swap(fresh);
        gait_sequencer_lock_.unlock();
        loaded_gs_type_ = this->get_parameter(stage_selection::kGaitSequencerTypeKey).as_string();
        // `fresh` now holds the previous instance and is destroyed here, outside
        // the lock, so no control loop waits on the old stage's destructor.
      } catch (const std::runtime_error &error) {
        // Unlike bring-up, a failed reload must not take down a walking robot:
        // keep the running gait sequencer and report loudly. StageLoadError and
        // StageInitError both derive from std::runtime_error.
        RCLCPP_ERROR(this->get_logger(),
                     "Could not reload the gait sequencer, keeping the running one: %s",
                     error.what());
      }
    }
  });

  // The mpc_state_weights_* asserts and the two Eigen copies they guarded went
  // with the MPC construction they fed: the MPC stage checks the vector lengths
  // itself and refuses to initialise, which reports the same mistake as a named
  // fatal error rather than as an assert that Release compiles out.
  while (!first_quad_state_received_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         2000,
                         "Waiting for first quad state message to properly initialize controller");
    rclcpp::spin_some(this->get_node_base_interface());
  }
  RCLCPP_INFO(this->get_logger(), "First quad_state received, initializing controller");

  // Pipeline stages, loaded through pluginlib (issue #9, M2.4).
  //
  // What used to stand here — a gait sequencer factory, an mpc_solver string
  // switch, a model adaptation switch, an SLC constructor and a WBC-constructing
  // generic lambda — is now five plugin loads. Each stock plugin's `Init` *is*
  // the factory body that lived here, moved behind the plugin boundary in M2.3
  // (src/plugins/*_plugins.cpp), so the pipeline is configured from exactly the
  // same parameters as before.
  //
  // Deliberately unguarded: `Load` throws StageLoadError (no such stage) or
  // StageInitError (the stage cannot configure itself), both fatal for bring-up
  // and both naming what went wrong — a loader error even lists the declared
  // alternatives (stage_loading.md §1). `main` reports the message and exits
  // non-zero. This replaces the previous mix of "log and rclcpp::shutdown()" and
  // "log and exit(-1)", and it is the one behaviour the modularity work insists
  // on: never start a quadruped with a stage nobody asked for.
  //
  // The order is the construction order of the code this replaces, and every
  // stage gets its own StageInit (each takes ownership of a model/state clone).
  gs_ = gs_loader_.Load(stage_selection::kGaitSequencerTypeKey, MakeStageInit());
  loaded_gs_type_ = this->get_parameter(stage_selection::kGaitSequencerTypeKey).as_string();
  mpc_ = mpc_loader_.Load(stage_selection::kMPCTypeKey, MakeStageInit());
  gs_->UpdateTarget(target_);
  // The model adaptation stage is always loaded, exactly as it was always
  // constructed; `use_model_adaptation_` still decides whether its loop does
  // anything.
  ma_ = ma_loader_.Load(stage_selection::kModelAdaptationTypeKey, MakeStageInit());
  slc_ = slc_loader_.Load(stage_selection::kSwingLegControllerTypeKey, MakeStageInit());
  wbc_ = wbc_loader_.Load(stage_selection::kWBCTypeKey, MakeStageInit());
  ValidateWBCCommandMode();
  // The contact stage is loaded last: it sits between the SLC and the WBC in the
  // control loop, and unlike the other five it replaces host code rather than a
  // host factory, so it has no construction order to preserve.
  contact_logic_ = contact_logic_loader_.Load(stage_selection::kContactLogicTypeKey, MakeStageInit());
  RCLCPP_INFO(this->get_logger(),
              "Pipeline stages loaded: gait sequencer [%s], mpc [%s], swing leg controller [%s], wbc [%s], "
              "model adaptation [%s], contact logic [%s]",
              this->get_parameter(stage_selection::kGaitSequencerTypeKey).as_string().c_str(),
              this->get_parameter(stage_selection::kMPCTypeKey).as_string().c_str(),
              this->get_parameter(stage_selection::kSwingLegControllerTypeKey).as_string().c_str(),
              this->get_parameter(stage_selection::kWBCTypeKey).as_string().c_str(),
              this->get_parameter(stage_selection::kModelAdaptationTypeKey).as_string().c_str(),
              this->get_parameter(stage_selection::kContactLogicTypeKey).as_string().c_str());

  // Who receives the model when the model adaptation changes it (issue #15,
  // M3.4). This block *is* the fan-out that used to be written out by hand in
  // ModelAdaptationCallback; registration order and lock grouping reproduce it
  // exactly — MPC and GS under mpc_lock_, WBC and the contact stage under
  // wbc_lock_, SLC under slc_lock_, in that order, and the names are the ones
  // the three log lines there already printed.
  //
  // GS is registered under mpc_lock_ rather than gait_sequencer_lock_ because
  // that is where it ran before, not because it belongs there: that is gap G6
  // (stage_contracts.md §7), which pipeline_host.md §8 keeps as a change of its
  // own so that this one stays reviewable as "same behaviour, different owner
  // of the list". Fixing it is now editing one argument on one line, here,
  // instead of surgery inside a callback.
  //
  // The model adaptation itself is the producer and is deliberately absent — it
  // is handed the model by DoModelAdaptation. Adding a stage to the pipeline
  // means adding one line here and nothing else; see
  // doc/modularity/model_update_broadcast.md.
  model_update_broadcast_.Register("MPC", mpc_lock_, mpc_);
  model_update_broadcast_.Register("GS", mpc_lock_, gs_);
  model_update_broadcast_.Register("WBC", wbc_lock_, wbc_);
  model_update_broadcast_.Register("contact logic", wbc_lock_, contact_logic_);
  model_update_broadcast_.Register("SLC", slc_lock_, slc_);

  // Now change modus of le driver acording to this controller
  // comment next paragraph if you want to use controller on bag data
  auto req = std::make_shared<interfaces::srv::ChangeLegDriverMode::Request>();
  req->target_mode = static_cast<int>(leg_control_mode_);
  RCLCPP_INFO(this->get_logger(), "Waiting for leg driver service to become available");
  change_leg_driver_mode_client_->wait_for_service();
  auto client_request = change_leg_driver_mode_client_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), client_request);
  if (client_request.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Starting controller");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Couldn't reach leg driver - undefined behaviour from now on");
  }

  rclcpp::SubscriptionOptions quad_control_target_subscription_opts;
  quad_control_target_subscription_opts.callback_group = mpc_call_back_group_;
  quad_control_target_subscription_ = this->create_subscription<interfaces::msg::QuadControlTarget>(
      "quad_control_target",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&MITController::QuadControlTargetUpdateCallback, this, std::placeholders::_1));

  mpc_loop_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::duration<double>(MPC_CONTROL_DT),
                                         std::bind(&MITController::MPCLoopCallback, this),
                                         mpc_call_back_group_);

  model_adaptation_loop_timer_ =
      rclcpp::create_timer(this,
                           this->get_clock(),
                           std::chrono::duration<double>(MODEL_ADAPTATION_DT),
                           std::bind(&MITController::ModelAdaptationCallback, this),
                           model_adaptation_callback_group_);  // TODO: at some point when MPC is adapting think
                                                               // if it has to be in the saem callback group
}

void MITController::QuadStateUpdateCallback(interfaces::msg::QuadState::SharedPtr quad_state_msg) {
  first_quad_state_received_ = true;
  quad_state_lock_.lock();
  quad_state_ = *quad_state_msg;
  quad_state_lock_.unlock();
}
StageInit MITController::MakeStageInit() {
  StageInit init;
  // Model and state clones the stage takes ownership of — the same constructor
  // injection the concrete stages always had (stage_contracts.md §3). Callers
  // reach this only after the first /quad_state has arrived, so both are valid
  // snapshots.
  init.model = std::make_unique<QuadModelPino>(quad_model_);
  quad_state_lock_.lock();
  init.state = std::make_unique<QuadState>(quad_state_);
  quad_state_lock_.unlock();

  // The node's whole parameter set, flattened to name -> value. Every stage gets
  // the same map and reads the keys it documents (plugin_lifecycle.md §3), which
  // is what keeps the host out of the business of knowing which parameter belongs
  // to which algorithm.
  const auto parameter_names =
      this->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names;
  for (const auto &name : parameter_names) {
    init.params.emplace(name, this->get_parameter(name).get_parameter_value());
  }
  return init;
}

// The `leg_control_mode` vocabulary `stage_selection::WBCCommandModeForLegControlMode` is written
// against. It takes the raw parameter value so it stays testable without a node, which only works
// while these agree.
static_assert(static_cast<int>(MITController::JOINT_CONTROL) == 0);
static_assert(static_cast<int>(MITController::JOINT_TORQUE_CONTROL) == 1);
static_assert(static_cast<int>(MITController::CARTESIAN_STIFFNESS_CONTROL) == 2);
static_assert(static_cast<int>(MITController::CARTESIAN_JOINT_CONTROL) == 3);

void MITController::ValidateWBCCommandMode() {
  const int64_t leg_control_mode = static_cast<int64_t>(leg_control_mode_);
  const std::optional<WBCCommandMode> required_mode =
      stage_selection::WBCCommandModeForLegControlMode(leg_control_mode);
  if (!required_mode.has_value()) {
    throw StageLoadError("leg_control_mode " + std::to_string(leg_control_mode)
                         + " is not a control mode; expected 0 (joint), 1 (joint torque), 2 (cartesian "
                           "stiffness) or 3 (cartesian joint)");
  }
  if (wbc_->SupportedCommandMode() == *required_mode) {
    return;
  }
  // Both are launch-time choices now (#13, M3.2), so nothing but this check stops them disagreeing.
  // Refuse to start rather than run: a WBC that cannot fill the command message this mode publishes
  // would leave the loop producing nothing every cycle, which on hardware is a robot that has been
  // commanded and is not being driven. Name both parameters and the fix, like StageLoader's errors
  // (stage_loading.md §1).
  const bool needs_cartesian = *required_mode == WBCCommandMode::kCartesian;
  throw StageLoadError(
      "stage '" + this->get_parameter(stage_selection::kWBCTypeKey).as_string() + "' (selected by '"
      + std::string(stage_selection::kWBCTypeKey) + "') produces "
      + (needs_cartesian ? "joint" : "cartesian") + " commands, but leg_control_mode "
      + std::to_string(leg_control_mode) + " needs " + (needs_cartesian ? "cartesian" : "joint")
      + " commands; either set '" + std::string(stage_selection::kWBCTypeKey) + "' to '"
      + stage_selection::WBCPluginsForCommandMode(*required_mode) + "' or change leg_control_mode");
}

void MITController::QuadControlTargetUpdateCallback(interfaces::msg::QuadControlTarget::SharedPtr quad_target_msg) {
  target_.hybrid_x_dot = quad_target_msg->body_x_dot;
  target_.hybrid_y_dot = quad_target_msg->body_y_dot;
  target_.z = quad_target_msg->world_z;
  target_.wz = quad_target_msg->hybrid_theta_dot;
  target_.roll = quad_target_msg->roll;
  target_.pitch = quad_target_msg->pitch;
  // TODO: this is maybe a race condition
}

void MITController::MPCLoopCallback() {
  // Get gait sequence and update other parts
  static GaitSequence gait_sequence_temp;
  static WrenchSequence ws_temp;
  static MPCPrediction mpc_prediction_temp;

  mpc_lock_.lock();
  gait_sequencer_lock_.lock();
  quad_state_lock_.lock();
  QuadState quad_state_temp = quad_state_;
  quad_state_lock_.unlock();
  gs_->UpdateTarget(target_);
  gs_->UpdateState(quad_state_temp);
  mpc_->UpdateState(quad_state_temp);
  slc_->UpdateState(quad_state_temp);
  gs_->GetGaitSequence(gait_sequence_temp);
  mpc_->UpdateGaitSequence(gait_sequence_temp);

  // The MPC now switches its own KEEP<->MOVE cost weights inside UpdateGaitSequence (issue #2, gap
  // G1), so the host no longer reaches into the concrete MPC here. It still mirrors the mode into
  // the heartbeat for diagnostics.
  if (PUBLISH_HEARTBEAT) {
    controller_heartbeat_.keep_pose_active = (gait_sequence_temp.sequence_mode == GaitSequence::KEEP);
  }

  static SolverInformation solver_info;
  static double mpc_solve_time = 0.0;
  // Calculate controls
  auto mpc_start_time = std::chrono::high_resolution_clock::now();
  mpc_->GetWrenchSequence(ws_temp, mpc_prediction_temp, solver_info);  // This one might block
  auto mpc_end_time = std::chrono::high_resolution_clock::now();

  RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), !solver_info.success, "MPC solver did not converge, code %d", solver_info.return_code);

  RCLCPP_WARN_EXPRESSION(this->get_logger(),
                         solver_info.total_solver_time >= MPC_CONTROL_DT,
                         "MPC solver took longer [%f s] than MPC control dt [%f s]",
                         solver_info.total_solver_time,
                         MPC_CONTROL_DT);

  RCLCPP_DEBUG(this->get_logger(),
               "MPC solver returned [%d] after [%f s], [%s]",
               solver_info.return_code,
               solver_info.total_solver_time,
               (solver_info.success ? "true" : "false"));

  gs_wrench_sequence_lock_.lock();
  wrench_sequence_ = ws_temp;
  gait_sequence_ = gait_sequence_temp;
  mpc_prediction_ = mpc_prediction_temp;
  gs_updated_ = true;
  gs_wrench_sequence_lock_.unlock();

  // Only if this ran once, we can start the slc thread:
  if (control_loop_timer_ == nullptr) {
    slc_loop_timer_ = rclcpp::create_timer(this,
                                           this->get_clock(),
                                           std::chrono::duration<double>(SWING_LEG_DT),
                                           std::bind(&MITController::SLCLoopCallback, this),
                                           slc_callback_group_);
    control_loop_timer_ = rclcpp::create_timer(this,
                                               this->get_clock(),
                                               std::chrono::duration<double>(CONTROL_DT),
                                               std::bind(&MITController::ControlLoopCallback, this),
                                               control_loop_call_back_group_);
  }
  gait_sequencer_lock_.unlock();
  mpc_lock_.unlock();

  mpc_solve_time = std::chrono::duration_cast<std::chrono::duration<double>>(mpc_end_time - mpc_start_time).count();

  // publish solve time
  if (PUBLISH_SOLVE_TIME) {
    interfaces::msg::MPCDiagnostics solve_time_message;
    solve_time_message.header.stamp = this->get_clock()->now();
    solve_time_message.solve_time = mpc_solve_time;
    solve_time_message.acados_solve_qp_time = solver_info.acados_solve_QP_time;
    solve_time_message.acados_condensing_time = solver_info.acados_condensing_time;
    solve_time_message.acados_interface_time = solver_info.acados_interface_time;
    solve_time_message.acados_total_time = solver_info.total_solver_time;
    solve_time_message.acados_num_iter = solver_info.acados_num_iter;
    solve_time_message.acados_t_computed = solver_info.acados_t_computed;
    solve_time_message.qp_objective_value = solver_info.qp_objective_value;
    solve_time_message.qp_residuals[0] = solver_info.qp_residuals[0];
    solve_time_message.qp_residuals[1] = solver_info.qp_residuals[1];
    solve_time_message.qp_residuals[2] = solver_info.qp_residuals[2];
    solve_time_message.qp_residuals[3] = solver_info.qp_residuals[3];
    solve_time_publisher_->publish(solve_time_message);
  }

  if (PUBLISH_HEARTBEAT) {
    if (!solver_info.success) {
      controller_heartbeat_.num_mpc_solver_fail++;
    }
    if (solver_info.total_solver_time >= MPC_CONTROL_DT) {
      controller_heartbeat_.num_mpc_solver_overtime++;
    }
  }

#ifdef DEBUG_PRINTS
  RCLCPP_DEBUG(this->get_logger(),
               "Contacts are \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[0][0],
               gait_sequence_.contact_sequence[0][1],
               gait_sequence_.contact_sequence[0][2],
               gait_sequence_.contact_sequence[0][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is  \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[1][0],
               gait_sequence_.contact_sequence[1][1],
               gait_sequence_.contact_sequence[1][2],
               gait_sequence_.contact_sequence[1][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[2][0],
               gait_sequence_.contact_sequence[2][1],
               gait_sequence_.contact_sequence[2][2],
               gait_sequence_.contact_sequence[2][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[3][0],
               gait_sequence_.contact_sequence[3][1],
               gait_sequence_.contact_sequence[3][2],
               gait_sequence_.contact_sequence[3][3]);

  RCLCPP_DEBUG(this->get_logger(),
               "Forces are    \n\t\t [(%f,%f,%f), \t(%f,%f,%f), \n\t\t "
               "(%f,%f,%f),  \t(%f,%f,%f)]",
               ws_temp.forces[0][0].x(),
               ws_temp.forces[0][0].y(),
               ws_temp.forces[0][0].z(),
               ws_temp.forces[0][1].x(),
               ws_temp.forces[0][1].y(),
               ws_temp.forces[0][1].z(),
               ws_temp.forces[0][2].x(),
               ws_temp.forces[0][2].y(),
               ws_temp.forces[0][2].z(),
               ws_temp.forces[0][3].x(),
               ws_temp.forces[0][3].y(),
               ws_temp.forces[0][3].z());
#endif
  if (gait_state_publisher_ != nullptr) {
    static interfaces::msg::GaitState gait_state_msg;
    gs_->GetGaitState(gait_state_msg);
    gait_state_publisher_->publish(gait_state_msg);
  }

  if (open_loop_publisher_ != nullptr) {
    assert(MPC_PREDICTION_HORIZON == 10);  // fixed message size
    static interfaces::msg::PositionSequence open_loop_positions;
    for (unsigned int ol_idx = 0; ol_idx < MPC_PREDICTION_HORIZON; ol_idx++) {
      open_loop_positions.valid[ol_idx] = true;
      open_loop_positions.x[ol_idx] = mpc_prediction_.position[ol_idx].x();
      open_loop_positions.y[ol_idx] = mpc_prediction_.position[ol_idx].y();
      open_loop_positions.z[ol_idx] = mpc_prediction_.position[ol_idx].z();
      open_loop_positions.qw[ol_idx] = mpc_prediction_.orientation[ol_idx].w();
      open_loop_positions.qx[ol_idx] = mpc_prediction_.orientation[ol_idx].x();
      open_loop_positions.qy[ol_idx] = mpc_prediction_.orientation[ol_idx].y();
      open_loop_positions.qz[ol_idx] = mpc_prediction_.orientation[ol_idx].z();
    }
    open_loop_publisher_->publish(open_loop_positions);
  }

  if (PUBLISH_GAIT_SEQUENCE && (gait_sequence_publisher_ != nullptr)) {
    auto gs_msg = gait_sequence_to_msg(gait_sequence_);
    gait_sequence_publisher_->publish(gs_msg);
  }
}

void MITController::ModelAdaptationCallback() {
  if (use_model_adaptation_) {
    interfaces::msg::QuadModelDebug quad_model_debug_msg;
    static QuadState quad_state_temp;
    static GaitSequence gs_temp;
    quad_state_lock_.lock();
    quad_state_temp = quad_state_;
    quad_state_lock_.unlock();
    ma_->UpdateState(quad_state_temp);
    gs_wrench_sequence_lock_.lock();
    gs_temp = gait_sequence_;
    gs_wrench_sequence_lock_.unlock();
    ma_->UpdateGaitSequence(gs_temp);
    bool changed_model = ma_->DoModelAdaptation(quad_model_);
    Eigen::Map<Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS>>(
        quad_model_debug_msg.parameter_vector.data()) = ma_->GetParameterVector();
    Eigen::Map<Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS>>(quad_model_debug_msg.p_matrix.data()) =
        ma_->GetParameterCovariance().diagonal();
    Eigen::Map<Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS>>(quad_model_debug_msg.update_change.data()) =
        ma_->GetDelta();
    Eigen::Map<Eigen::Vector<double, 6>>(quad_model_debug_msg.total_force_torque.data()) = ma_->GetTotalForceTorque();
    Eigen::Map<Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS>>(quad_model_debug_msg.sv.data()) =
        ma_->GetSV();
    quad_model_debug_publisher_->publish(quad_model_debug_msg);
    if (changed_model) {
      RCLCPP_INFO(this->get_logger(), "Model Adaptation changed QuadModel");
      interfaces::msg::QuadModel quad_model_msg;
      // The five UpdateModel calls, three lock/unlock pairs and three log lines
      // that stood here are now the registration block in the constructor
      // (issue #15, M3.4): same stages, same order, same locks, same log text —
      // the difference is that a sixth stage no longer has to be threaded into
      // this callback by hand.
      model_update_broadcast_.Broadcast(quad_model_, this->get_logger());
      quad_model_msg.header.stamp = this->get_clock()->now();
      quad_model_msg.mass = quad_model_.GetMass();
      Eigen::Map<Eigen::Vector3d>(quad_model_msg.com.data()) = quad_model_.GetBodyToCOM().vector();
      Eigen::Map<Eigen::Matrix3d>(quad_model_msg.inertia.data()) = quad_model_.GetInertia();
      quad_model_publisher_->publish(quad_model_msg);
      RCLCPP_INFO(this->get_logger(), "Published Model update to other components");
      if (PUBLISH_HEARTBEAT) {
        controller_heartbeat_.num_model_updates++;
      }
    }
  }
}

void MITController::ControlLoopCallback() {
  // Quad state does not have to be locked, as it is running in a different
  // callback group This only makes sense to run, if the MPC and so on was
  // running at least once
  static WrenchSequence wrench_sequence_temp;
  static GaitSequence gait_sequence_temp;
  static FeetTargets feet_targets_temp;
  static QuadState quad_state_temp;
  static MPCPrediction mpc_prediction_temp;
  static std::array<double, ModelInterface::N_LEGS> feet_swing_progress_temp;
  static std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_temp;

  gs_wrench_sequence_lock_.lock();
  wrench_sequence_temp = wrench_sequence_;
  gait_sequence_temp = gait_sequence_;
  mpc_prediction_temp = mpc_prediction_;
  gs_wrench_sequence_lock_.unlock();
  targets_lock_.lock();
  feet_targets_temp = feet_targets_;
  feet_swing_progress_temp = feet_swing_progress_;
  feet_swing_states_temp = feet_swing_states_;
  targets_lock_.unlock();
  quad_state_lock_.lock();
  quad_state_temp = quad_state_;
  quad_state_lock_.unlock();

  wbc_lock_.lock();

  static double wbc_solve_time = 0.0;  // TODO: not static?
  auto wbc_start_time = std::chrono::high_resolution_clock::now();
  // Update WBC
  wbc_->UpdateState(quad_state_temp);
  //  wbc_->UpdateTarget(gait_sequence_temp.target_orientation,
  //                     gait_sequence_temp.target_position,
  //                     gait_sequence_temp.target_velocity,
  //                     gait_sequence_temp.target_twist);  // TODO: take MPC first prediction here?`

  wbc_->UpdateTarget(mpc_prediction_temp.orientation[1],
                     mpc_prediction_temp.position[1],
                     mpc_prediction_temp.linear_velocity[1],
                     mpc_prediction_temp.angular_velocity[1]);  // TODO: take MPC first prediction here?

  // Contact reconciliation (issue #12, M3.1). The per-leg early/late/lost contact
  // FSM that used to be two switch statements right here is the sixth pipeline
  // stage now; the host feeds it, calls it once, and reacts to what it reports.
  contact_logic_->UpdateState(quad_state_temp);
  contact_logic_->UpdateGaitSequence(gait_sequence_temp);
  contact_logic_->UpdateWrenchSequence(wrench_sequence_temp);
  contact_logic_->UpdateSwingLegState(feet_targets_temp, feet_swing_progress_temp, feet_swing_states_temp);

  // Prepare for WBC
  auto wrenches = wrench_sequence_temp.forces[0];
  auto feet_targets = feet_targets_temp;
  auto gait = gait_sequence_temp.contact_sequence[0];

  // Reconcile the plan with what the feet actually sense. In/out: the three
  // objects seeded above are the stage's inputs and, on return, the WBC's.
  // Exactly once per cycle — this is the only stage method that mutates state.
  contact_logic_->Reconcile(gait, wrenches, feet_targets);

  // The logging and the heartbeat counter stayed here: they are host concerns,
  // and keeping them out of the stage is what lets it be a plain algorithm with
  // no logger and no message dependency. The messages and their severities are
  // the ones the inline code emitted, on the same cycles.
  static ContactLogicInterface::ContactEvents contact_events;
  contact_logic_->GetContactEvents(contact_events);
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    if (contact_events.early_contact_detected[leg_idx]) {
      RCLCPP_INFO(this->get_logger(), "Foot [%d] has early contact", leg_idx);
      if (PUBLISH_HEARTBEAT) {
        controller_heartbeat_.num_early_contacts++;
      }
    }
    if (contact_events.late_contact_detected[leg_idx]) {
      RCLCPP_INFO(this->get_logger(), "Foot [%d] has late contact", leg_idx);
    }
    if (contact_events.lost_contact_detected[leg_idx]) {
      RCLCPP_WARN(this->get_logger(),
                  "Foot [%d] lost contact and is kept a last contact position relative to body",
                  leg_idx);
    }
    // The event covers both SLC states that mean "not swinging yet"; these two
    // messages only ever fired for NOT_STARTED, so the host still gates on it.
    if (contact_events.swing_scheduled_before_slc_started[leg_idx]) {
      RCLCPP_ERROR_EXPRESSION(this->get_logger(),
                              feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::NOT_STARTED,
                              "Controll loop reached Swing phase for foot [%d], but SLC is still in stance",
                              leg_idx);
      RCLCPP_WARN_EXPRESSION(this->get_logger(),
                             feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::NOT_STARTED,
                             "Controll loop reached Swing phase for foot [%d], but SLC has not yet started",
                             leg_idx);
    }
  }

  // Set PD weights:
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    if (gait[leg_idx]) {  // Stance
      switch (leg_control_mode_) {
        case CARTESIAN_JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_joint_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_joint_control_stance_Kd_;
          break;
        case CARTESIAN_STIFFNESS_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_stiffness_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_stiffness_control_stance_Kd_;
          break;
        case JOINT_TORQUE_CONTROL:
          [[fallthrough]];
        case JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kp.data() + leg_idx * 3) = joint_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kd.data() + leg_idx * 3) = joint_control_stance_Kd_;
          break;
      }
    } else {
      switch (leg_control_mode_) {  // Swing
        case CARTESIAN_JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_joint_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_joint_control_swing_Kd_;
          break;
        case CARTESIAN_STIFFNESS_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_stiffness_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_stiffness_control_swing_Kd_;
          break;
        case JOINT_TORQUE_CONTROL:
          [[fallthrough]];
        case JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kp.data() + leg_idx * 3) = joint_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kd.data() + leg_idx * 3) = joint_control_swing_Kd_;
          break;
      }
    }
  }

  wbc_->UpdateFeetTarget(feet_targets);
  wbc_->UpdateFootContact(gait);
  wbc_->UpdateWrenches(wrenches);

  // Which getter serves this cycle follows leg_control_mode_ alone. Before #13 (M3.2) the WBC's
  // command type was a compile-time property (a std::conditional typedef over USE_WBC), so this had
  // to be an `if constexpr` inside a generic lambda to keep the incompatible branch from being
  // compiled at all — and a leg_control_mode_ the build could not serve was a per-cycle logged
  // error. Both are gone: the interface carries both getters, and the loaded plugin's
  // SupportedCommandMode() is checked against leg_control_mode_ once at bring-up, so reaching this
  // switch at all means the pairing is already known good. The cost is unchanged — one virtual call
  // through the same stage pointer, into the same solver.
  static WBCReturn wbc_return;
  switch (leg_control_mode_) {
    case CARTESIAN_JOINT_CONTROL:
      [[fallthrough]];
    case CARTESIAN_STIFFNESS_CONTROL: {
      leg_cmd_.header.stamp = this->get_clock()->now();
      CartesianCommands commands;
      wbc_return = wbc_->GetCartesianCommand(commands);
      assign(commands.position, leg_cmd_.ee_pos);
      assign(commands.velocity, leg_cmd_.ee_vel);
      assign(commands.force, leg_cmd_.ee_force);
      leg_cmd_publisher_->publish(leg_cmd_);
      break;
    }
    case JOINT_TORQUE_CONTROL:
      [[fallthrough]];
    case JOINT_CONTROL: {
      leg_joint_cmd_.header.stamp = this->get_clock()->now();
      JointTorqueVelocityPositionCommands commands;
      wbc_return = wbc_->GetJointCommand(commands);
      assign(commands.velocity, leg_joint_cmd_.velocity);
      assign(commands.position, leg_joint_cmd_.position);
      assign(commands.torque, leg_joint_cmd_.effort);
      leg_joint_cmd_publisher_->publish(leg_joint_cmd_);
      break;
    }
  }
  auto wbc_end_time = std::chrono::high_resolution_clock::now();
  wbc_solve_time = std::chrono::duration_cast<std::chrono::duration<double>>(wbc_end_time - wbc_start_time).count();
  wbc_lock_.unlock();
  if (PUBLISH_WBC_SOLVE_TIME) {
    interfaces::msg::WBCReturn wbc_solve_time_message;
    wbc_solve_time_message.total_time = wbc_solve_time;
    wbc_solve_time_message.success = wbc_return.success;
    wbc_solve_time_message.qp_solve_time = wbc_return.qp_solve_time;
    wbc_solve_time_message.qp_update_time = wbc_return.qp_update_time;
    wbc_solve_time_message.header.stamp = this->get_clock()->now();
    wbc_solve_time_publisher_->publish(wbc_solve_time_message);
  }

  RCLCPP_ERROR_EXPRESSION(this->get_logger(), !wbc_return.success, "WBC solver did not converge");

  //  RCLCPP_WARN_EXPRESSION(this->get_logger(),
  //                         wbc_solve_time >= WBC_CYCLE_DT,
  //                         "WBC solver took longer [%f s] than MPC control dt [%f s]",
  //                         wbc_solve_time,
  //                         WBC_CYCLE_DT); //TODO: maybe enable!

  if (swing_leg_trajs_publisher_ != nullptr) {
    static interfaces::msg::VectorSequence swing_leg_traj;  // Only the pending and runnign (4 in total)
    std::array<Eigen::Vector3d, N_LEGS> start, end;
    slc_->GetCurrentTrajs(start, end);
    for (unsigned int foot_idx = 0; foot_idx < N_LEGS; foot_idx++) {
      Eigen::Vector3d vector_start_to_end = end[foot_idx] - start[foot_idx];
      swing_leg_traj.origin_x[foot_idx] = start[foot_idx].x();
      swing_leg_traj.origin_y[foot_idx] = start[foot_idx].y();
      swing_leg_traj.origin_z[foot_idx] = start[foot_idx].z();
      swing_leg_traj.vector_x[foot_idx] = vector_start_to_end.x();
      swing_leg_traj.vector_y[foot_idx] = vector_start_to_end.y();
      swing_leg_traj.vector_z[foot_idx] = vector_start_to_end.z();
    }
    swing_leg_trajs_publisher_->publish(swing_leg_traj);
  }

  if (PUBLISH_WBC_TARGET) {
    static interfaces::msg::WBCTarget wbc_target_msg_;
    wbc_target_msg_.header.stamp = this->get_clock()->now();
    eigenToRosMsg(mpc_prediction_temp.position[1], wbc_target_msg_.pose.position);
    eigenToRosMsg(mpc_prediction_temp.orientation[1], wbc_target_msg_.pose.orientation);
    eigenToRosMsg(mpc_prediction_temp.linear_velocity[1], wbc_target_msg_.twist.linear);
    eigenToRosMsg(mpc_prediction_temp.angular_velocity[1], wbc_target_msg_.twist.angular);
    wbc_target_msg_.feet_contacts = gait;
    eigenToRosMsg(wrenches, wbc_target_msg_.feet_wrenches);
    eigenToRosMsg(feet_targets.positions, wbc_target_msg_.feet_pos_targets);
    eigenToRosMsg(feet_targets.velocities, wbc_target_msg_.feet_vel_targets);
    eigenToRosMsg(feet_targets.accelerations, wbc_target_msg_.feet_acc_targets);
    wbc_target_publisher_->publish(wbc_target_msg_);
  }

  if (PUBLISH_HEARTBEAT) {
    if (wbc_solve_time >= WBC_CYCLE_DT) {
      controller_heartbeat_.num_wbc_overtime++;
    }
    if (!wbc_return.success) {
      controller_heartbeat_.num_wbc_solver_fail++;
    }
  }
}

void MITController::SLCLoopCallback() {
  static QuadState quad_state_temp;
  static GaitSequence gs_tmp;
  static std::array<double, ModelInterface::N_LEGS> feet_swing_progress_temp;
  static std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_temp;
  static FeetTargets feet_targets_temp;
  slc_lock_.lock();
  gs_wrench_sequence_lock_.lock();
  if (gs_updated_) {
    gs_tmp = gait_sequence_;
    gs_wrench_sequence_lock_.unlock();
    slc_->UpdateGaitSequence(gs_tmp);
  } else {
    gs_wrench_sequence_lock_.unlock();
  }

  quad_state_lock_.lock();
  quad_state_temp = quad_state_;
  quad_state_lock_.unlock();

  slc_->UpdateState(quad_state_temp);
  slc_->GetFeetTargets(feet_targets_temp);
  slc_->GetProgress(feet_swing_progress_temp, feet_swing_states_temp);

  targets_lock_.lock();
  feet_targets_ = feet_targets_temp;
  feet_swing_progress_ = feet_swing_progress_temp;
  feet_swing_states_ = feet_swing_states_temp;
  targets_lock_.unlock();
  slc_lock_.unlock();
}

void MITController::HartbeatCallback() {
  controller_heartbeat_.header.stamp = this->get_clock()->now();
  controller_heartbeat_publisher_->publish(controller_heartbeat_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MITController> node;
  // Stage selection and stage initialisation fail by throwing (StageLoadError /
  // StageInitError, stage_loading.md §2). Both are fatal — there is no fallback
  // stage — so the only thing left to do is put the message where a human bringing
  // the robot up will read it, and exit non-zero instead of unwinding through an
  // uncaught exception whose message the terminate handler mangles.
  try {
    node = std::make_shared<MITController>("mit_controller_node");
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("mit_controller_node"), "Could not start the controller: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::ExecutorOptions exopt;
  rclcpp::executors::MultiThreadedExecutor executor(exopt, 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
