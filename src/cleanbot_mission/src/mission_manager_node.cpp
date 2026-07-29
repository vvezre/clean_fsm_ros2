/*
 * 文件作用：执行自动清扫任务段，并协调转向、RTK 直线跟踪、滚刷和安全停车。
 *
 * 输入：ExecuteCleaning/NavigateWaypoints Action、暂停服务、RTK、下位机状态、命令执行状态和最终命令。
 * 输出：跟踪目标、任务车辆命令、滚刷命令、安全命令和 Action 反馈/结果。
 *
 * 关键约束：
 * 1. 本节点不访问串口，所有车辆命令必须经过 command_arbiter_node。
 * 2. 转向完成必须同时匹配 source 和 request_id。
 * 3. 直行完成必须匹配 tracking generation，旧任务回调不能推进新任务。
 * 4. 暂停、手动接管和 RTK 恢复后都从当前段开头重新执行。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_interfaces/action/execute_cleaning.hpp"
#include "cleanbot_interfaces/action/recover_mission.hpp"
#include "cleanbot_interfaces/action/navigate_waypoints.hpp"
#include "cleanbot_interfaces/action/return_home.hpp"
#include "cleanbot_interfaces/msg/brush_command.hpp"
#include "cleanbot_interfaces/msg/command_execution_status.hpp"
#include "cleanbot_interfaces/msg/hardware_status.hpp"
#include "cleanbot_interfaces/msg/rtk_fix.hpp"
#include "cleanbot_interfaces/msg/tracking_status.hpp"
#include "cleanbot_interfaces/msg/tracking_target.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "cleanbot_interfaces/msg/vehicle_state.hpp"
#include "cleanbot_interfaces/srv/set_mission_pause.hpp"
#include "cleanbot_mission/mission_checkpoint.hpp"
#include "cleanbot_mission/mission_state_machine.hpp"
#include "cleanbot_mission/straight_edge_guard.hpp"
#include "cleanbot_mission/vehicle_state_builder.hpp"
#include "cleanbot_mission/waypoint_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace cleanbot {
namespace mission {

class MissionManagerNode : public rclcpp::Node {
 public:
  using ExecuteCleaning = cleanbot_interfaces::action::ExecuteCleaning;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCleaning>;
  using RecoverMission = cleanbot_interfaces::action::RecoverMission;
  using RecoverGoalHandle = rclcpp_action::ServerGoalHandle<RecoverMission>;
  using NavigateWaypoints = cleanbot_interfaces::action::NavigateWaypoints;
  using WaypointGoalHandle =
      rclcpp_action::ServerGoalHandle<NavigateWaypoints>;
  using ReturnHome = cleanbot_interfaces::action::ReturnHome;
  using ReturnHomeGoalHandle = rclcpp_action::ServerGoalHandle<ReturnHome>;
  using VehicleCommand = cleanbot_interfaces::msg::VehicleCommand;
  using TrackingStatus = cleanbot_interfaces::msg::TrackingStatus;
  using CommandExecutionStatus =
      cleanbot_interfaces::msg::CommandExecutionStatus;

  MissionManagerNode() : Node("mission_manager_node") {
    tracking_target_publisher_ =
        create_publisher<cleanbot_interfaces::msg::TrackingTarget>(
        "/tracking/target", common::latest_command_qos());
    mission_command_publisher_ = create_publisher<VehicleCommand>(
        "/control/mission_cmd", common::latest_command_qos());
    brush_publisher_ = create_publisher<cleanbot_interfaces::msg::BrushCommand>(
        "/control/brush_cmd", common::latest_command_qos());
    safety_command_publisher_ = create_publisher<VehicleCommand>(
        "/control/safety_cmd", common::latest_command_qos());
    vehicle_state_publisher_ =
        create_publisher<cleanbot_interfaces::msg::VehicleState>(
        "/vehicle/state", common::latched_status_qos());
    vehicle_state_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MissionManagerNode::publishVehicleState, this));

    tracking_status_subscription_ = create_subscription<TrackingStatus>(
        "/tracking/status",
        common::status_qos(),
        std::bind(&MissionManagerNode::onTrackingStatus, this, std::placeholders::_1));
    rtk_subscription_ = create_subscription<cleanbot_interfaces::msg::RtkFix>(
        "/rtk/fix",
        common::rtk_fix_qos(),
        std::bind(&MissionManagerNode::onRtkFix, this, std::placeholders::_1));
    hardware_subscription_ =
        create_subscription<cleanbot_interfaces::msg::HardwareStatus>(
        "/hardware/status",
        common::hardware_status_qos(),
        std::bind(&MissionManagerNode::onHardwareStatus, this, std::placeholders::_1));
    command_status_subscription_ = create_subscription<CommandExecutionStatus>(
        "/hardware/command_status",
        common::command_status_qos(),
        std::bind(&MissionManagerNode::onCommandStatus, this, std::placeholders::_1));
    final_command_subscription_ = create_subscription<VehicleCommand>(
        "/control/final_cmd",
        common::latest_command_qos(),
        std::bind(&MissionManagerNode::onFinalCommand, this, std::placeholders::_1));

    pause_service_ =
        create_service<cleanbot_interfaces::srv::SetMissionPause>(
        "/mission/set_pause",
        std::bind(
            &MissionManagerNode::onSetPause,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<ExecuteCleaning>(
        this,
        "/mission/execute_cleaning",
        std::bind(
            &MissionManagerNode::onGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &MissionManagerNode::onCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &MissionManagerNode::onAccepted,
            this,
            std::placeholders::_1));

    waypoint_action_server_ = rclcpp_action::create_server<NavigateWaypoints>(
        this,
        "/mission/navigate_waypoints",
        std::bind(
            &MissionManagerNode::onWaypointGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &MissionManagerNode::onWaypointCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &MissionManagerNode::onWaypointAccepted,
            this,
            std::placeholders::_1));

    return_home_action_server_ = rclcpp_action::create_server<ReturnHome>(
        this,
        "/mission/return_home",
        std::bind(
            &MissionManagerNode::onReturnHomeGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &MissionManagerNode::onReturnHomeCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &MissionManagerNode::onReturnHomeAccepted,
            this,
            std::placeholders::_1));

    recover_action_server_ = rclcpp_action::create_server<RecoverMission>(
        this,
        "/mission/recover",
        std::bind(
            &MissionManagerNode::onRecoverGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &MissionManagerNode::onRecoverCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &MissionManagerNode::onRecoverAccepted,
            this,
            std::placeholders::_1));

    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "mission.rtk_recovery_timeout_sec",
            "mission.command_completion_timeout_sec",
            "mission.turn_heartbeat_ms",
            "mission.turn_z_speed",
            "mission.turn_angle_tolerance_deg",
            "mission.low_battery_threshold_percent",
            "mission.rtk_freshness_timeout_sec",
            "mission.hardware_freshness_timeout_sec",
            "mission.edge_debounce_ms",
            "mission.edge_target_tolerance_m",
            "mission.checkpoint_path",
        },
        false,
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });

    RCLCPP_INFO(get_logger(), "mission manager waiting for configuration");
  }

  ~MissionManagerNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutting_down_ = true;
      condition_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  enum class StepResult {
    kSuccess,
    kRestart,
    kCancelled,
    kFailed,
  };

  rclcpp_action::GoalResponse onGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const ExecuteCleaning::Goal> goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      RCLCPP_WARN(get_logger(), "CONFIG_NOT_READY: cleaning goal rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mission_active_ || goal_reserved_ || goal->segments.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(
      const std::shared_ptr<GoalHandle>) {
    condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    if (worker_.joinable()) {
      worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_reserved_ = false;
      mission_active_ = true;
      preflight_complete_ = false;
      pause_requested_ = false;
      edge_confirmation_required_ = false;
      pause_reason_.clear();
      fault_code_.clear();
      fault_message_.clear();
      expected_turn_request_id_ = 0u;
      expected_tracking_generation_ = 0u;
      turn_status_received_ = false;
      tracking_status_received_ = false;
      current_action_ = "cleaning";
      lifecycle_state_ = "ACCEPTED";
      lifecycle_message_ = "cleaning mission accepted";
      state_current_segment_ = 0u;
      state_total_segments_ = static_cast<std::uint32_t>(
          goal_handle->get_goal()->segments.size());
    }
    worker_ = std::thread(&MissionManagerNode::execute, this, goal_handle);
  }

  rclcpp_action::GoalResponse onWaypointGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const NavigateWaypoints::Goal> goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      RCLCPP_WARN(get_logger(), "CONFIG_NOT_READY: waypoint goal rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mission_active_ || goal_reserved_ || goal->waypoints.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    WaypointSequence sequence(goal->waypoints.size(), goal->loop, goal->loop_count);
    if (!sequence.valid()) {
      RCLCPP_WARN(
          get_logger(), "invalid waypoint loop: %s", sequence.error_code().c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto& waypoint : goal->waypoints) {
      if (!validate_waypoint(waypoint.lat, waypoint.lon)) {
        RCLCPP_WARN(
            get_logger(), "invalid waypoint coordinate id=%s", waypoint.id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onWaypointCancel(
      const std::shared_ptr<WaypointGoalHandle>) {
    condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onWaypointAccepted(
      const std::shared_ptr<WaypointGoalHandle> goal_handle) {
    if (worker_.joinable()) {
      worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_reserved_ = false;
      mission_active_ = true;
      preflight_complete_ = false;
      pause_requested_ = false;
      edge_confirmation_required_ = false;
      pause_reason_.clear();
      fault_code_.clear();
      fault_message_.clear();
      expected_turn_request_id_ = 0u;
      expected_tracking_generation_ = 0u;
      turn_status_received_ = false;
      tracking_status_received_ = false;
      waypoint_current_index_ = 0u;
      waypoint_total_ = static_cast<std::uint32_t>(
          goal_handle->get_goal()->waypoints.size());
      waypoint_completed_ = 0u;
      waypoint_total_targets_ = 0u;
      waypoint_current_loop_ = 0u;
      waypoint_target_lat_ = 0.0;
      waypoint_target_lon_ = 0.0;
      current_action_ = "navigate_waypoints";
      lifecycle_state_ = "ACCEPTED";
      lifecycle_message_ = "waypoint mission accepted";
      state_current_segment_ = 0u;
      state_total_segments_ = waypoint_total_;
    }
    worker_ = std::thread(
        &MissionManagerNode::executeWaypoints, this, goal_handle);
  }

  rclcpp_action::GoalResponse onReturnHomeGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const ReturnHome::Goal> goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      RCLCPP_WARN(get_logger(), "CONFIG_NOT_READY: return-home goal rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mission_active_ || goal_reserved_ ||
        !validate_waypoint(goal->target_lat, goal->target_lon)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onReturnHomeCancel(
      const std::shared_ptr<ReturnHomeGoalHandle>) {
    condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onReturnHomeAccepted(
      const std::shared_ptr<ReturnHomeGoalHandle> goal_handle) {
    if (worker_.joinable()) {
      worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_reserved_ = false;
      mission_active_ = true;
      preflight_complete_ = false;
      pause_requested_ = false;
      edge_confirmation_required_ = false;
      pause_reason_.clear();
      fault_code_.clear();
      fault_message_.clear();
      expected_turn_request_id_ = 0u;
      expected_tracking_generation_ = 0u;
      turn_status_received_ = false;
      tracking_status_received_ = false;
      current_action_ = "return_home";
      lifecycle_state_ = "ACCEPTED";
      lifecycle_message_ = "return-home mission accepted";
      state_current_segment_ = 0u;
      state_total_segments_ = 1u;
    }
    worker_ = std::thread(
        &MissionManagerNode::executeReturnHome, this, goal_handle);
  }

  rclcpp_action::GoalResponse onRecoverGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const RecoverMission::Goal> goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_ || !cleaning_checkpoint_store_ ||
        !return_home_checkpoint_store_) {
      RCLCPP_WARN(get_logger(), "CONFIG_NOT_READY: recovery goal rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mission_active_ || goal_reserved_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::string error;
    const auto checkpoint = loadRecoveryCheckpoint(goal->run_id, &error);
    if (!checkpoint.has_value()) {
      RCLCPP_WARN(
          get_logger(),
          "NO_RECOVERABLE_MISSION: %s",
          error.empty() ? "checkpoint is missing" : error.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->run_id.empty() && goal->run_id != checkpoint->run_id) {
      RCLCPP_WARN(
          get_logger(),
          "RECOVERY_RUN_ID_MISMATCH: requested=%s saved=%s",
          goal->run_id.c_str(),
          checkpoint->run_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onRecoverCancel(
      const std::shared_ptr<RecoverGoalHandle>) {
    condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onRecoverAccepted(
      const std::shared_ptr<RecoverGoalHandle> goal_handle) {
    if (worker_.joinable()) {
      worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_reserved_ = false;
      mission_active_ = true;
      preflight_complete_ = false;
      pause_requested_ = false;
      edge_confirmation_required_ = false;
      pause_reason_.clear();
      fault_code_.clear();
      fault_message_.clear();
      expected_turn_request_id_ = 0u;
      expected_tracking_generation_ = 0u;
      turn_status_received_ = false;
      tracking_status_received_ = false;
      current_action_ = "recover";
      lifecycle_state_ = "ACCEPTED";
      lifecycle_message_ = "mission recovery accepted";
      state_current_segment_ = 0u;
      state_total_segments_ = 0u;
    }
    worker_ = std::thread(
        &MissionManagerNode::executeRecoverMission, this, goal_handle);
  }

  void onSetPause(
      const std::shared_ptr<cleanbot_interfaces::srv::SetMissionPause::Request> request,
      std::shared_ptr<cleanbot_interfaces::srv::SetMissionPause::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      response->success = false;
      response->code = "CONFIG_NOT_READY";
      response->message = "任务配置尚未就绪";
      return;
    }
    if (!mission_active_) {
      response->success = false;
      response->code = "NO_ACTIVE_MISSION";
      response->message = "当前没有正在执行的任务";
      return;
    }
    if (!request->pause && edge_confirmation_required_) {
      pause_requested_ = false;
      edge_confirmation_required_ = false;
      pause_reason_.clear();
      response->success = true;
      response->code = "EDGE_CONTINUE_CONFIRMED";
      response->message = "已确认继续，重新执行当前直行段";
      condition_.notify_all();
      return;
    }

    pause_requested_ = request->pause;
    pause_reason_ = request->pause ? "用户暂停任务" : "";
    response->success = true;
    response->code = request->pause ? "MISSION_PAUSE_REQUESTED" : "MISSION_RESUME_REQUESTED";
    response->message = request->pause ? "任务正在暂停" : "任务正在恢复";
    condition_.notify_all();
  }

  void onRtkFix(const cleanbot_interfaces::msg::RtkFix::SharedPtr fix) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_rtk_ = *fix;
    has_rtk_ = true;
    latest_rtk_at_ = std::chrono::steady_clock::now();
    condition_.notify_all();
  }

  void onHardwareStatus(
      const cleanbot_interfaces::msg::HardwareStatus::SharedPtr hardware) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_hardware_ = *hardware;
    has_hardware_ = true;
    latest_hardware_at_ = std::chrono::steady_clock::now();
    // 预检阶段允许串口从“未连接”过渡到“就绪”，只有任务正式开始后才锁定为运行故障。
    if (mission_active_ && preflight_complete_) {
      if (!hardware->connected) {
        setFaultLocked("HARDWARE_DISCONNECTED", "下位机连接已断开");
      } else if (isLowBattery(*hardware)) {
        setFaultLocked("LOW_BATTERY", "电量低于任务安全阈值");
      }
    }
    condition_.notify_all();
  }

  void onTrackingStatus(const TrackingStatus::SharedPtr tracking) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mission_active_ &&
        tracking->generation == expected_tracking_generation_) {
      latest_tracking_status_ = *tracking;
      tracking_status_received_ = true;
      condition_.notify_all();
    }
  }

  void onCommandStatus(const CommandExecutionStatus::SharedPtr status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mission_active_ && status->request_id == expected_turn_request_id_ &&
        status->source == "mission_turn") {
      latest_turn_status_ = *status;
      turn_status_received_ = true;
      condition_.notify_all();
    }
  }

  void onFinalCommand(const VehicleCommand::SharedPtr command) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_final_command_ = *command;
    has_final_command_ = true;
    if (!mission_active_ || !command->active) {
      return;
    }
    if (command->priority == VehicleCommand::PRIORITY_MANUAL) {
      pause_requested_ = true;
      pause_reason_ = "手动控制已接管车辆，任务保持暂停";
      condition_.notify_all();
    } else if (command->priority == VehicleCommand::PRIORITY_EMERGENCY ||
        command->source == "software_emergency_stop") {
      pause_requested_ = true;
      pause_reason_ = "软件急停已触发，等待用户明确恢复任务";
      condition_.notify_all();
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = *goal_handle->get_goal();
    auto result = std::make_shared<ExecuteCleaning::Result>();
    MissionStateMachine machine;
    std::uint32_t completed_segments = 0u;
    std::uint32_t current_loop = 0u;

    publishSafetyRelease();
    const auto preflight = waitForPreflight(goal_handle);
    if (preflight != StepResult::kSuccess) {
      finishAction(goal_handle, machine, result, preflight, completed_segments);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preflight_complete_ = true;
    }

    const bool continuous_loop = goal.loop && goal.loop_count == 0u;
    const std::uint32_t requested_loops = goal.loop
        ? std::max<std::uint32_t>(1u, goal.loop_count)
        : 1u;
    MissionCheckpointRecord checkpoint;
    checkpoint.run_id = makeRunId("cleaning");
    checkpoint.mission_kind = "cleaning";
    checkpoint.task_id = goal.task_id;
    checkpoint.model_id = goal.model_id;
    checkpoint.model_version = goal.model_version;
    checkpoint.plan_id = goal.plan_id;
    checkpoint.plan_hash = goal.plan_hash;
    checkpoint.loop = goal.loop;
    checkpoint.loop_count = goal.loop_count;
    checkpoint.current_loop = current_loop;
     // total_loops=0 is the persisted representation of an unbounded loop.
     checkpoint.total_loops = continuous_loop ? 0u : requested_loops;
    checkpoint.next_segment = 0u;
    checkpoint.total_segments = static_cast<std::uint32_t>(goal.segments.size());
    checkpoint.completed_segments = completed_segments;
    checkpoint.brush_speed = goal.brush_speed;
    checkpoint.segments = goal.segments;
    if (!saveCheckpoint(checkpoint, "MISSION_PREPARING")) {
      setFault("CHECKPOINT_SAVE_FAILED", "无法保存清扫任务断点，拒绝启动车辆");
      finishAction(goal_handle, machine, result, StepResult::kFailed, completed_segments);
      return;
    }
    bool mission_finished = false;
    StepResult final_step = StepResult::kSuccess;

    while (rclcpp::ok() && !mission_finished) {
      if (!machine.start(goal.segments.size())) {
        setFault("MISSION_STATE_ERROR", "任务状态机无法启动");
        final_step = StepResult::kFailed;
        break;
      }
      publishFeedback(
          goal_handle, machine, current_loop, requested_loops, "PREPARING", "准备执行任务");

      while (!machine.terminal()) {
        const auto operational = ensureOperational(
            goal_handle, machine, current_loop, requested_loops);
        if (operational == StepResult::kRestart) {
          continue;
        }
        if (operational != StepResult::kSuccess) {
          final_step = operational;
          break;
        }

        const auto segment_index = machine.current_segment();
        if (segment_index >= goal.segments.size()) {
          setFault("MISSION_SEGMENT_INDEX_INVALID", "任务段索引超出范围");
          machine.fail();
          final_step = StepResult::kFailed;
          break;
        }
        const auto& segment = goal.segments[segment_index];
        publishBrushForSegment(segment, goal.brush_speed);

        if (std::abs(segment.turn_angle_deg) > turn_angle_tolerance_deg_) {
          const auto turn = executeTurn(
              goal_handle, machine, segment, current_loop, requested_loops);
          if (turn == StepResult::kRestart) {
            continue;
          }
          if (turn != StepResult::kSuccess) {
            final_step = turn;
            break;
          }
        }

        const auto tracking = executeTracking(
            goal_handle,
            machine,
            segment,
            goal.brush_speed,
            current_loop,
            requested_loops);
        if (tracking == StepResult::kRestart) {
          continue;
        }
        if (tracking != StepResult::kSuccess) {
          final_step = tracking;
          break;
        }
        ++completed_segments;
        checkpoint.current_loop = current_loop;
        checkpoint.next_segment = static_cast<std::uint32_t>(machine.current_segment());
        checkpoint.completed_segments = completed_segments;
        if (!saveCheckpoint(
                checkpoint,
                machine.terminal() ? "MISSION_LOOP_COMPLETED" : "MISSION_SEGMENT_COMPLETED")) {
          setFault("CHECKPOINT_SAVE_FAILED", "无法保存清扫进度，任务已安全停止");
          final_step = StepResult::kFailed;
          break;
        }
        publishFeedback(
            goal_handle,
            machine,
            current_loop,
            requested_loops,
            machine.terminal() ? "LOOP_COMPLETED" : "SEGMENT_COMPLETED",
            "当前任务段已完成");
      }

      if (final_step != StepResult::kSuccess || !machine.terminal() ||
          machine.state() != MissionState::kCompleted) {
        break;
      }
      ++current_loop;
      mission_finished = !continuous_loop && current_loop >= requested_loops;
      if (!mission_finished) {
        checkpoint.current_loop = current_loop;
        checkpoint.next_segment = 0u;
        if (!saveCheckpoint(checkpoint, "MISSION_NEXT_LOOP")) {
          setFault("CHECKPOINT_SAVE_FAILED", "无法保存清扫循环进度，任务已安全停止");
          final_step = StepResult::kFailed;
          break;
        }
      }
    }

    finishAction(goal_handle, machine, result, final_step, completed_segments);
  }

  void executeWaypoints(
      const std::shared_ptr<WaypointGoalHandle> goal_handle) {
    const auto goal = *goal_handle->get_goal();
    auto result = std::make_shared<NavigateWaypoints::Result>();
    WaypointSequence sequence(goal.waypoints.size(), goal.loop, goal.loop_count);
    std::uint32_t completed_waypoints = 0u;
    StepResult final_step = StepResult::kSuccess;

    const std::uint64_t finite_target_count = goal.loop
        ? (goal.loop_count == 0u
            ? 0u
            : 1u + static_cast<std::uint64_t>(goal.loop_count) *
                static_cast<std::uint64_t>(goal.waypoints.size()))
        : static_cast<std::uint64_t>(goal.waypoints.size());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      waypoint_total_targets_ = finite_target_count;
    }

    publishSafetyRelease();
    const auto preflight = waitForPreflight(goal_handle);
    if (preflight != StepResult::kSuccess) {
      finishWaypointAction(
          goal_handle, result, preflight, completed_waypoints);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preflight_complete_ = true;
    }

    WaypointTarget target;
    while (rclcpp::ok() && sequence.next(target)) {
      const auto& waypoint = goal.waypoints[target.waypoint_index];
      {
        std::lock_guard<std::mutex> lock(mutex_);
        waypoint_current_index_ = static_cast<std::uint32_t>(
            target.waypoint_index);
        waypoint_current_loop_ = target.completed_loop;
        waypoint_target_lat_ = waypoint.lat;
        waypoint_target_lon_ = waypoint.lon;
      }

      bool target_complete = false;
      while (rclcpp::ok() && !target_complete) {
        MissionStateMachine machine;
        if (!machine.start(1u)) {
          setFault("MISSION_STATE_ERROR", "多路点状态机无法启动");
          final_step = StepResult::kFailed;
          break;
        }

        const std::uint32_t total_loops = goal.loop ? goal.loop_count : 1u;
        const auto operational = ensureOperational(
            goal_handle, machine, target.completed_loop, total_loops);
        if (operational == StepResult::kRestart) {
          continue;
        }
        if (operational != StepResult::kSuccess) {
          final_step = operational;
          break;
        }

        cleanbot_interfaces::msg::RtkFix current_fix;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!rtkReadyLocked()) {
            setFaultLocked("RTK_NOT_READY", "无法读取当前车辆中心坐标");
            final_step = StepResult::kFailed;
            break;
          }
          current_fix = latest_rtk_;
        }

        const auto plan = build_waypoint_segment(
            current_fix.lat,
            current_fix.lon,
            current_fix.heading_deg,
            waypoint.lat,
            waypoint.lon);
        cleanbot_interfaces::msg::TaskSegment segment;
        segment.index = static_cast<std::uint32_t>(target.waypoint_index);
        segment.id = waypoint.id;
        segment.segment_type =
            cleanbot_interfaces::msg::TaskSegment::SEGMENT_TRANSFER;
        segment.start_lat = plan.start_lat;
        segment.start_lon = plan.start_lon;
        segment.end_lat = plan.end_lat;
        segment.end_lon = plan.end_lon;
        segment.heading_deg = plan.heading_deg;
        segment.turn_angle_deg = plan.turn_angle_deg;
        segment.speed = 0;
        segment.mode = 0u;

        publishWaypointFeedback(goal_handle, "PREPARING");
        if (std::abs(segment.turn_angle_deg) > turn_angle_tolerance_deg_) {
          const auto turn = executeTurn(
              goal_handle,
              machine,
              segment,
              target.completed_loop,
              total_loops);
          if (turn == StepResult::kRestart) {
            continue;
          }
          if (turn != StepResult::kSuccess) {
            final_step = turn;
            break;
          }
        }

        const auto tracking = executeTracking(
            goal_handle,
            machine,
            segment,
            0,
            target.completed_loop,
            total_loops,
            false);
        if (tracking == StepResult::kRestart) {
          continue;
        }
        if (tracking != StepResult::kSuccess) {
          final_step = tracking;
          break;
        }

        if (completed_waypoints < std::numeric_limits<std::uint32_t>::max()) {
          ++completed_waypoints;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          waypoint_completed_ = completed_waypoints;
        }
        publishWaypointFeedback(goal_handle, "WAYPOINT_REACHED");
        target_complete = true;
      }

      if (final_step != StepResult::kSuccess) {
        break;
      }
    }

    if (!rclcpp::ok() && final_step == StepResult::kSuccess) {
      final_step = StepResult::kCancelled;
    }

    finishWaypointAction(
        goal_handle, result, final_step, completed_waypoints);
  }

  template <typename GoalHandleT>
  StepResult repositionToPoint(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      const double target_lat,
      const double target_lon,
      const std::string& target_id,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops) {
    // Recovery must first move from the reboot position to the unfinished
    // segment start. This transfer never enables the cleaning brush.
    while (rclcpp::ok()) {
      MissionStateMachine transfer_machine;
      if (!transfer_machine.start(1u)) {
        return StepResult::kFailed;
      }
      const auto operational = ensureOperational(
          goal_handle, transfer_machine, current_loop, total_loops);
      if (operational == StepResult::kRestart) {
        continue;
      }
      if (operational != StepResult::kSuccess) {
        return operational;
      }

      cleanbot_interfaces::msg::RtkFix current_fix;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!rtkReadyLocked()) {
          setFaultLocked("RTK_NOT_READY", "reposition requires a fresh RTK fix");
          return StepResult::kFailed;
        }
        current_fix = latest_rtk_;
      }
      const auto plan = build_waypoint_segment(
          current_fix.lat,
          current_fix.lon,
          current_fix.heading_deg,
          target_lat,
          target_lon);
      cleanbot_interfaces::msg::TaskSegment transfer;
      transfer.index = 0u;
      transfer.id = target_id;
      transfer.segment_type =
          cleanbot_interfaces::msg::TaskSegment::SEGMENT_TRANSFER;
      transfer.start_lat = plan.start_lat;
      transfer.start_lon = plan.start_lon;
      transfer.end_lat = plan.end_lat;
      transfer.end_lon = plan.end_lon;
      transfer.heading_deg = plan.heading_deg;
      transfer.turn_angle_deg = plan.turn_angle_deg;
      transfer.speed = 0;
      transfer.mode = 0u;

      publishBrush(false, 0, false);
      publishFeedback(
          goal_handle,
          transfer_machine,
          current_loop,
          total_loops,
          "RECOVERY_REPOSITIONING",
          "moving to the unfinished segment start");
      if (std::abs(transfer.turn_angle_deg) > turn_angle_tolerance_deg_) {
        const auto turn = executeTurn(
            goal_handle, transfer_machine, transfer, current_loop, total_loops);
        if (turn == StepResult::kRestart) {
          continue;
        }
        if (turn != StepResult::kSuccess) {
          return turn;
        }
      }
      const auto tracking = executeTracking(
          goal_handle,
          transfer_machine,
          transfer,
          0,
          current_loop,
          total_loops,
          false);
      if (tracking == StepResult::kRestart) {
        continue;
      }
      return tracking;
    }
    return StepResult::kCancelled;
  }

  void executeReturnHome(
      const std::shared_ptr<ReturnHomeGoalHandle> goal_handle) {
    const auto goal = *goal_handle->get_goal();
    auto result = std::make_shared<ReturnHome::Result>();
    MissionStateMachine final_machine;
    StepResult final_step = StepResult::kSuccess;
    MissionCheckpointRecord checkpoint;
    checkpoint.run_id = makeRunId("return-home");
    checkpoint.mission_kind = "return_home";
    checkpoint.target_lat = goal.target_lat;
    checkpoint.target_lon = goal.target_lon;
    checkpoint.target_valid = true;
    checkpoint.dock_after_arrival = goal.dock_after_arrival;
    checkpoint.next_segment = 0u;
    checkpoint.total_segments = 1u;
    checkpoint.total_loops = 1u;

    publishSafetyRelease();
    publishBrush(false, 0, false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preflight_complete_ = false;
    }

    // A restart means RTK recovered or the current pose changed. Rebuild the
    // return segment from the latest vehicle pose instead of reusing stale geometry.
    while (rclcpp::ok() && final_step == StepResult::kSuccess) {
      const auto preflight = waitForPreflight(goal_handle);
      if (preflight != StepResult::kSuccess) {
        final_step = preflight;
        break;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        preflight_complete_ = true;
      }

      cleanbot_interfaces::msg::RtkFix current_fix;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!rtkReadyLocked()) {
          setFaultLocked("RTK_NOT_READY", "return-home requires fresh RTK fix");
          final_step = StepResult::kFailed;
          break;
        }
        current_fix = latest_rtk_;
      }
      const auto plan = build_waypoint_segment(
          current_fix.lat,
          current_fix.lon,
          current_fix.heading_deg,
          goal.target_lat,
          goal.target_lon);
      cleanbot_interfaces::msg::TaskSegment segment;
      segment.index = 0u;
      segment.id = "return-home";
      segment.segment_type =
          cleanbot_interfaces::msg::TaskSegment::SEGMENT_RETURN;
      segment.start_lat = plan.start_lat;
      segment.start_lon = plan.start_lon;
      segment.end_lat = plan.end_lat;
      segment.end_lon = plan.end_lon;
      segment.heading_deg = plan.heading_deg;
      segment.turn_angle_deg = plan.turn_angle_deg;
      segment.speed = 0;
      segment.mode = 0u;
      checkpoint.segments.clear();
      checkpoint.segments.push_back(segment);
      if (!saveCheckpoint(checkpoint, "RETURN_HOME_PREPARING")) {
        setFault("CHECKPOINT_SAVE_FAILED", "无法保存返航断点，拒绝启动车辆");
        final_step = StepResult::kFailed;
        break;
      }

      MissionStateMachine attempt_machine;
      if (!attempt_machine.start(1u)) {
        setFault("MISSION_STATE_ERROR", "return-home state machine cannot start");
        final_step = StepResult::kFailed;
        break;
      }
      publishFeedback(
          goal_handle, attempt_machine, 0u, 1u, "PREPARING", "return-home preparing");
      if (std::abs(segment.turn_angle_deg) > turn_angle_tolerance_deg_) {
        const auto turn = executeTurn(goal_handle, attempt_machine, segment, 0u, 1u);
        if (turn == StepResult::kRestart) {
          continue;
        }
        if (turn != StepResult::kSuccess) {
          final_step = turn;
          break;
        }
      }
      const auto tracking = executeTracking(
          goal_handle, attempt_machine, segment, 0, 0u, 1u, false);
      if (tracking == StepResult::kRestart) {
        continue;
      }
      if (tracking != StepResult::kSuccess) {
        final_step = tracking;
        break;
      }
      if (attempt_machine.state() == MissionState::kCompleted) {
        checkpoint.next_segment = 1u;
        checkpoint.completed_segments = 1u;
        if (!saveCheckpoint(checkpoint, "RETURN_HOME_COMPLETED")) {
          setFault("CHECKPOINT_SAVE_FAILED", "无法保存返航完成状态");
          final_step = StepResult::kFailed;
          break;
        }
        final_machine = std::move(attempt_machine);
      }
      break;
    }
    if (!rclcpp::ok() && final_step == StepResult::kSuccess) {
      final_step = StepResult::kCancelled;
    }
    finishReturnHomeAction(goal_handle, final_machine, result, final_step);
  }

  void executeRecoverMission(
      const std::shared_ptr<RecoverGoalHandle> goal_handle) {
    auto result = std::make_shared<RecoverMission::Result>();
    MissionStateMachine machine;
    std::string error;
    std::optional<MissionCheckpointRecord> checkpoint;
    if (cleaning_checkpoint_store_ && return_home_checkpoint_store_) {
      checkpoint = loadRecoveryCheckpoint(goal_handle->get_goal()->run_id, &error);
    }
    if (!checkpoint.has_value()) {
      setFault(
          "NO_RECOVERABLE_MISSION",
          error.empty() ? "no recoverable mission checkpoint" : error);
      finishRecoverAction(goal_handle, machine, result, StepResult::kFailed, "");
      return;
    }
    const auto goal = goal_handle->get_goal();
    if (!goal->run_id.empty() && goal->run_id != checkpoint->run_id) {
      setFault(
          "RECOVERY_RUN_ID_MISMATCH",
          "requested run_id does not match saved checkpoint");
      finishRecoverAction(goal_handle, machine, result, StepResult::kFailed, "");
      return;
    }
    if (checkpoint->mission_kind != "cleaning" &&
        checkpoint->mission_kind != "return_home") {
      setFault(
          "RECOVERY_KIND_UNSUPPORTED",
          "only cleaning and return-home checkpoints are currently recoverable");
      finishRecoverAction(goal_handle, machine, result, StepResult::kFailed, "");
      return;
    }
    executeRecoveredSegments(goal_handle, std::move(*checkpoint), result);
  }

  void executeRecoveredSegments(
      const std::shared_ptr<RecoverGoalHandle> goal_handle,
      MissionCheckpointRecord checkpoint,
      const std::shared_ptr<RecoverMission::Result>& result) {
    MissionStateMachine machine;
    StepResult final_step = StepResult::kSuccess;
    std::uint32_t completed_segments = checkpoint.completed_segments;

    if (checkpoint.segments.empty()) {
      setFault("RECOVERY_SEGMENTS_EMPTY", "checkpoint has no executable segments");
      finishRecoverAction(goal_handle, machine, result, StepResult::kFailed, "");
      return;
    }

    publishSafetyRelease();
    const auto preflight = waitForPreflight(goal_handle);
    if (preflight != StepResult::kSuccess) {
      finishRecoverAction(
          goal_handle, machine, result, preflight, checkpoint.mission_kind);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preflight_complete_ = true;
    }

    const bool continuous_loop = checkpoint.loop && checkpoint.loop_count == 0u;
    const std::uint32_t requested_loops = checkpoint.total_loops > 0u
        ? checkpoint.total_loops
        : (checkpoint.loop ? std::max<std::uint32_t>(1u, checkpoint.loop_count) : 1u);
    std::uint32_t current_loop = checkpoint.current_loop;
    std::uint32_t start_segment = checkpoint.next_segment;
    bool mission_finished = false;
    bool needs_initial_reposition = checkpoint.mission_kind == "cleaning";
    bool replan_first_recovered_segment = needs_initial_reposition;

    if (start_segment >= checkpoint.segments.size()) {
      ++current_loop;
      start_segment = 0u;
    }
    if (!continuous_loop && current_loop >= requested_loops) {
      clearCheckpoint(checkpoint.mission_kind, "recover_already_complete");
      result->success = true;
      result->code = "MISSION_ALREADY_COMPLETE";
      result->message = "checkpoint has no remaining segment";
      goal_handle->succeed(result);
      resetRuntimeAfterWorker(result->code, result->message);
      return;
    }

    while (rclcpp::ok() && !mission_finished) {
      if (needs_initial_reposition) {
        const auto& unfinished = checkpoint.segments[start_segment];
        const auto reposition = repositionToPoint(
            goal_handle,
            unfinished.start_lat,
            unfinished.start_lon,
            unfinished.id + "-reposition",
            current_loop,
            requested_loops);
        if (reposition != StepResult::kSuccess) {
          final_step = reposition;
          break;
        }
        needs_initial_reposition = false;
      }
      if (!machine.start(checkpoint.segments.size(), start_segment)) {
        setFault("MISSION_STATE_ERROR", "recovered state machine cannot start");
        final_step = StepResult::kFailed;
        break;
      }
      publishFeedback(
          goal_handle,
          machine,
          current_loop,
          requested_loops,
          "RECOVERY_PREPARING",
          "recovered mission preparing");

      while (!machine.terminal()) {
        const auto operational = ensureOperational(
            goal_handle, machine, current_loop, requested_loops);
        if (operational == StepResult::kRestart) {
          continue;
        }
        if (operational != StepResult::kSuccess) {
          final_step = operational;
          break;
        }

        const auto segment_index = machine.current_segment();
        if (segment_index >= checkpoint.segments.size()) {
          setFault("MISSION_SEGMENT_INDEX_INVALID", "recovered segment index is invalid");
          machine.fail();
          final_step = StepResult::kFailed;
          break;
        }
        auto segment = checkpoint.segments[segment_index];
        const bool replan_from_current_pose =
            checkpoint.mission_kind == "return_home" ||
            (replan_first_recovered_segment && segment_index == start_segment);
        if (replan_from_current_pose) {
          cleanbot_interfaces::msg::RtkFix current_fix;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!rtkReadyLocked()) {
              setFaultLocked("RTK_NOT_READY", "recovery requires a fresh RTK fix");
              final_step = StepResult::kFailed;
            } else {
              current_fix = latest_rtk_;
            }
          }
          if (final_step != StepResult::kSuccess) {
            break;
          }
          const auto plan = build_waypoint_segment(
              current_fix.lat,
              current_fix.lon,
              current_fix.heading_deg,
              checkpoint.mission_kind == "return_home"
                  ? checkpoint.target_lat : segment.end_lat,
              checkpoint.mission_kind == "return_home"
                  ? checkpoint.target_lon : segment.end_lon);
          segment.start_lat = plan.start_lat;
          segment.start_lon = plan.start_lon;
          segment.end_lat = plan.end_lat;
          segment.end_lon = plan.end_lon;
          segment.heading_deg = plan.heading_deg;
          segment.turn_angle_deg = plan.turn_angle_deg;
        }
        if (checkpoint.mission_kind == "cleaning") {
          publishBrushForSegment(segment, checkpoint.brush_speed);
        } else {
          publishBrush(false, 0, false);
        }

        if (std::abs(segment.turn_angle_deg) > turn_angle_tolerance_deg_) {
          const auto turn = executeTurn(
              goal_handle, machine, segment, current_loop, requested_loops);
          if (turn == StepResult::kRestart) {
            continue;
          }
          if (turn != StepResult::kSuccess) {
            final_step = turn;
            break;
          }
        }

        const auto tracking = executeTracking(
            goal_handle,
            machine,
            segment,
            checkpoint.brush_speed,
            current_loop,
            requested_loops,
            checkpoint.mission_kind == "cleaning");
        if (tracking == StepResult::kRestart) {
          continue;
        }
        if (tracking != StepResult::kSuccess) {
          final_step = tracking;
          break;
        }
        if (replan_first_recovered_segment && segment_index == start_segment) {
          replan_first_recovered_segment = false;
        }
        if (completed_segments < std::numeric_limits<std::uint32_t>::max()) {
          ++completed_segments;
        }
        checkpoint.completed_segments = completed_segments;
        checkpoint.current_loop = current_loop;
        checkpoint.next_segment = static_cast<std::uint32_t>(machine.current_segment());
        if (!saveCheckpoint(
                checkpoint,
                machine.terminal() ? "RECOVERY_LOOP_COMPLETED" : "RECOVERY_SEGMENT_COMPLETED")) {
          setFault("CHECKPOINT_SAVE_FAILED", "无法保存恢复任务进度，任务已安全停止");
          final_step = StepResult::kFailed;
          break;
        }
        publishFeedback(
            goal_handle,
            machine,
            current_loop,
            requested_loops,
            machine.terminal() ? "LOOP_COMPLETED" : "SEGMENT_COMPLETED",
            "recovered segment completed");
      }

      if (final_step != StepResult::kSuccess || !machine.terminal() ||
          machine.state() != MissionState::kCompleted) {
        break;
      }
      ++current_loop;
      start_segment = 0u;
      mission_finished = !continuous_loop && current_loop >= requested_loops;
      if (!mission_finished) {
        checkpoint.current_loop = current_loop;
        checkpoint.next_segment = 0u;
        if (!saveCheckpoint(checkpoint, "RECOVERY_NEXT_LOOP")) {
          setFault("CHECKPOINT_SAVE_FAILED", "无法保存恢复循环进度，任务已安全停止");
          final_step = StepResult::kFailed;
          break;
        }
      }
    }

    finishRecoverAction(
        goal_handle, machine, result, final_step, checkpoint.mission_kind);
  }

  template <typename GoalHandleT>
  StepResult waitForPreflight(
      const std::shared_ptr<GoalHandleT>& goal_handle) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutting_down_ && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return StepResult::kCancelled;
      }
      if (!fault_code_.empty()) {
        return StepResult::kFailed;
      }
      if (rtkReadyLocked() && hardwareReadyLocked()) {
        return StepResult::kSuccess;
      }
      if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
        if (!rtkReadyLocked()) {
          setFaultLocked("RTK_NOT_READY", "RTK 固定解、中心坐标或航向尚未就绪");
        } else {
          setFaultLocked("HARDWARE_NOT_READY", "下位机状态尚未就绪");
        }
        return StepResult::kFailed;
      }
    }
    return StepResult::kCancelled;
  }

  template <typename GoalHandleT>
  StepResult ensureOperational(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops) {
    bool paused = false;
    bool rtk_lost = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutting_down_ || goal_handle->is_canceling()) {
        machine.cancel();
        return StepResult::kCancelled;
      }
      if (!fault_code_.empty() || !hardwareReadyLocked()) {
        if (fault_code_.empty()) {
          setFaultLocked("HARDWARE_NOT_READY", "下位机状态失效或状态数据超时");
        }
        machine.fail();
        return StepResult::kFailed;
      }
      paused = pause_requested_;
      rtk_lost = !rtkReadyLocked();
    }
    if (paused) {
      return waitWhilePaused(goal_handle, machine, current_loop, total_loops);
    }
    if (rtk_lost) {
      return waitForRtkRecovery(goal_handle, machine, current_loop, total_loops);
    }
    return StepResult::kSuccess;
  }

  template <typename GoalHandleT>
  StepResult waitWhilePaused(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops) {
    machine.pause();
    publishStoppedOutputs("mission_paused", false);
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reason = pause_reason_;
    }
    publishFeedback(goal_handle, machine, current_loop, total_loops, "PAUSED", reason);

    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &goal_handle]() {
      return shutting_down_ || !rclcpp::ok() || goal_handle->is_canceling() ||
          !pause_requested_ || !fault_code_.empty();
    });
    if (shutting_down_ || !rclcpp::ok() || goal_handle->is_canceling()) {
      lock.unlock();
      machine.cancel();
      return StepResult::kCancelled;
    }
    if (!fault_code_.empty()) {
      lock.unlock();
      machine.fail();
      return StepResult::kFailed;
    }
    lock.unlock();
    machine.resume();
    publishFeedback(
        goal_handle, machine, current_loop, total_loops, "RESUMING", "从当前段重新执行");
    return StepResult::kRestart;
  }

  template <typename GoalHandleT>
  StepResult waitForEdgeConfirmation(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops,
      const double distance_to_target_m,
      const double signed_remaining_m) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pause_requested_ = true;
      edge_confirmation_required_ = true;
      pause_reason_ = "直行过程中提前触边，等待人工确认是否继续";
    }

    machine.pause();
    publishStoppedOutputs("edge_confirm_required", false);
    RCLCPP_WARN(
        get_logger(),
        "edge confirmation required: distance=%.3f m, signed_remaining=%.3f m",
        distance_to_target_m,
        signed_remaining_m);
    publishFeedback(
        goal_handle,
        machine,
        current_loop,
        total_loops,
        "EDGE_CONFIRM_REQUIRED",
        "未到目标点发生持续触边，车辆已停车并关闭滚刷，请人工确认是否继续");

    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &goal_handle]() {
      return shutting_down_ || !rclcpp::ok() || goal_handle->is_canceling() ||
          !pause_requested_ || !fault_code_.empty();
    });
    if (shutting_down_ || !rclcpp::ok() || goal_handle->is_canceling()) {
      edge_confirmation_required_ = false;
      lock.unlock();
      machine.cancel();
      return StepResult::kCancelled;
    }
    if (!fault_code_.empty()) {
      edge_confirmation_required_ = false;
      lock.unlock();
      machine.fail();
      return StepResult::kFailed;
    }
    edge_confirmation_required_ = false;
    lock.unlock();

    machine.resume();
    publishFeedback(
        goal_handle,
        machine,
        current_loop,
        total_loops,
        "RESUMING",
        "人工已确认继续，从当前直行段重新执行");
    return StepResult::kRestart;
  }

  template <typename GoalHandleT>
  StepResult waitForRtkRecovery(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops) {
    machine.begin_rtk_recovery();
    publishStoppedOutputs("rtk_recovering", true);
    publishFeedback(
        goal_handle,
        machine,
        current_loop,
        total_loops,
        "RTK_RECOVERING",
        "RTK 固定解丢失，车辆已停车等待恢复");

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(rtk_recovery_timeout_sec_));
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutting_down_ && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        lock.unlock();
        machine.cancel();
        return StepResult::kCancelled;
      }
      if (!fault_code_.empty()) {
        lock.unlock();
        machine.fail();
        return StepResult::kFailed;
      }
      if (pause_requested_) {
        lock.unlock();
        publishSafetyRelease();
        return waitWhilePaused(goal_handle, machine, current_loop, total_loops);
      }
      if (rtkReadyLocked()) {
        lock.unlock();
        machine.recover_rtk();
        publishSafetyRelease();
        publishFeedback(
            goal_handle,
            machine,
            current_loop,
            total_loops,
            "RTK_RECOVERED",
            "RTK 已恢复，从当前段重新执行");
        return StepResult::kRestart;
      }
      if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
        setFaultLocked("RTK_RECOVERY_TIMEOUT", "RTK 在规定时间内未恢复固定解");
        lock.unlock();
        machine.fail();
        return StepResult::kFailed;
      }
    }
    machine.cancel();
    return StepResult::kCancelled;
  }

  template <typename GoalHandleT>
  StepResult executeTurn(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const cleanbot_interfaces::msg::TaskSegment& segment,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops) {
    const std::uint64_t request_id = nextRequestId();
    machine.begin_turn(request_id);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expected_turn_request_id_ = request_id;
      turn_status_received_ = false;
    }
    publishTurnCommand(segment, request_id, true);
    publishFeedback(
        goal_handle, machine, current_loop, total_loops, "TURNING", "正在执行当前段转向");

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(command_completion_timeout_sec_));
    auto next_heartbeat = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(turn_heartbeat_ms_);

    while (rclcpp::ok()) {
      const auto operational = ensureOperational(
          goal_handle, machine, current_loop, total_loops);
      if (operational != StepResult::kSuccess) {
        clearExpectedTurn();
        return operational;
      }

      CommandExecutionStatus status;
      bool received = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_until(lock, std::min(deadline, next_heartbeat), [this]() {
          return shutting_down_ || turn_status_received_ || pause_requested_ ||
              !fault_code_.empty() || !rtkReadyLocked();
        });
        if (turn_status_received_) {
          status = latest_turn_status_;
          turn_status_received_ = false;
          received = true;
        }
      }
      if (received) {
        if (status.state == CommandExecutionStatus::STATE_COMPLETED &&
            (status.completion_event == 0x02u || status.completion_event == 0x03u)) {
          machine.complete_turn(request_id);
          clearExpectedTurn();
          return StepResult::kSuccess;
        }
        if (status.state == CommandExecutionStatus::STATE_REJECTED ||
            status.state == CommandExecutionStatus::STATE_TIMED_OUT ||
            status.state == CommandExecutionStatus::STATE_TRANSPORT_LOST) {
          setFault(
              status.state == CommandExecutionStatus::STATE_REJECTED
                  ? "TURN_COMMAND_REJECTED"
                  : (status.state == CommandExecutionStatus::STATE_TRANSPORT_LOST
                      ? "TURN_COMMAND_TRANSPORT_LOST"
                      : "TURN_COMMAND_TIMEOUT"),
              status.detail);
          machine.fail();
          clearExpectedTurn();
          return StepResult::kFailed;
        }
      }
      const auto now_at = std::chrono::steady_clock::now();
      if (now_at >= deadline) {
        setFault("TURN_COMPLETION_TIMEOUT", "等待下位机上报转向完成超时");
        machine.fail();
        clearExpectedTurn();
        return StepResult::kFailed;
      }
      if (now_at >= next_heartbeat) {
        // 心跳内容必须与首条有限转向命令一致，只刷新任务租约，不触发再次下发。
        publishTurnCommand(segment, request_id, true);
        next_heartbeat = now_at + std::chrono::milliseconds(turn_heartbeat_ms_);
      }
    }
    machine.cancel();
    clearExpectedTurn();
    return StepResult::kCancelled;
  }

  template <typename GoalHandleT>
  StepResult executeTracking(
      const std::shared_ptr<GoalHandleT>& goal_handle,
      MissionStateMachine& machine,
      const cleanbot_interfaces::msg::TaskSegment& segment,
      const std::int32_t brush_speed,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops,
      const bool manage_brush = true) {
    bool restore_brush = false;
    while (rclcpp::ok()) {
      if (restore_brush && manage_brush) {
        publishBrushForSegment(segment, brush_speed);
      }

      const std::uint64_t generation = nextTrackingGeneration();
      if (!machine.begin_tracking(generation)) {
        setFault("MISSION_STATE_ERROR", "任务状态机无法开始当前直行段");
        machine.fail();
        return StepResult::kFailed;
      }
      straight_edge_guard_->reset();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        expected_tracking_generation_ = generation;
        tracking_status_received_ = false;
      }
      publishTrackingTarget(segment, generation, true);
      publishFeedback(
          goal_handle,
          machine,
          current_loop,
          total_loops,
          "TRACKING",
          restore_brush ? "继续执行当前直线路径段" : "正在执行当前直线路径段");

      bool restart_tracking = false;
      while (rclcpp::ok()) {
        const auto operational = ensureOperational(
            goal_handle, machine, current_loop, total_loops);
        if (operational == StepResult::kRestart) {
          clearExpectedTracking();
          if (!manage_brush) {
            return StepResult::kRestart;
          }
          restart_tracking = true;
          break;
        }
        if (operational != StepResult::kSuccess) {
          clearExpectedTracking();
          return operational;
        }

        TrackingStatus tracking;
        bool received = false;
        bool edge_clear = true;
        double distance_to_target_m = std::numeric_limits<double>::quiet_NaN();
        double signed_remaining_m = std::numeric_limits<double>::quiet_NaN();
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return shutting_down_ || tracking_status_received_ || pause_requested_ ||
                !fault_code_.empty() || !rtkReadyLocked();
          });
          if (tracking_status_received_) {
            tracking = latest_tracking_status_;
            tracking_status_received_ = false;
            received = true;
          }
          edge_clear = latest_hardware_.edge_clear;
          if (latest_tracking_status_.generation == generation) {
            distance_to_target_m = latest_tracking_status_.distance_to_target_m;
            signed_remaining_m = latest_tracking_status_.signed_remaining_m;
          }
        }

        if (received && tracking.finished) {
          machine.complete_tracking(generation);
          publishTrackingRelease(generation);
          clearExpectedTracking();
          return StepResult::kSuccess;
        }
        if (received && tracking.blocked) {
          if (tracking.code == "RTK_INVALID") {
            clearExpectedTracking();
            const auto recovery = waitForRtkRecovery(
                goal_handle, machine, current_loop, total_loops);
            if (recovery == StepResult::kRestart) {
              if (!manage_brush) {
                return StepResult::kRestart;
              }
              restart_tracking = true;
              break;
            }
            return recovery;
          }
          setFault(
              tracking.code.empty() ? "TRACKING_BLOCKED" : tracking.code,
              tracking.message.empty() ? "路径跟踪被阻塞" : tracking.message);
          machine.fail();
          clearExpectedTracking();
          return StepResult::kFailed;
        }

        const auto edge_decision = straight_edge_guard_->evaluate(
            edge_clear,
            monotonicMilliseconds(),
            distance_to_target_m,
            signed_remaining_m);
        if (edge_decision == StraightEdgeDecision::kTargetReached) {
          machine.complete_tracking(generation);
          publishTrackingRelease(generation);
          clearExpectedTracking();
          publishFeedback(
              goal_handle,
              machine,
              current_loop,
              total_loops,
              "SEGMENT_EDGE_REACHED",
              "触边位置已到达或越过目标点，当前直行段完成");
          return StepResult::kSuccess;
        }
        if (edge_decision == StraightEdgeDecision::kConfirmRequired) {
          clearExpectedTracking();
          const auto confirmation = waitForEdgeConfirmation(
              goal_handle,
              machine,
              current_loop,
              total_loops,
              distance_to_target_m,
              signed_remaining_m);
          if (confirmation == StepResult::kRestart) {
            if (!manage_brush) {
              return StepResult::kRestart;
            }
            restart_tracking = true;
            break;
          }
          return confirmation;
        }
      }

      if (restart_tracking) {
        restore_brush = true;
        continue;
      }
      break;
    }
    machine.cancel();
    clearExpectedTracking();
    return StepResult::kCancelled;
  }

  void finishAction(
      const std::shared_ptr<GoalHandle>& goal_handle,
      MissionStateMachine& machine,
      const std::shared_ptr<ExecuteCleaning::Result>& result,
      StepResult step,
      const std::uint32_t completed_segments) {
    publishStoppedOutputs("mission_finished", step == StepResult::kFailed);
    publishSafetyRelease();
    result->completed_segments = completed_segments;

    if (goal_handle->is_canceling() || step == StepResult::kCancelled) {
      machine.cancel();
      result->success = false;
      result->code = "MISSION_CANCELLED";
      result->message = "任务已取消";
      goal_handle->canceled(result);
    } else if (step == StepResult::kSuccess &&
        machine.state() == MissionState::kCompleted) {
      result->success = true;
      result->code = "MISSION_COMPLETED";
      clearCheckpoint("cleaning", "mission_completed");
      result->message = "自动清扫任务已完成";
      goal_handle->succeed(result);
    } else {
      machine.fail();
      result->success = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result->code = fault_code_.empty() ? "MISSION_FAILED" : fault_code_;
        result->message = fault_message_.empty() ? "任务执行失败" : fault_message_;
      }
      goal_handle->abort(result);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    mission_active_ = false;
    preflight_complete_ = false;
    pause_requested_ = false;
    edge_confirmation_required_ = false;
    pause_reason_.clear();
    expected_turn_request_id_ = 0u;
    expected_tracking_generation_ = 0u;
    current_action_ = "idle";
    lifecycle_state_ = result->code;
    lifecycle_message_ = result->message;
    state_current_segment_ = completed_segments;
    condition_.notify_all();
  }

  void finishWaypointAction(
      const std::shared_ptr<WaypointGoalHandle>& goal_handle,
      const std::shared_ptr<NavigateWaypoints::Result>& result,
      const StepResult step,
      const std::uint32_t completed_waypoints) {
    // 正常完成和用户取消只停止车辆，不主动改变独立滚刷开关。
    publishTrackingRelease();
    publishMissionBrake("waypoint_finished");
    if (step == StepResult::kFailed) {
      publishBrush(false, 0, false);
      publishSafetyBrake("waypoint_failed");
    }
    publishSafetyRelease();
    result->completed_waypoints = completed_waypoints;

    if (goal_handle->is_canceling() || step == StepResult::kCancelled) {
      result->success = false;
      result->code = "WAYPOINT_CANCELLED";
      result->message = "多路点导航已取消";
      goal_handle->canceled(result);
    } else if (step == StepResult::kSuccess) {
      result->success = true;
      result->code = "WAYPOINT_COMPLETED";
      result->message = "多路点导航已完成";
      goal_handle->succeed(result);
    } else {
      result->success = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result->code = fault_code_.empty()
            ? "WAYPOINT_FAILED" : fault_code_;
        result->message = fault_message_.empty()
            ? "多路点导航失败" : fault_message_;
      }
      goal_handle->abort(result);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    mission_active_ = false;
    preflight_complete_ = false;
    pause_requested_ = false;
    edge_confirmation_required_ = false;
    pause_reason_.clear();
    expected_turn_request_id_ = 0u;
    expected_tracking_generation_ = 0u;
    waypoint_completed_ = completed_waypoints;
    current_action_ = "idle";
    lifecycle_state_ = result->code;
    lifecycle_message_ = result->message;
    state_current_segment_ = completed_waypoints;
    condition_.notify_all();
  }

  void finishReturnHomeAction(
      const std::shared_ptr<ReturnHomeGoalHandle>& goal_handle,
      MissionStateMachine& machine,
      const std::shared_ptr<ReturnHome::Result>& result,
      const StepResult step) {
    publishTrackingRelease();
    publishBrush(false, 0, false);
    publishMissionBrake("return_home_finished");
    if (step == StepResult::kFailed) {
      publishSafetyBrake("return_home_failed");
    }
    publishSafetyRelease();

    if (goal_handle->is_canceling() || step == StepResult::kCancelled) {
      machine.cancel();
      result->success = false;
      result->code = "RETURN_HOME_CANCELLED";
      result->message = "return-home cancelled";
      clearCheckpoint("return_home", "return_home_cancelled");
      goal_handle->canceled(result);
    } else if (step == StepResult::kSuccess &&
        machine.state() == MissionState::kCompleted) {
      result->success = true;
      result->code = "RETURN_HOME_COMPLETED";
      result->message = goal_handle->get_goal()->dock_after_arrival
          ? "return-home arrived; docking is not implemented yet"
          : "return-home completed";
      clearCheckpoint("return_home", "return_home_completed");
      goal_handle->succeed(result);
    } else {
      machine.fail();
      result->success = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result->code = fault_code_.empty() ? "RETURN_HOME_FAILED" : fault_code_;
        result->message = fault_message_.empty() ? "return-home failed" : fault_message_;
      }
      goal_handle->abort(result);
    }

    resetRuntimeAfterWorker(result->code, result->message);
  }

  void finishRecoverAction(
      const std::shared_ptr<RecoverGoalHandle>& goal_handle,
      MissionStateMachine& machine,
      const std::shared_ptr<RecoverMission::Result>& result,
      const StepResult step,
      const std::string& mission_kind) {
    publishStoppedOutputs("recover_finished", step == StepResult::kFailed);
    publishSafetyRelease();

    if (goal_handle->is_canceling() || step == StepResult::kCancelled) {
      machine.cancel();
      result->success = false;
      result->code = "RECOVERY_CANCELLED";
      result->message = "recovery cancelled";
      goal_handle->canceled(result);
    } else if (step == StepResult::kSuccess &&
        machine.state() == MissionState::kCompleted) {
      result->success = true;
      result->code = "RECOVERY_COMPLETED";
      result->message = "recovered mission completed";
      if (!mission_kind.empty()) {
        clearCheckpoint(mission_kind, "recovery_completed");
      }
      goal_handle->succeed(result);
    } else {
      machine.fail();
      result->success = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result->code = fault_code_.empty() ? "RECOVERY_FAILED" : fault_code_;
        result->message = fault_message_.empty() ? "recovery failed" : fault_message_;
      }
      goal_handle->abort(result);
    }

    resetRuntimeAfterWorker(result->code, result->message);
  }

  void publishTurnCommand(
      const cleanbot_interfaces::msg::TaskSegment& segment,
      const std::uint64_t request_id,
      const bool operator_intent) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = request_id;
    command.request_id = request_id;
    command.source = "mission_turn";
    command.priority = VehicleCommand::PRIORITY_MISSION;
    command.active = true;
    command.operator_intent = operator_intent;
    command.status = 3u;
    command.z_speed = turn_z_speed_;
    command.target_rotation = static_cast<std::int32_t>(
        std::lround(segment.turn_angle_deg * 10.0));
    command.heading_deg = segment.heading_deg;
    command.brake = false;
    mission_command_publisher_->publish(command);
  }

  void publishTrackingTarget(
      const cleanbot_interfaces::msg::TaskSegment& segment,
      const std::uint64_t generation,
      const bool operator_intent) {
    cleanbot_interfaces::msg::TrackingTarget target;
    target.stamp = now();
    target.generation = generation;
    target.active = true;
    target.operator_intent = operator_intent;
    target.segment = segment;
    tracking_target_publisher_->publish(target);
  }

  void publishTrackingRelease(const std::uint64_t generation = 0u) {
    cleanbot_interfaces::msg::TrackingTarget target;
    target.stamp = now();
    target.generation = generation == 0u ? expected_tracking_generation_ : generation;
    target.active = false;
    target.operator_intent = false;
    tracking_target_publisher_->publish(target);
  }

  void publishMissionBrake(const std::string& source) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = nextRequestId();
    command.request_id = command.command_id;
    command.source = source;
    command.priority = VehicleCommand::PRIORITY_MISSION;
    command.active = true;
    command.operator_intent = false;
    command.status = 0u;
    command.brake = true;
    mission_command_publisher_->publish(command);
  }

  void publishMissionRelease() {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = nextRequestId();
    command.request_id = command.command_id;
    command.source = "mission_release";
    command.priority = VehicleCommand::PRIORITY_MISSION;
    command.active = false;
    command.brake = true;
    mission_command_publisher_->publish(command);
  }

  void publishSafetyBrake(const std::string& source) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = nextRequestId();
    command.request_id = command.command_id;
    command.source = source;
    command.priority = VehicleCommand::PRIORITY_SAFETY;
    command.active = true;
    command.operator_intent = false;
    command.status = 0u;
    command.brake = true;
    safety_command_publisher_->publish(command);
  }

  void publishSafetyRelease() {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = nextRequestId();
    command.request_id = command.command_id;
    command.source = "mission_safety_release";
    command.priority = VehicleCommand::PRIORITY_SAFETY;
    command.active = false;
    command.brake = true;
    safety_command_publisher_->publish(command);
  }

  void publishBrushForSegment(
      const cleanbot_interfaces::msg::TaskSegment& segment,
      const std::int32_t requested_speed) {
    const bool cleaning =
        segment.segment_type ==
            cleanbot_interfaces::msg::TaskSegment::SEGMENT_CLEANING ||
        (segment.segment_type ==
             cleanbot_interfaces::msg::TaskSegment::SEGMENT_UNKNOWN &&
         segment.mode == 1u);
    publishBrush(cleaning, cleaning ? requested_speed : 0, true);
  }

  void publishBrush(
      const bool enabled,
      const std::int32_t speed,
      const bool operator_intent) {
    cleanbot_interfaces::msg::BrushCommand command;
    command.stamp = now();
    command.source = "mission";
    command.enabled = enabled;
    command.speed = speed;
    command.operator_intent = operator_intent;
    brush_publisher_->publish(command);
  }

  void publishStoppedOutputs(const std::string& source, const bool safety) {
    publishTrackingRelease();
    publishBrush(false, 0, false);
    publishMissionBrake(source);
    if (safety) {
      publishSafetyBrake(source);
    }
  }

  void publishFeedback(
      const std::shared_ptr<GoalHandle>& goal_handle,
      const MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops,
      const std::string& state,
      const std::string& message) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lifecycle_state_ = state;
      lifecycle_message_ = message;
      state_current_segment_ =
          static_cast<std::uint32_t>(machine.current_segment());
      state_total_segments_ =
          static_cast<std::uint32_t>(machine.total_segments());
    }
    auto feedback = std::make_shared<ExecuteCleaning::Feedback>();
    feedback->current_segment = static_cast<std::uint32_t>(machine.current_segment());
    feedback->total_segments = static_cast<std::uint32_t>(machine.total_segments());
    feedback->current_loop = current_loop;
    const double loop_progress = machine.total_segments() == 0u
        ? 0.0
        : static_cast<double>(machine.current_segment()) /
            static_cast<double>(machine.total_segments());
    feedback->progress = static_cast<float>(
        total_loops == 0u
            ? loop_progress
            : (static_cast<double>(current_loop) + loop_progress) /
                static_cast<double>(total_loops));
    feedback->state = state;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void publishFeedback(
      const std::shared_ptr<WaypointGoalHandle>& goal_handle,
      const MissionStateMachine&,
      const std::uint32_t,
      const std::uint32_t,
      const std::string& state,
      const std::string& message) {
    RCLCPP_DEBUG(get_logger(), "waypoint state=%s detail=%s", state.c_str(), message.c_str());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lifecycle_state_ = state;
      lifecycle_message_ = message;
      state_current_segment_ = waypoint_current_index_;
      state_total_segments_ = waypoint_total_;
    }
    publishWaypointFeedback(goal_handle, state);
  }

  void publishFeedback(
      const std::shared_ptr<ReturnHomeGoalHandle>& goal_handle,
      const MissionStateMachine& machine,
      const std::uint32_t,
      const std::uint32_t,
      const std::string& state,
      const std::string& message) {
    RCLCPP_DEBUG(get_logger(), "return-home state=%s detail=%s", state.c_str(), message.c_str());
    auto feedback = std::make_shared<ReturnHome::Feedback>();
    feedback->state = state;
    feedback->progress = machine.state() == MissionState::kCompleted ? 1.0f : 0.0f;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lifecycle_state_ = state;
      lifecycle_message_ = message;
      state_current_segment_ =
          machine.state() == MissionState::kCompleted ? 1u : 0u;
      state_total_segments_ = 1u;
      feedback->distance_remaining_m =
          std::isfinite(latest_tracking_status_.distance_to_target_m)
              ? latest_tracking_status_.distance_to_target_m
              : 0.0;
    }
    goal_handle->publish_feedback(feedback);
  }

  void publishFeedback(
      const std::shared_ptr<RecoverGoalHandle>& goal_handle,
      const MissionStateMachine& machine,
      const std::uint32_t current_loop,
      const std::uint32_t total_loops,
      const std::string& state,
      const std::string& message) {
    RCLCPP_DEBUG(get_logger(), "recovery state=%s detail=%s", state.c_str(), message.c_str());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lifecycle_state_ = state;
      lifecycle_message_ = message;
      state_current_segment_ =
          static_cast<std::uint32_t>(machine.current_segment());
      state_total_segments_ =
          static_cast<std::uint32_t>(machine.total_segments());
    }
    auto feedback = std::make_shared<RecoverMission::Feedback>();
    feedback->next_segment = static_cast<std::uint32_t>(machine.current_segment());
    feedback->total_segments = static_cast<std::uint32_t>(machine.total_segments());
    feedback->current_loop = current_loop;
    const double loop_progress = machine.total_segments() == 0u
        ? 0.0
        : static_cast<double>(machine.current_segment()) /
            static_cast<double>(machine.total_segments());
    feedback->progress = static_cast<float>(
        total_loops == 0u
            ? loop_progress
            : (static_cast<double>(current_loop) + loop_progress) /
                static_cast<double>(total_loops));
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  void publishWaypointFeedback(
      const std::shared_ptr<WaypointGoalHandle>& goal_handle,
      const std::string& state) {
    auto feedback = std::make_shared<NavigateWaypoints::Feedback>();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback->current_waypoint = waypoint_current_index_;
      feedback->total_waypoints = waypoint_total_;
      feedback->current_loop = waypoint_current_loop_;
      feedback->progress = waypoint_total_targets_ == 0u
          ? 0.0f
          : static_cast<float>(std::min(
              1.0,
              static_cast<double>(waypoint_completed_) /
                  static_cast<double>(waypoint_total_targets_)));
    }
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  bool rtkReadyLocked() const {
    if (!has_rtk_ || !latest_rtk_.fixed_valid || !latest_rtk_.center_valid ||
        !latest_rtk_.heading_valid) {
      return false;
    }
    return ageSeconds(latest_rtk_at_) <= rtk_freshness_timeout_sec_;
  }

  bool hardwareReadyLocked() const {
    if (!has_hardware_ || !latest_hardware_.connected ||
        isLowBattery(latest_hardware_)) {
      return false;
    }
    return ageSeconds(latest_hardware_at_) <= hardware_freshness_timeout_sec_;
  }

  void publishVehicleState() {
    VehicleStateInput input;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input.configured = configured_;
      input.hardware_ready = hardwareReadyLocked();
      input.rtk_ready = rtkReadyLocked();
      input.mission_active = mission_active_;
      input.goal_reserved = goal_reserved_;
      input.has_final_command = has_final_command_;
      input.final_command_active = latest_final_command_.active;
      input.final_command_brake = latest_final_command_.brake;
      input.final_command_priority = latest_final_command_.priority;
      input.current_action = current_action_;
      input.lifecycle_message = lifecycle_message_;
      input.pause_reason = pause_reason_;
      input.fault_code = fault_code_;
      input.fault_message = fault_message_;
      input.battery_percent =
          has_hardware_ ? latest_hardware_.battery_percent
                        : std::numeric_limits<double>::quiet_NaN();
      input.current_segment = state_current_segment_;
      input.total_segments = state_total_segments_;
    }

    const auto snapshot = build_vehicle_state(input);
    cleanbot_interfaces::msg::VehicleState message;
    message.stamp = now();
    message.control_state = snapshot.control_state;
    message.health_state = snapshot.health_state;
    message.fault_state = snapshot.fault_state;
    message.current_action = snapshot.current_action;
    message.message = snapshot.message;
    message.start_ready = snapshot.start_ready;
    message.parking = snapshot.parking;
    message.cleaning = snapshot.cleaning;
    message.rtk_fixed = snapshot.rtk_fixed;
    message.in_garage = snapshot.in_garage;
    message.battery_percent = snapshot.battery_percent;
    message.current_segment = snapshot.current_segment;
    message.total_segments = snapshot.total_segments;
    vehicle_state_publisher_->publish(message);
  }

  bool isLowBattery(
      const cleanbot_interfaces::msg::HardwareStatus& hardware) const {
    return low_battery_threshold_percent_ > 0.0 &&
        hardware.battery_percent > 0.0 &&
        hardware.battery_percent <= low_battery_threshold_percent_;
  }

  static double ageSeconds(
      const std::chrono::steady_clock::time_point& received_at) {
    if (received_at == std::chrono::steady_clock::time_point{}) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - received_at).count();
  }

  static std::uint64_t monotonicMilliseconds() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  static std::int64_t wallClockMilliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
  }

  void resetRuntimeAfterWorker(
      const std::string& terminal_state,
      const std::string& terminal_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    mission_active_ = false;
    preflight_complete_ = false;
    pause_requested_ = false;
    edge_confirmation_required_ = false;
    pause_reason_.clear();
    expected_turn_request_id_ = 0u;
    expected_tracking_generation_ = 0u;
    turn_status_received_ = false;
    tracking_status_received_ = false;
    current_action_ = "idle";
    lifecycle_state_ = terminal_state;
    lifecycle_message_ = terminal_message;
    condition_.notify_all();
  }

  std::string makeRunId(const std::string& prefix) {
    std::ostringstream stream;
    stream << prefix << "-" << now().nanoseconds() << "-" << nextRequestId();
    return stream.str();
  }

  MissionCheckpointStore* checkpointStoreForKind(const std::string& mission_kind) const {
    if (mission_kind == "cleaning") {
      return cleaning_checkpoint_store_.get();
    }
    if (mission_kind == "return_home") {
      return return_home_checkpoint_store_.get();
    }
    return nullptr;
  }

  std::optional<MissionCheckpointRecord> loadRecoveryCheckpoint(
      const std::string& requested_run_id,
      std::string* error) const {
    std::string return_error;
    std::string cleaning_error;
    const auto return_home = return_home_checkpoint_store_->load_latest(&return_error);
    const auto cleaning = cleaning_checkpoint_store_->load_latest(&cleaning_error);

    if (!requested_run_id.empty()) {
      if (return_home.has_value() && return_home->run_id == requested_run_id) {
        return return_home;
      }
      if (cleaning.has_value() && cleaning->run_id == requested_run_id) {
        return cleaning;
      }
      if (error != nullptr) {
        *error = "requested run_id does not match a saved mission checkpoint";
      }
      return std::nullopt;
    }

    // Return-home is transient but takes precedence when both records exist:
    // after a power loss, the vehicle must first finish the active safety return.
    if (return_home.has_value()) {
      return return_home;
    }
    if (cleaning.has_value()) {
      return cleaning;
    }
    if (error != nullptr) {
      *error = !return_error.empty() ? return_error : cleaning_error;
      if (error->empty()) {
        *error = "no recoverable mission checkpoint";
      }
    }
    return std::nullopt;
  }

  bool saveCheckpoint(MissionCheckpointRecord& checkpoint, const std::string& state) {
    checkpoint.state = state;
    const auto now_ms = wallClockMilliseconds();
    if (checkpoint.created_at_ms == 0) {
      checkpoint.created_at_ms = now_ms;
    }
    checkpoint.updated_at_ms = now_ms;
    auto* store = checkpointStoreForKind(checkpoint.mission_kind);
    if (store == nullptr) {
      RCLCPP_WARN(get_logger(), "checkpoint store is not ready; state=%s", state.c_str());
      return false;
    }
    std::string error;
    if (!store->save(checkpoint, &error)) {
      RCLCPP_WARN(
          get_logger(),
          "failed to save mission checkpoint state=%s error=%s",
          state.c_str(),
          error.c_str());
      return false;
    }
    return true;
  }

  void clearCheckpoint(const std::string& mission_kind, const std::string& source) {
    auto* store = checkpointStoreForKind(mission_kind);
    if (store == nullptr) {
      return;
    }
    std::string error;
    if (!store->clear(&error)) {
      RCLCPP_WARN(
          get_logger(),
          "failed to clear mission checkpoint source=%s error=%s",
          source.c_str(),
          error.c_str());
    }
  }

  void clearExpectedTurn() {
    std::lock_guard<std::mutex> lock(mutex_);
    expected_turn_request_id_ = 0u;
    turn_status_received_ = false;
  }

  void clearExpectedTracking() {
    std::lock_guard<std::mutex> lock(mutex_);
    expected_tracking_generation_ = 0u;
    tracking_status_received_ = false;
  }

  void setFault(const std::string& code, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    setFaultLocked(code, message);
    condition_.notify_all();
  }

  void setFaultLocked(const std::string& code, const std::string& message) {
    if (fault_code_.empty()) {
      fault_code_ = code;
      fault_message_ = message;
    }
  }

  std::uint64_t nextRequestId() {
    std::lock_guard<std::mutex> lock(id_mutex_);
    return next_request_id_++;
  }

  std::uint64_t nextTrackingGeneration() {
    std::lock_guard<std::mutex> lock(id_mutex_);
    return next_tracking_generation_++;
  }

  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initial && configured_) {
      RCLCPP_WARN(
          get_logger(),
          "mission configuration changed; restart required before it becomes active");
      return;
    }
    rtk_recovery_timeout_sec_ =
        snapshot.get_double("mission.rtk_recovery_timeout_sec");
    command_completion_timeout_sec_ =
        snapshot.get_double("mission.command_completion_timeout_sec");
    turn_heartbeat_ms_ = snapshot.get_integer("mission.turn_heartbeat_ms");
    turn_z_speed_ = static_cast<std::int32_t>(
        snapshot.get_integer("mission.turn_z_speed"));
    turn_angle_tolerance_deg_ =
        snapshot.get_double("mission.turn_angle_tolerance_deg");
    low_battery_threshold_percent_ =
        snapshot.get_double("mission.low_battery_threshold_percent");
    rtk_freshness_timeout_sec_ =
        snapshot.get_double("mission.rtk_freshness_timeout_sec");
    hardware_freshness_timeout_sec_ =
        snapshot.get_double("mission.hardware_freshness_timeout_sec");
    edge_debounce_ms_ = snapshot.get_integer("mission.edge_debounce_ms");
    edge_target_tolerance_m_ =
        snapshot.get_double("mission.edge_target_tolerance_m");
    checkpoint_path_ = snapshot.get_string("mission.checkpoint_path");
    cleaning_checkpoint_store_ =
        std::make_unique<MissionCheckpointStore>(checkpoint_path_);
    return_home_checkpoint_store_ = std::make_unique<MissionCheckpointStore>(
        checkpoint_path_.string() + ".return_home");
    straight_edge_guard_ = std::make_unique<StraightEdgeGuard>(
        static_cast<std::uint64_t>(edge_debounce_ms_),
        edge_target_tolerance_m_);
    configured_ = true;
    RCLCPP_INFO(get_logger(), "mission configuration ready");
  }

  double rtk_recovery_timeout_sec_{30.0};
  double command_completion_timeout_sec_{20.0};
  std::int64_t turn_heartbeat_ms_{250};
  std::int32_t turn_z_speed_{0};
  double turn_angle_tolerance_deg_{0.5};
  double low_battery_threshold_percent_{0.0};
  double rtk_freshness_timeout_sec_{2.0};
  double hardware_freshness_timeout_sec_{1.0};
  std::int64_t edge_debounce_ms_{200};
  double edge_target_tolerance_m_{0.10};
  std::unique_ptr<StraightEdgeGuard> straight_edge_guard_;
  std::unique_ptr<config::ConfigClient> config_client_;
  std::filesystem::path checkpoint_path_;
  std::unique_ptr<MissionCheckpointStore> cleaning_checkpoint_store_;
  std::unique_ptr<MissionCheckpointStore> return_home_checkpoint_store_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool shutting_down_{false};
  bool configured_{false};
  bool mission_active_{false};
  bool preflight_complete_{false};
  bool goal_reserved_{false};
  bool pause_requested_{false};
  bool edge_confirmation_required_{false};
  std::string pause_reason_;
  std::string fault_code_;
  std::string fault_message_;
  bool has_rtk_{false};
  bool has_hardware_{false};
  cleanbot_interfaces::msg::RtkFix latest_rtk_;
  cleanbot_interfaces::msg::HardwareStatus latest_hardware_;
  std::chrono::steady_clock::time_point latest_rtk_at_;
  std::chrono::steady_clock::time_point latest_hardware_at_;
  std::uint64_t expected_turn_request_id_{0u};
  std::uint64_t expected_tracking_generation_{0u};
  bool turn_status_received_{false};
  bool tracking_status_received_{false};
  CommandExecutionStatus latest_turn_status_;
  TrackingStatus latest_tracking_status_;
  bool has_final_command_{false};
  VehicleCommand latest_final_command_;
  std::string current_action_{"idle"};
  std::string lifecycle_state_{"IDLE"};
  std::string lifecycle_message_{"vehicle is idle"};
  std::uint32_t state_current_segment_{0u};
  std::uint32_t state_total_segments_{0u};

  std::mutex id_mutex_;
  std::uint64_t next_request_id_{1u};
  std::uint64_t next_tracking_generation_{1u};
  std::thread worker_;

  std::uint32_t waypoint_current_index_{0u};
  std::uint32_t waypoint_total_{0u};
  std::uint32_t waypoint_completed_{0u};
  std::uint64_t waypoint_total_targets_{0u};
  std::uint32_t waypoint_current_loop_{0u};
  double waypoint_target_lat_{0.0};
  double waypoint_target_lon_{0.0};

  rclcpp_action::Server<ExecuteCleaning>::SharedPtr action_server_;
  rclcpp_action::Server<NavigateWaypoints>::SharedPtr waypoint_action_server_;
  rclcpp_action::Server<ReturnHome>::SharedPtr return_home_action_server_;
  rclcpp_action::Server<RecoverMission>::SharedPtr recover_action_server_;
  rclcpp::Service<cleanbot_interfaces::srv::SetMissionPause>::SharedPtr pause_service_;
  rclcpp::Publisher<cleanbot_interfaces::msg::TrackingTarget>::SharedPtr
      tracking_target_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr mission_command_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::BrushCommand>::SharedPtr
      brush_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr safety_command_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::VehicleState>::SharedPtr
      vehicle_state_publisher_;
  rclcpp::Subscription<TrackingStatus>::SharedPtr tracking_status_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::RtkFix>::SharedPtr rtk_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::HardwareStatus>::SharedPtr
      hardware_subscription_;
  rclcpp::Subscription<CommandExecutionStatus>::SharedPtr
      command_status_subscription_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr final_command_subscription_;
  rclcpp::TimerBase::SharedPtr vehicle_state_timer_;
};

}  // namespace mission
}  // namespace cleanbot

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::mission::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
