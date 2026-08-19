/*
 * 文件作用：RTK直线路径跟踪节点。
 *
 * 输入：订阅 /tracking/target 获取当前目标线段，订阅 /rtk/fix 获取实时车体中心坐标和航向。
 * 输出：发布 /control/mission_cmd 任务控制命令、/tracking/debug 纠偏数据和
 *      /tracking/status 跟踪状态。
 *
 * 主流程：接收目标线段 -> 每帧RTK触发计算 -> Kalman坐标滤波 -> 计算航向误差和横向偏差
 *          -> 生成x_speed/z_speed -> 判断到点 -> 发布任务命令或刹车命令。
 * 边界：本节点只生成任务来源的控制建议，不能直接访问串口；最终命令必须经过命令仲裁节点。
 */

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_control/tracking_core.hpp"
#include "cleanbot_interfaces/msg/rtk_fix.hpp"
#include "cleanbot_interfaces/msg/tracking_debug.hpp"
#include "cleanbot_interfaces/msg/tracking_status.hpp"
#include "cleanbot_interfaces/msg/tracking_target.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace control {

class TrackingNode : public rclcpp::Node {
 public:
  TrackingNode() : Node("tracking_node") {
    // 1. 建立输出主题：任务命令交给仲裁器，调试和状态主题用于监控与上层业务判断。
    command_publisher_ = create_publisher<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/mission_cmd", common::latest_command_qos());
    debug_publisher_ = create_publisher<cleanbot_interfaces::msg::TrackingDebug>(
        "/tracking/debug", common::debug_qos());
    status_publisher_ = create_publisher<cleanbot_interfaces::msg::TrackingStatus>(
        "/tracking/status", common::status_qos());

    // 2. 目标变化负责启停路径段；每一帧RTK定位数据负责驱动一次实时纠偏计算。
    target_subscription_ = create_subscription<cleanbot_interfaces::msg::TrackingTarget>(
        "/tracking/target",
        common::latest_command_qos(),
        std::bind(&TrackingNode::onTarget, this, std::placeholders::_1));
    rtk_subscription_ = create_subscription<cleanbot_interfaces::msg::RtkFix>(
        "/rtk/fix",
        common::rtk_fix_qos(),
        std::bind(&TrackingNode::onRtkFix, this, std::placeholders::_1));

    // 3. 所有业务参数从配置中心读取；只有基础前进速度允许运行时立即生效。
    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "motion.base_forward_speed",
            "tracking.process_noise",
            "tracking.measurement_noise",
            "tracking.max_dt",
            "tracking.heading_gain",
            "tracking.cte_gain",
            "tracking.short_range_heading_limit_deg",
            "tracking.max_z_speed",
            "tracking.target_tolerance_m",
            "tracking.overshoot_cte_tolerance_m",
        },
        false,
        // 配置回调作用：接收最新配置快照，并刷新本节点对应的运行参数。
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

 private:
  // 路径目标回调：切换新线段时保存目标并重置滤波器；取消目标时发布释放和刹车语义。
  void onTarget(const cleanbot_interfaces::msg::TrackingTarget::SharedPtr target) {
    if (!configured_ || !filter_ || !controller_) {
      publishStatus(
          false, false, true, 0.0,
          std::numeric_limits<double>::quiet_NaN(),
          "CONFIG_NOT_READY", "tracking configuration is not ready");
      return;
    }
    if (!target->active) {
      // 目标被取消后不能继续沿用上一段滤波状态或控制输出。
      target_active_ = false;
      pending_operator_intent_ = false;
      filter_->reset();
      publishRelease();
      publishStatus(
          false,
          false,
          false,
          last_distance_to_target_,
          last_signed_remaining_,
          "TARGET_RELEASED",
          "");
      return;
    }

    target_ = *target;
    target_active_ = true;
    // operator_intent只随新目标的第一条任务命令发布，用于仲裁器识别用户的新操作意图。
    pending_operator_intent_ = target->operator_intent;
    last_distance_to_target_ = 0.0;
    last_signed_remaining_ = std::numeric_limits<double>::quiet_NaN();
    filter_->reset();
    publishStatus(
        true,
        false,
        false,
        0.0,
        last_signed_remaining_,
        "TRACKING_ACTIVE",
        "");
  }

  // RTK回调是纠偏主循环：只有存在活动目标时，每收到一帧有效RTK才计算并发布一次控制量。
  void onRtkFix(const cleanbot_interfaces::msg::RtkFix::SharedPtr fix) {
    if (!configured_ || !filter_ || !controller_ || !target_active_) {
      return;
    }
    if (!fix->fixed_valid || !fix->center_valid || !fix->heading_valid) {
      // 定位、车体中心或航向任一无效时立即发布任务刹车，禁止使用旧位置继续行驶。
      auto command = missionCommandBase();
      command.source = "tracking_rtk_invalid";
      command.status = 0u;
      command.brake = true;
      publishMission(command);
      publishStatus(
          true,
          false,
          true,
          last_distance_to_target_,
          last_signed_remaining_,
          "RTK_INVALID",
          "RTK fixed vehicle-center position is unavailable");
      return;
    }

    const auto& segment = target_.segment;
    // build_tracking_command内部依次执行坐标滤波和直线P控制，输出距离、CTE、航向误差和z_speed。
    const auto tracking = build_tracking_command(
        *filter_,
        *controller_,
        segment.start_lat,
        segment.start_lon,
        segment.end_lat,
        segment.end_lon,
        fix->lat,
        fix->lon,
        fix->heading_deg,
        segment.heading_deg,
        monotonicSeconds());
    last_distance_to_target_ = tracking.distance_to_target_m;
    last_signed_remaining_ = tracking.signed_remaining_m;
    publishDebug(tracking, fix->heading_deg, segment.heading_deg);

    // 到达目标容差，或者越过终点且横向偏差可接受时，结束当前路径段并主动刹车。
    if (should_finish_point_to_point(
            tracking.distance_to_target_m,
            tracking.signed_remaining_m,
            tracking.cte_m,
            parameters_.target_tolerance_m,
            parameters_.overshoot_cte_tolerance_m)) {
      auto command = missionCommandBase();
      command.source = "tracking_complete";
      command.status = 0u;
      command.brake = true;
      publishMission(command);
      target_active_ = false;
      publishStatus(
          false,
          true,
          false,
          tracking.distance_to_target_m,
          tracking.signed_remaining_m,
          "TRACKING_COMPLETE",
          "");
      return;
    }

    // 路径未结束时保持目标前进速度，并把实时纠偏z_speed作为任务命令交给仲裁器。
    auto command = missionCommandBase();
    command.source = "rtk_straight_line_tracking";
    command.status = 1u;
    command.x_speed = base_forward_speed_;
    command.z_speed = tracking.z_speed;
    command.heading_deg = segment.heading_deg;
    command.brake = false;
    publishMission(command);
    publishStatus(
        true,
        false,
        false,
        tracking.distance_to_target_m,
        tracking.signed_remaining_m,
        "TRACKING_RUNNING",
        "");
  }

  // 构造任务命令公共字段：分配command_id并声明固定的任务优先级。
  cleanbot_interfaces::msg::VehicleCommand missionCommandBase() {
    cleanbot_interfaces::msg::VehicleCommand command;
    command.stamp = now();
    command.command_id = next_command_id_++;
    command.priority = cleanbot_interfaces::msg::VehicleCommand::PRIORITY_MISSION;
    command.active = true;
    command.operator_intent = pending_operator_intent_;
    command.brush_speed = 0;
    return command;
  }

  // 每个新目标的operator_intent只发布一次，后续RTK高频命令只刷新任务控制租约。
  void publishMission(cleanbot_interfaces::msg::VehicleCommand& command) {
    command_publisher_->publish(command);
    pending_operator_intent_ = false;
  }

  // 发布active=false通知仲裁器释放任务来源，同时携带刹车保证释放瞬间无残留运动输出。
  void publishRelease() {
    auto command = missionCommandBase();
    command.source = "tracking_release";
    command.active = false;
    command.brake = true;
    publishMission(command);
  }

  // 发布原始/滤波坐标、误差和控制输出，供日志、界面和真车调参使用。
  void publishDebug(
      const TrackingCommand& tracking,
      const double current_heading,
      const double target_heading) {
    cleanbot_interfaces::msg::TrackingDebug debug;
    debug.stamp = now();
    debug.raw_lat = tracking.raw_lat;
    debug.raw_lon = tracking.raw_lon;
    debug.filtered_lat = tracking.filtered_lat;
    debug.filtered_lon = tracking.filtered_lon;
    debug.target_heading_deg = target_heading;
    debug.current_heading_deg = current_heading;
    debug.heading_error_deg = tracking.heading_error_deg;
    debug.cte_m = tracking.cte_m;
    debug.distance_to_target_m = tracking.distance_to_target_m;
    debug.signed_remaining_m = tracking.signed_remaining_m;
    debug.z_speed = tracking.z_speed;
    debug.control_source = tracking.source;
    debug_publisher_->publish(debug);
  }

  // 发布当前跟踪代次、完成/阻塞状态、目标距离和业务说明。
  void publishStatus(
      const bool active,
      const bool finished,
      const bool blocked,
      const double distance,
      const double signed_remaining,
      const std::string& code,
      const std::string& message) {
    cleanbot_interfaces::msg::TrackingStatus status;
    status.stamp = now();
    status.generation = target_.generation;
    status.active = active;
    status.finished = finished;
    status.blocked = blocked;
    status.distance_to_target_m = distance;
    status.signed_remaining_m = signed_remaining;
    status.code = code;
    status.message = message;
    status_publisher_->publish(status);
  }

  // 从配置快照加载跟踪参数；运行期仅立即更新基础前进速度。
  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    base_forward_speed_ = static_cast<std::int32_t>(
        snapshot.get_integer("motion.base_forward_speed"));
    if (!initial && filter_ && controller_) {
      RCLCPP_INFO(
          get_logger(), "base forward speed updated immediately to %d",
          base_forward_speed_);
      return;
    }
    parameters_.process_noise = snapshot.get_double("tracking.process_noise");
    parameters_.measurement_noise = snapshot.get_double("tracking.measurement_noise");
    parameters_.max_dt = snapshot.get_double("tracking.max_dt");
    parameters_.heading_gain = snapshot.get_double("tracking.heading_gain");
    parameters_.cte_gain = snapshot.get_double("tracking.cte_gain");
    parameters_.short_range_heading_limit_deg =
        snapshot.get_double("tracking.short_range_heading_limit_deg");
    parameters_.max_z_speed = static_cast<int>(
        snapshot.get_integer("tracking.max_z_speed"));
    parameters_.target_tolerance_m =
        snapshot.get_double("tracking.target_tolerance_m");
    parameters_.overshoot_cte_tolerance_m =
        snapshot.get_double("tracking.overshoot_cte_tolerance_m");
    filter_ = std::make_unique<RtkKalmanFilter2D>(parameters_);
    controller_ = std::make_unique<StraightLinePController>(parameters_);
    configured_ = true;
    RCLCPP_INFO(
        get_logger(), "tracking configuration ready base_forward_speed=%d",
        base_forward_speed_);
  }

  // 返回用于滤波时间步长计算的单调秒计时。
  static double monotonicSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  TrackingParameters parameters_;
  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<RtkKalmanFilter2D> filter_;
  std::unique_ptr<StraightLinePController> controller_;
  bool configured_{false};
  std::int32_t base_forward_speed_{0};
  cleanbot_interfaces::msg::TrackingTarget target_;
  bool target_active_{false};
  bool pending_operator_intent_{false};
  double last_distance_to_target_{0.0};
  double last_signed_remaining_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t next_command_id_{1u};
  rclcpp::Publisher<cleanbot_interfaces::msg::VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::TrackingDebug>::SharedPtr debug_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::TrackingStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<cleanbot_interfaces::msg::TrackingTarget>::SharedPtr target_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::RtkFix>::SharedPtr rtk_subscription_;
};

}  // namespace control
}  // namespace cleanbot

// 程序入口：初始化ROS2并运行纠偏节点，目标和RTK订阅回调共同驱动路径跟踪。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::control::TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
