#include "cleanbot_hardware/gateway_readiness.hpp"

namespace cleanbot {
namespace hardware {

// 串口连接后开始安全恢复握手，等待指定刹车命令ACK和一帧有效状态。
void GatewayReadiness::on_connected(const std::uint16_t brake_sequence) {
  brake_sequence_ = brake_sequence;
  brake_acknowledged_ = false;
  valid_status_seen_ = false;
  state_ = GatewayReadinessState::kWaitingBrakeAck;
}

// 串口断开后清除本轮握手的全部状态，禁止普通运动命令。
void GatewayReadiness::on_disconnected() {
  brake_sequence_ = 0u;
  brake_acknowledged_ = false;
  valid_status_seen_ = false;
  state_ = GatewayReadinessState::kDisconnected;
}

// 仅接受当前重连刹车的A1结果；匹配成功后更新是否已确认刹车。
bool GatewayReadiness::observe_command_ack(
    const std::uint16_t sequence,
    const std::uint8_t command_type,
    const bool accepted) {
  if (state_ == GatewayReadinessState::kDisconnected ||
      sequence != brake_sequence_ || command_type != 0u || !accepted) {
    return false;
  }
  brake_acknowledged_ = true;
  update_state();
  return true;
}

// 记录重连后已经收到有效状态帧，并尝试推进到就绪状态。
bool GatewayReadiness::observe_valid_status() {
  if (state_ == GatewayReadinessState::kDisconnected) {
    return false;
  }
  valid_status_seen_ = true;
  update_state();
  return true;
}

// 只有刹车ACK和有效状态帧两个条件同时满足时才返回true。
bool GatewayReadiness::ready() const {
  return state_ == GatewayReadinessState::kReady;
}

// 返回当前恢复握手阶段。
GatewayReadinessState GatewayReadiness::state() const { return state_; }

// 返回当前恢复握手绑定的刹车命令sequence。
std::uint16_t GatewayReadiness::brake_sequence() const { return brake_sequence_; }

// 根据已收集的握手证据设置等待ACK、等待状态或完全就绪。
void GatewayReadiness::update_state() {
  if (brake_acknowledged_ && valid_status_seen_) {
    state_ = GatewayReadinessState::kReady;
  } else if (brake_acknowledged_) {
    state_ = GatewayReadinessState::kWaitingStatus;
  } else {
    state_ = GatewayReadinessState::kWaitingBrakeAck;
  }
}

}  // namespace hardware
}  // namespace cleanbot
