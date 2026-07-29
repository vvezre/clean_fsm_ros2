#ifndef CLEANBOT_HARDWARE__GATEWAY_READINESS_HPP_
#define CLEANBOT_HARDWARE__GATEWAY_READINESS_HPP_

#include <cstdint>

namespace cleanbot {
namespace hardware {

enum class GatewayReadinessState {
  kDisconnected = 0,
  kWaitingBrakeAck = 1,
  kWaitingStatus = 2,
  kReady = 3,
};

class GatewayReadiness {
 public:
  /// 串口连接后进入恢复握手，记录本次强制刹车命令的sequence。
  void on_connected(std::uint16_t brake_sequence);
  /// 串口断开后清除握手证据并回到未连接状态。
  void on_disconnected();
  /// 观察A1命令确认；仅匹配本次重连刹车的ACK，并记录是否被接受。
  bool observe_command_ack(
      std::uint16_t sequence, std::uint8_t command_type, bool accepted);
  /// 记录已经收到一帧可解析的下位机状态，用作恢复握手的第二个条件。
  bool observe_valid_status();

  /// 判断刹车ACK和有效状态帧是否均已收到，网关是否允许普通运动命令。
  bool ready() const;
  /// 返回当前网关恢复状态，供日志和诊断使用。
  GatewayReadinessState state() const;
  /// 返回本次重连刹车命令的sequence。
  std::uint16_t brake_sequence() const;

 private:
  /// 根据连接、刹车ACK和状态帧证据重新计算当前恢复状态。
  void update_state();

  GatewayReadinessState state_{GatewayReadinessState::kDisconnected};
  std::uint16_t brake_sequence_{0u};
  bool brake_acknowledged_{false};
  bool valid_status_seen_{false};
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__GATEWAY_READINESS_HPP_
