#ifndef CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace cleanbot {
namespace mission {

struct FinalCommandEvidence {
  std::uint64_t generation{0u};
  std::uint64_t request_id{0u};
  std::uint64_t command_id{0u};
  std::string source;
  bool active{false};
  bool brake{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

struct CommandStatusEvidence {
  std::uint64_t generation{0u};
  std::uint64_t request_id{0u};
  std::uint64_t command_id{0u};
  std::string source;
  std::uint8_t state{0u};
};

struct HardwareEvidence {
  // Nonzero session number, constant within a publisher session and strictly
  // increasing when Task5 replaces a publisher. Retired epochs are ignored.
  std::uint64_t publisher_epoch{0u};
  std::uint64_t frame_sequence{0u};
  bool fresh{false};
  bool connected{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

struct MaintenanceGateSnapshot {
  std::uint64_t generation{0u};
  bool gate_active{false};
  bool mission_idle{false};
  bool command_gate_applied{false};
  bool brake_acknowledged{false};
  bool hardware_fresh{false};
  bool linear_speed_zero{false};
  bool angular_speed_zero{false};
  bool brush_off{false};
  bool ready{false};
  std::string phase;
  std::string blocker_code;
  std::string message;
};

class MaintenanceGate {
 public:
  bool request(std::uint64_t generation, bool mission_idle);
  bool release(std::uint64_t generation);

  void setMissionIdle(bool mission_idle);
  void observeFinalCommand(const FinalCommandEvidence& command);
  void observeCommandStatus(const CommandStatusEvidence& status);
  void observeHardware(const HardwareEvidence& hardware);

  MaintenanceGateSnapshot snapshot() const;
  std::uint64_t lastGeneration() const;
  std::size_t pendingStatusCapacity() const;

 private:
  enum class BrakeFailure {
    kNone,
    kRejected,
    kTimedOut,
    kTransportLost,
    kSuperseded,
    kCorrelationOverflow,
  };

  struct PendingCommandStatus {
    bool acknowledged{false};
    BrakeFailure failure{BrakeFailure::kNone};
  };

  static constexpr std::size_t kPendingStatusCapacity = 16u;

  bool matches(
      std::uint64_t generation,
      std::uint64_t request_id,
      const std::string& source) const;
  static BrakeFailure failureForState(std::uint8_t state);
  void rememberPendingStatus(
      std::uint64_t command_id,
      bool acknowledged,
      BrakeFailure failure);
  void applyPendingStatus(std::uint64_t command_id);
  void acknowledgeCurrentCommand();
  void latchFailure(BrakeFailure failure);
  void resetEvidence();
  void resetHardwareConfirmation();
  void resetHardwareEvidence();

  bool active_{false};
  std::uint64_t generation_{0u};
  std::uint64_t last_generation_{0u};
  bool mission_idle_{false};
  bool command_gate_applied_{false};
  std::uint64_t current_command_id_{0u};
  bool brake_acknowledged_{false};
  BrakeFailure brake_failure_{BrakeFailure::kNone};
  std::map<std::uint64_t, PendingCommandStatus> pending_command_statuses_;

  bool hardware_sample_observed_{false};
  bool hardware_fresh_{false};
  bool linear_speed_zero_{false};
  bool angular_speed_zero_{false};
  bool brush_off_{false};
  bool has_publisher_epoch_{false};
  std::uint64_t publisher_epoch_{0u};
  bool has_last_frame_sequence_{false};
  std::uint64_t last_frame_sequence_{0u};
  std::uint8_t consecutive_zero_frames_{0u};
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_
