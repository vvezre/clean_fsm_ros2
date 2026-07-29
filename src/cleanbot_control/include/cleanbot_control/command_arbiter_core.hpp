#ifndef CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_
#define CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_

#include <cstdint>
#include <string>

namespace cleanbot {
namespace control {

enum class CommandSource {
  kEmergency = 0,
  kSafety = 1,
  kManual = 2,
  kMission = 3,
  kVision = 4,
};

struct ArbiterParameters {
  std::uint64_t manual_lease_ms{800u};
  std::uint64_t mission_lease_ms{1000u};
  std::uint64_t vision_lease_ms{500u};
};

struct ControlCommand {
  std::uint64_t stamp_ms{0u};
  std::uint64_t command_id{0u};
  std::uint64_t request_id{0u};
  std::string source;
  std::uint8_t priority{0u};
  bool active{false};
  bool operator_intent{false};
  std::uint8_t status{0u};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t steering_offset{0};
  std::int32_t brush_speed{0};
  std::int32_t target_distance{0};
  std::int32_t target_rotation{0};
  double heading_deg{0.0};
  bool brake{false};
  bool charge{false};
};

struct CommandSlot {
  ControlCommand command;
  std::uint64_t received_at_ms{0u};
  bool present{false};
};

class MaintenanceGateCache {
 public:
  bool update(bool active, std::uint64_t generation);
  bool has_state() const;
  bool active() const;
  std::uint64_t generation() const;

 private:
  bool has_state_{false};
  bool active_{false};
  std::uint64_t generation_{0u};
};

class CommandArbiterCore {
 public:
  explicit CommandArbiterCore(const ArbiterParameters& parameters = ArbiterParameters());

  bool update(CommandSource source, const ControlCommand& command, std::uint64_t now_ms);
  void set_brush(bool enabled, std::int32_t speed, bool operator_intent);
  bool set_maintenance(bool active, std::uint64_t generation);
  bool maintenance_active() const;
  std::uint64_t maintenance_generation() const;
  ControlCommand output(std::uint64_t now_ms);
  bool software_stopped() const;

 private:
  CommandSlot& slot(CommandSource source);
  bool slotActive(
      const CommandSlot& slot, std::uint64_t lease_ms, std::uint64_t now_ms) const;
  void clearOperatorSlots();
  void clearAllSlots();
  void clearMaintenanceInputs();
  bool isNewerThanStop(const ControlCommand& command) const;
  static ControlCommand brakingOutput(const std::string& source);
  static ControlCommand maintenanceOutput(std::uint64_t generation);

  ArbiterParameters parameters_;
  CommandSlot emergency_;
  CommandSlot safety_;
  CommandSlot manual_;
  CommandSlot mission_;
  CommandSlot vision_;
  bool software_stopped_{false};
  std::uint64_t stop_boundary_command_id_{0u};
  std::uint64_t stop_boundary_stamp_ms_{0u};
  CommandSource operator_mode_{CommandSource::kMission};
  bool has_operator_mode_{false};
  bool brush_enabled_{false};
  std::int32_t brush_speed_{0};
  MaintenanceGateCache maintenance_gate_;
};

}  // namespace control
}  // namespace cleanbot

#endif  // CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_
