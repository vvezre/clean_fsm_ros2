#ifndef CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_

#include <cstdint>
#include <filesystem>
#include <string>

#include "cleanbot_mission/maintenance_gate.hpp"
#include "cleanbot_mission/maintenance_store.hpp"

namespace cleanbot {
namespace mission {

struct MaintenanceRuntimeSnapshot {
  bool initialized{false};
  bool store_fault{false};
  bool admission_closed{true};
  std::uint64_t generation{0u};
  std::uint64_t last_generation{0u};
  bool gate_active{false};
  bool mission_idle{false};
  bool command_gate_applied{false};
  bool brake_acknowledged{false};
  bool hardware_fresh{false};
  bool linear_speed_zero{false};
  bool angular_speed_zero{false};
  bool brush_off{false};
  bool ready{false};
  std::string requester;
  std::string reason;
  std::string phase;
  std::string blocker_code;
  std::string message;
};

class MaintenanceRuntime {
 public:
  explicit MaintenanceRuntime(std::filesystem::path state_path);

  MaintenanceRuntime(const MaintenanceRuntime&) = delete;
  MaintenanceRuntime& operator=(const MaintenanceRuntime&) = delete;
  MaintenanceRuntime(MaintenanceRuntime&&) = delete;
  MaintenanceRuntime& operator=(MaintenanceRuntime&&) = delete;

  // Initialization is single-shot. Repeated calls return the latched result
  // without rereading persistent state or mutating the restored gate.
  bool initialize(bool mission_idle) noexcept;
  void setMissionIdle(bool mission_idle) noexcept;

  const MaintenanceRuntimeSnapshot& snapshot() const noexcept;
  bool initialized() const noexcept;
  bool storeFault() const noexcept;
  bool admissionClosed() const noexcept;
  std::uint64_t lastGeneration() const noexcept;

 private:
  static MaintenanceRuntimeSnapshot initialSnapshot();
  static MaintenanceRuntimeSnapshot emergencyFaultSnapshot();
  static bool validRecord(const MaintenanceStoreRecord& record) noexcept;
  static const char* storeCodeName(MaintenanceStoreCode code) noexcept;
  static void swapSnapshots(
      MaintenanceRuntimeSnapshot& lhs,
      MaintenanceRuntimeSnapshot& rhs) noexcept;

  bool restore(
      const MaintenanceStoreRecord& record,
      bool mission_idle) noexcept;
  bool refreshHealthySnapshot(
      const MaintenanceStoreRecord& record) noexcept;
  bool refreshHealthySnapshot(
      const std::string& requester,
      const std::string& reason) noexcept;
  void latchStoreFault(const std::string& message) noexcept;
  void latchEmergencyFault() noexcept;

  MaintenanceStore store_;
  MaintenanceGate gate_;
  bool initialized_{false};
  bool store_fault_{false};
  bool admission_closed_{true};
  MaintenanceRuntimeSnapshot snapshot_;
  MaintenanceRuntimeSnapshot emergency_fault_snapshot_;
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
